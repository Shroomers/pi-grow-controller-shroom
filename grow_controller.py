#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grow tent controller — robust + UV schedule + CSV logging
- LOW_TRIGGERED_RELAYS (active-low support)
- Safe GPIO init (initial=RELAY_OFF)
- Robust sensor read (try/except + None-guard)
- Timestamped logging
- Anti-chatter (min on/off per output)
- CO2 EMA smoothing + validity guard
- CSV telemetry logging
- NEW: UV lamp on IN5, 8x per day 15 min (total 120 min/day), every 3 hours starting 00:00
"""

import time
import datetime as dt
import board
import RPi.GPIO as GPIO
import adafruit_scd4x
import csv, os

# -------------------- Config --------------------
LOW_TRIGGERED_RELAYS = True
RELAY_ON  = GPIO.LOW  if LOW_TRIGGERED_RELAYS else GPIO.HIGH
RELAY_OFF = GPIO.HIGH if LOW_TRIGGERED_RELAYS else GPIO.LOW

# -------------------- Pin Mapping Overview --------------------
# Raspberry Pi in BCM mode (GPIO.setmode(GPIO.BCM))
# Relay board is LOW-triggered (set via LOW_TRIGGERED_RELAYS flag).
#
# Device           Relay IN   Pi BCM   Pi Physical Pin
# ----------------------------------------------------
# Outside fan   ->   IN2    ->  27   ->  Pin 13
# Inside fan    ->   IN3    ->  22   ->  Pin 15
# Humidifier    ->   IN4    ->  23   ->  Pin 16
# UV lamp       ->   IN5    ->  17   ->  Pin 11   <-- ADJUST if your IN5 maps to a different BCM pin
#
# Notes:
# - Relay VCC -> Pi 5V (pin 2 or 4)
# - Relay GND -> Pi GND (pin 6/9/14/20/25/30/34/39)
# - All grounds (Pi, relay board, sensors) must be common.
# ---------------------------------------------------------------

OUTSIDE_FAN_PIN = 27  # BCM27, physical pin 13, Relay IN2
INSIDE_FAN_PIN  = 22  # BCM22, physical pin 15, Relay IN3
HUMIDIFIER_PIN  = 23  # BCM23, physical pin 16, Relay IN4
UV_PIN          = 17  # BCM17, physical pin 11, Relay IN5  <-- SET THIS to your wiring

# Hysteresis thresholds (environment control)
RH_ON, RH_OFF   = 85.0, 95.0     # %RH
CO2_ON, CO2_OFF = 800, 700       # ppm

SLEEP_SECONDS = 10               # loop cadence (s)

# Anti-chatter (seconds): minimal dwell times per output
MIN_ON_S  = 60
MIN_OFF_S = 60

# CO2 filtering
CO2_EMA_ALPHA = 0.30
CO2_MIN_VALID = 300
CO2_MAX_VALID = 40000

# UV schedule (total 120 min/day as 8 × 15 min), evenly every 3 hours
UV_SLOT_MINUTES   = 15
UV_SLOTS_PER_DAY  = 8
UV_INTERVAL_HOURS = 24 // UV_SLOTS_PER_DAY   # -> 3
UV_START_HOUR     = 0                        # first slot at 00:00 local time

LOGFILE = "telemetry.csv"

# -------------------- Helpers --------------------
def log(msg: str) -> None:
    ts = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)

def act_snapshot(reason: str, is_humidifying: bool, is_venting: bool, is_uv_on: bool) -> None:
    inside_on = is_humidifying or is_venting
    log(f"[ACT] HUM={'ON' if is_humidifying else 'OFF'} | "
        f"OUT={'ON' if is_venting else 'OFF'} | "
        f"IN={'ON' if inside_on else 'OFF'} | "
        f"UV={'ON' if is_uv_on else 'OFF'} :: {reason}")

def uv_should_be_on(now: dt.datetime) -> bool:
    """
    Returns True if current local time is within any of the UV slots today.
    Slots: start at UV_START_HOUR and then every UV_INTERVAL_HOURS for UV_SLOT_MINUTES.
    Example: start 00:00, then 03:00, 06:00, ..., 21:00; each lasts 15 minutes.
    """
    slot_duration = dt.timedelta(minutes=UV_SLOT_MINUTES)
    for k in range(UV_SLOTS_PER_DAY):
        start_hour = (UV_START_HOUR + k * UV_INTERVAL_HOURS) % 24
        slot_start = now.replace(hour=start_hour, minute=0, second=0, microsecond=0)
        # if slot_start is in the future (next day) due to hour wrap, pull it back 24h
        if slot_start > now and (slot_start - now) > dt.timedelta(hours=12):
            slot_start -= dt.timedelta(days=1)
        slot_end = slot_start + slot_duration
        if slot_start <= now < slot_end:
            return True
    return False

# -------------------- Hardware Setup --------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in (OUTSIDE_FAN_PIN, INSIDE_FAN_PIN, HUMIDIFIER_PIN, UV_PIN):
    GPIO.setup(pin, GPIO.OUT, initial=RELAY_OFF)

# Logical states
is_venting = False
is_humidifying = False
is_uv_on = False

# Anti-chatter tracking
last_change = {HUMIDIFIER_PIN: 0.0, OUTSIDE_FAN_PIN: 0.0, INSIDE_FAN_PIN: 0.0, UV_PIN: 0.0}
phys_state  = {HUMIDIFIER_PIN: False, OUTSIDE_FAN_PIN: False, INSIDE_FAN_PIN: False, UV_PIN: False}

def _set_pin(pin: int, want_on: bool) -> None:
    now = time.monotonic()
    cur = phys_state[pin]
    if cur == want_on:
        return
    min_gap = MIN_ON_S if want_on else MIN_OFF_S
    if now - last_change[pin] < min_gap:
        return
    GPIO.output(pin, RELAY_ON if want_on else RELAY_OFF)
    phys_state[pin] = want_on
    last_change[pin] = now
    log(f"[ACT_HW] {'ON ' if want_on else 'OFF'} -> pin {pin}")

def apply_outputs() -> None:
    inside_on = is_humidifying or is_venting
    _set_pin(HUMIDIFIER_PIN, is_humidifying)
    _set_pin(OUTSIDE_FAN_PIN, is_venting)
    _set_pin(INSIDE_FAN_PIN, inside_on)
    _set_pin(UV_PIN, is_uv_on)

# -------------------- Sensor Setup --------------------
i2c = board.I2C()
scd4x = adafruit_scd4x.SCD4X(i2c)
scd4x.start_periodic_measurement()

# -------------------- CSV Setup --------------------
new_file = not os.path.exists(LOGFILE)
csv_file = open(LOGFILE, "a", newline="")
csv_writer = csv.writer(csv_file)
if new_file:
    csv_writer.writerow([
        "timestamp","co2_raw","co2_smooth","rh_pct","temp_c",
        "humidifying","venting","inside_fan","uv_on"
    ])

# -------------------- Main --------------------
try:
    log("[INIT] Waiting for first SCD4x sample...")
    co2_ema = None

    # Wait for first valid sample
    while True:
        try:
            if scd4x.data_ready:
                co2 = scd4x.CO2
                rh  = scd4x.relative_humidity
                tc  = scd4x.temperature
                if None not in (co2, rh, tc):
                    co2_ema = float(co2)
                    break
        except Exception as e:
            log(f"[INIT] sensor read error: {e}")
        time.sleep(1)

    log("[INIT] Sensor OK, controller armed.")
    apply_outputs()
    act_snapshot("startup -> all OFF", is_humidifying, is_venting, is_uv_on)

    while True:
        try:
            # --- UV schedule (time-based, independent of sensors) ---
            now = dt.datetime.now()
            prev_uv = is_uv_on
            is_uv_on = uv_should_be_on(now)
            if prev_uv != is_uv_on:
                act_snapshot(f"[UV] schedule → UV {'ON' if is_uv_on else 'OFF'}", is_humidifying, is_venting, is_uv_on)
                apply_outputs()

            # --- Environmental control loop ---
            if scd4x.data_ready:
                co2_raw = scd4x.CO2
                rh      = scd4x.relative_humidity
                tc      = scd4x.temperature

                if None in (co2_raw, rh, tc):
                    raise ValueError("Sensor returned None value(s)")

                co2_ema = (CO2_EMA_ALPHA * co2_raw) + ((1 - CO2_EMA_ALPHA) * co2_ema)

                tf = tc * 9/5 + 32
                log(f"-- CO2_raw:{int(co2_raw)}ppm  CO2_smooth:{int(co2_ema)}ppm  "
                    f"RH:{rh:.1f}%  T:{tc:.1f}C/{tf:.1f}F  UV:{'ON' if is_uv_on else 'OFF'}")

                co2_valid_for_act = (CO2_MIN_VALID <= co2_ema <= CO2_MAX_VALID)

                prev_humid = is_humidifying
                if rh < RH_ON and not is_humidifying:
                    is_humidifying = True
                    act_snapshot(f"[RH] {rh:.1f}% < {RH_ON:.1f}% → HUM ON",
                                 is_humidifying, is_venting, is_uv_on)
                elif rh >= RH_OFF and is_humidifying:
                    is_humidifying = False
                    act_snapshot(f"[RH] {rh:.1f}% ≥ {RH_OFF:.1f}% → HUM OFF",
                                 is_humidifying, is_venting, is_uv_on)

                prev_vent = is_venting
                if co2_valid_for_act:
                    if co2_ema > CO2_ON and not is_venting:
                        is_venting = True
                        act_snapshot(f"[CO2] {int(co2_ema)} > {CO2_ON} → VENT ON",
                                     is_humidifying, is_venting, is_uv_on)
                    elif co2_ema <= CO2_OFF and is_venting:
                        is_venting = False
                        act_snapshot(f"[CO2] {int(co2_ema)} ≤ {CO2_OFF} → VENT OFF",
                                     is_humidifying, is_venting, is_uv_on)
                else:
                    log(f"[CO2] reading {int(co2_ema)}ppm ignored for actuation "
                        f"(valid {CO2_MIN_VALID}-{CO2_MAX_VALID})")

                if (prev_humid != is_humidifying) or (prev_vent != is_venting):
                    apply_outputs()

                # --- CSV logging ---
                inside_on = is_humidifying or is_venting
                csv_writer.writerow([
                    dt.datetime.now().isoformat(timespec="seconds"),
                    int(co2_raw), int(co2_ema), f"{rh:.1f}", f"{tc:.1f}",
                    int(is_humidifying), int(is_venting), int(inside_on), int(is_uv_on)
                ])
                csv_file.flush()

        except Exception as e:
            log(f"[LOOP] error: {e}")

        time.sleep(SLEEP_SECONDS)

except KeyboardInterrupt:
    log("Stopping controller and cleaning up GPIO.")
    GPIO.cleanup()
    csv_file.close()
except Exception as e:
    log(f"[FATAL] {e}")
    GPIO.cleanup()
    csv_file.close()
