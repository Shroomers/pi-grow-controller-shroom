#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grow tent controller (Lion's Mane) — fixed minimal version
- Adds LOW_TRIGGERED_RELAYS flag (default: True)
- Uses GPIO.setup(..., initial=RELAY_OFF) to prevent power-on pulses
- Adds robust sensor read with try/except and None-guards
- Adds explicit decision logging so you can see *why* it switches
- Preserves your hysteresis logic and 10s loop cadence
"""

import time
import board
import RPi.GPIO as GPIO
import adafruit_scd4x

# -------------------- Config --------------------
# Set to True if your relay board is LOW-triggered (most common)
LOW_TRIGGERED_RELAYS = True

# Derive relay drive levels from polarity
RELAY_ON  = GPIO.LOW  if LOW_TRIGGERED_RELAYS else GPIO.HIGH
RELAY_OFF = GPIO.HIGH if LOW_TRIGGERED_RELAYS else GPIO.LOW

# GPIO pins (BCM numbering) — keep your existing mapping
OUTSIDE_FAN_PIN = 27  # Exhaust / outside fan
INSIDE_FAN_PIN  = 22  # Circulation / inside fan
HUMIDIFIER_PIN  = 23  # Humidifier

# Hysteresis thresholds
RH_ON, RH_OFF     = 85.0, 95.0    # %RH: below RH_ON -> humidify; at/above RH_OFF -> stop humidifying
CO2_ON, CO2_OFF   = 800, 700      # ppm: above CO2_ON -> vent; at/below CO2_OFF -> stop venting

SLEEP_SECONDS = 10                # main loop cadence

# -------------------- Hardware Setup --------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Ensure outputs start safe (OFF) to avoid unintended pulses at boot
GPIO.setup(OUTSIDE_FAN_PIN, GPIO.OUT, initial=RELAY_OFF)
GPIO.setup(INSIDE_FAN_PIN,  GPIO.OUT, initial=RELAY_OFF)
GPIO.setup(HUMIDIFIER_PIN,  GPIO.OUT, initial=RELAY_OFF)

# Track logical states (True = intended ON, regardless of relay polarity)
is_venting = False
is_humidifying = False

# Sensor init
i2c = board.I2C()
scd4x = adafruit_scd4x.SCD4X(i2c)
scd4x.start_periodic_measurement()

def apply_outputs():
    """Apply the current logical states to the GPIO pins and log changes."""
    GPIO.output(HUMIDIFIER_PIN, RELAY_ON if is_humidifying else RELAY_OFF)
    GPIO.output(OUTSIDE_FAN_PIN, RELAY_ON if is_venting else RELAY_OFF)
    # Inside fan follows if any device is active
    inside_on = is_humidifying or is_venting
    GPIO.output(INSIDE_FAN_PIN, RELAY_ON if inside_on else RELAY_OFF)

def print_actuation(reason: str):
    print(f"[ACT] HUM={'ON' if is_humidifying else 'OFF'} | OUT={'ON' if is_venting else 'OFF'} | IN={'ON' if (is_humidifying or is_venting) else 'OFF'}  :: {reason}")

# -------------------- Main --------------------
try:
    # Wait for the first valid sample to arm the controller
    print("[INIT] Waiting for first SCD4x sample...")
    while True:
        try:
            if scd4x.data_ready:
                co2 = scd4x.CO2
                rh  = scd4x.relative_humidity
                tc  = scd4x.temperature
                if None not in (co2, rh, tc):
                    break
        except Exception as e:
            print(f"[INIT] sensor read error: {e}")
        time.sleep(1)
    print("[INIT] Sensor OK, controller armed.")

    apply_outputs()  # ensure outputs are off at start
    print_actuation("startup -> all OFF")


    while True:
        try:
            if scd4x.data_ready:
                co2 = scd4x.CO2
                rh  = scd4x.relative_humidity
                tc  = scd4x.temperature

                # Guard against spurious None readings
                if None in (co2, rh, tc):
                    raise ValueError("Sensor returned None value(s)")

                tf = tc * 9/5 + 32
                print(f"-- CO2:{int(co2)}ppm  RH:{rh:.1f}%  T:{tc:.1f}C/{tf:.1f}F")

                # --- Humidity control with hysteresis ---
                prev_humid = is_humidifying
                if rh < RH_ON and not is_humidifying:
                    is_humidifying = True
                    print_actuation(f"[RH] {rh:.1f}% < {RH_ON:.1f}% → HUM ON")
                elif rh >= RH_OFF and is_humidifying:
                    is_humidifying = False
                    print_actuation(f"[RH] {rh:.1f}% ≥ {RH_OFF:.1f}% → HUM OFF")

                # --- CO2 control with hysteresis ---
                prev_vent = is_venting
                if co2 > CO2_ON and not is_venting:
                    is_venting = True
                    print_actuation(f"[CO2] {int(co2)} > {CO2_ON} → VENT ON")
                elif co2 <= CO2_OFF and is_venting:
                    is_venting = False
                    print_actuation(f"[CO2] {int(co2)} ≤ {CO2_OFF} → VENT OFF")

                # Apply outputs if any logical state changed
                if (prev_humid != is_humidifying) or (prev_vent != is_venting):
                    apply_outputs()

        except Exception as e:
            print(f"[LOOP] error: {e}")

        time.sleep(SLEEP_SECONDS)

except KeyboardInterrupt:
    print("Stopping controller and cleaning up GPIO.")
    GPIO.cleanup()
except Exception as e:
    print(f"[FATAL] {e}")
    GPIO.cleanup()
