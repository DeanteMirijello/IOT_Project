import RPi.GPIO as GPIO
import time

# BCM pins (match current setup)
PIN_YELLOW = 21
PIN_GREEN  = 25
PIN_RED    = 16

LEDS = {
    "yellow": {"pin": PIN_YELLOW, "state": False},
    "green":  {"pin": PIN_GREEN,  "state": False},
    "red":    {"pin": PIN_RED,    "state": False},
}

def init():
    GPIO.setmode(GPIO.BCM)
    for name, led in LEDS.items():
        GPIO.setup(led["pin"], GPIO.OUT)
        GPIO.output(led["pin"], GPIO.LOW)
        led["state"] = False

def _set_pin(pin, on):
    GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)

def on(name):
    LEDS[name]["state"] = True
    _set_pin(LEDS[name]["pin"], True)

def off(name):
    LEDS[name]["state"] = False
    _set_pin(LEDS[name]["pin"], False)

def toggle(name):
    new_state = not LEDS[name]["state"]
    LEDS[name]["state"] = new_state
    _set_pin(LEDS[name]["pin"], new_state)
    return new_state

def all_on():
    for n in LEDS: on(n)

def all_off():
    for n in LEDS: off(n)

def status():
    return {n: ("ON" if d["state"] else "OFF") for n, d in LEDS.items()}

def party_once():
    # Simple “wave” effect across the 3 LEDs
    sequence = ["yellow", "green", "red", "green"]
    for n in sequence:
        on(n); time.sleep(0.15); off(n)

def cleanup():
    all_off()
