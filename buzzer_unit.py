import RPi.GPIO as GPIO
import time

PIN_BUZZER = 5  # BCM

def init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_BUZZER, GPIO.OUT)
    GPIO.output(PIN_BUZZER, GPIO.LOW)

def on():
    GPIO.output(PIN_BUZZER, GPIO.HIGH)

def off():
    GPIO.output(PIN_BUZZER, GPIO.LOW)

def beep(duration=0.3):
    on(); time.sleep(duration); off()

def siren(duration=1.2):
    end = time.time() + duration
    while time.time() < end:
        on(); time.sleep(0.2)
        off(); time.sleep(0.2)

def cleanup():
    off()
