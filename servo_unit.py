import RPi.GPIO as GPIO
import time

PIN_SERVO = 12  # BCM

_pwm = None
_current_angle = 90  # start centered

def angle_to_duty(angle):
    # 50Hz servo, ~2.5–12.5% duty for 0–180 deg
    angle = max(0, min(180, float(angle)))
    return 2.5 + (angle / 180.0) * 10.0

def init():
    global _pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_SERVO, GPIO.OUT)
    _pwm = GPIO.PWM(PIN_SERVO, 50)  # 50 Hz
    _pwm.start(0)
    set_angle(_current_angle)

def set_angle(angle):
    global _current_angle
    duty = angle_to_duty(angle)
    _pwm.ChangeDutyCycle(duty)
    time.sleep(0.4)
    _pwm.ChangeDutyCycle(0)
    _current_angle = int(round(max(0, min(180, float(angle)))))

def sweep_once():
    for a in [0, 90, 180, 90]:
        set_angle(a)
        time.sleep(0.5)

def get_angle():
    return _current_angle

def cleanup():
    global _pwm
    if _pwm:
        _pwm.stop()
        _pwm = None
