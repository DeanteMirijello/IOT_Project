import RPi.GPIO as GPIO
import time
import threading

PIN_PIR = 26  # BCM

_motion_thread = None
_running = False
_last_motion = 0.0
_edge_min_gap = 20.0   # seconds between motion “1” events

def init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_PIR, GPIO.IN)  # PIRs usually have onboard pull; adjust if needed
    time.sleep(2.0)  # warm-up

def _loop(callback_on_motion=None):
    global _last_motion
    while _running:
        if GPIO.input(PIN_PIR):
            now = time.time()
            if (now - _last_motion) >= _edge_min_gap:
                _last_motion = now
                if callback_on_motion:
                    try: callback_on_motion()
                    except Exception: pass
            # wait for quiet
            while _running and GPIO.input(PIN_PIR):
                time.sleep(0.05)
        time.sleep(0.05)

def start(callback_on_motion=None):
    global _motion_thread, _running
    if _running:
        return
    _running = True
    _motion_thread = threading.Thread(target=_loop, args=(callback_on_motion,), daemon=True)
    _motion_thread.start()

def stop():
    global _running, _motion_thread
    _running = False
    if _motion_thread:
        _motion_thread.join(timeout=1.0)
        _motion_thread = None

def cleanup():
    stop()
