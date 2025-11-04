import time
import adafruit_dht
import board

PIN_DHT_BCM = 6   # BCM, board.D6 on Pi

_dht = None

def init():
    global _dht
    # board.D6 corresponds to BCM 6 with Blinka
    _dht = adafruit_dht.DHT22(board.D6, use_pulseio=False)

def read():
    """Return (temperature_c, humidity) or (None, None) if read failed."""
    try:
        t = _dht.temperature
        h = _dht.humidity
        if t is None or h is None:
            return (None, None)
        return (float(t), float(h))
    except Exception:
        # DHT is finicky; a failure is normal. Caller can retry later.
        return (None, None)

def cleanup():
    try:
        if _dht:
            _dht.exit()
    except Exception:
        pass
