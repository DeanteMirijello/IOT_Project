#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart Home Automations (modular) + Adafruit IO
- Uses leds.py, servo_unit.py, dht_sensor.py, buzzer_unit.py, motion_detector.py
- Reads config.json (username/key, pins, intervals, rate limits, feeds)
- Publishes states to AIO; accepts MQTT commands for leds/buzzer/servo/target-temp
- Keeps menu-driven UI for local testing
"""

import os
import json
import time
import signal
import threading
from pathlib import Path
from datetime import datetime

# ---- Local modules (same pins as you already wired) ----
import leds
import servo_unit
import dht_sensor
import buzzer_unit
import motion_detector

# ---- Load config ----
CONFIG_PATH = Path(__file__).with_name("config.json")
with open(CONFIG_PATH, "r") as f:
    cfg = json.load(f)

# Credentials: allow env override, else config.json
ADAFRUIT_IO_USERNAME = os.getenv("AIO_USERNAME", cfg["adafruit"]["username"])
ADAFRUIT_IO_KEY      = os.getenv("AIO_KEY", cfg["adafruit"]["key"])

# Intervals / limits / feeds
sensor_publish_interval = int(cfg["intervals"]["sensor_publish_seconds"])
heartbeat_interval      = int(cfg["intervals"]["heartbeat_seconds"])
GLOBAL_MIN_GAP          = float(cfg["rate_limits"]["global_min_gap"])
PER_FEED_MIN            = {k: float(v) for k, v in cfg["rate_limits"]["per_feed_min"].items()}
MOTION_EDGE_MIN_GAP     = float(cfg["motion"]["edge_min_gap"])
MOTION_QUIET_REQUIRED   = float(cfg["motion"]["quiet_required"])
FEEDS                   = list(cfg["feeds"])
target_temp_c           = float(cfg.get("target_temp_c", 26.0))

# ---- Adafruit IO ----
from Adafruit_IO import Client as AIOClient, MQTTClient as AIOMQTT, Feed, RequestError
aio_rest = AIOClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
aio_mqtt = AIOMQTT(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

def ensure_required_feeds():
    print(f"[AIO] Ensuring {len(FEEDS)} required feeds exist...")
    for name in FEEDS:
        try:
            aio_rest.feeds(name)
        except RequestError:
            try:
                aio_rest.create_feed(Feed(name=name))
                print(f"[AIO] Created feed '{name}'")
            except Exception as e:
                print(f"[AIO] Could not create '{name}': {e} (continuing)")

# ---- Rate limiting state ----
last_sent_at     = {k: 0.0 for k in PER_FEED_MIN}
last_global_send = 0.0
backoff_until    = 0.0
last_states = {
    "temperature": None,
    "humidity": None,
    "motion": 0,
    "led-yellow": 0,
    "led-green":  0,
    "led-red":    0,
    "buzzer":     0,
    "servo":      0,  # angle
    "target-temp": target_temp_c,
}

def now():
    return time.monotonic()

def publish_allowed(feed):
    t = now()
    if t < backoff_until:
        return False
    if (t - last_global_send) < GLOBAL_MIN_GAP:
        return False
    if (t - last_sent_at.get(feed, 0.0)) < PER_FEED_MIN.get(feed, 5.0):
        return False
    return True

def handle_rate_limit(e):
    global backoff_until
    msg = str(e).lower()
    if "limit" in msg or "too many" in msg or "rate" in msg:
        backoff_until = now() + 8.0
        print("[AIO] Backing off publishes ~8s (rate limit).")

def publish(feed, value):
    global last_global_send
    t = now()
    if not publish_allowed(feed):
        return
    try:
        aio_rest.send(feed, value)
        last_states[feed] = value
        last_sent_at[feed] = t
        last_global_send = t
    except Exception as e:
        print(f"[AIO] publish '{feed}' failed: {e}")
        handle_rate_limit(e)

def publish_if_changed(feed, value):
    if last_states.get(feed) != value:
        publish(feed, value)

# ---- MQTT callbacks ----
def mqtt_connected(client):
    print("[AIO-MQTT] Connected. Subscribing...")
    for name in FEEDS:
        try:
            client.subscribe(name)
        except Exception as e:
            print(f"[AIO-MQTT] subscribe {name} failed: {e}")

def mqtt_disconnected(client):
    print("[AIO-MQTT] Disconnected. Reconnecting when possible...")

def mqtt_message(client, feed_id, payload):
    global target_temp_c
    val = str(payload).strip().lower()
    try:
        if feed_id in ("led-yellow","led-green","led-red","buzzer"):
            on = val in ("1","on","true","yes")
            if feed_id == "led-yellow":
                leds.on("yellow") if on else leds.off("yellow")
                publish_if_changed("led-yellow", 1 if on else 0)
            elif feed_id == "led-green":
                leds.on("green") if on else leds.off("green")
                publish_if_changed("led-green", 1 if on else 0)
            elif feed_id == "led-red":
                leds.on("red") if on else leds.off("red")
                publish_if_changed("led-red", 1 if on else 0)
            elif feed_id == "buzzer":
                if on:
                    buzzer_unit.on()
                    publish_if_changed("buzzer", 1)
                else:
                    buzzer_unit.off()
                    publish_if_changed("buzzer", 0)

        elif feed_id == "servo":
            if val in ("open","unlock"):
                servo_unit.set_angle(90);  publish_if_changed("servo", 90)
            elif val in ("close","lock"):
                servo_unit.set_angle(0);   publish_if_changed("servo", 0)
            else:
                try:
                    angle = float(val)
                    servo_unit.set_angle(angle)
                    publish_if_changed("servo", int(round(angle)))
                except ValueError:
                    print("[AIO-MQTT] servo expects angle/open/close")

        elif feed_id == "target-temp":
            try:
                target_temp_c = float(val)
                publish_if_changed("target-temp", target_temp_c)
                print(f"[AIO-MQTT] target-temp set to {target_temp_c:.1f}°C")
            except ValueError:
                print("[AIO-MQTT] target-temp expects number")

    except Exception as e:
        print(f"[AIO-MQTT] handler error: {e}")

aio_mqtt.on_connect    = mqtt_connected
aio_mqtt.on_disconnect = mqtt_disconnected
aio_mqtt.on_message    = mqtt_message

# ---- Threads: sensors, heartbeat, motion-reset ----
running = True
_motion_reset_timer = None
_motion_lock = threading.Lock()

def motion_mark_clear_after(delay):
    """Clear motion feed after 'delay' seconds."""
    def _clear():
        with _motion_lock:
            publish_if_changed("motion", 0)
    t = threading.Timer(delay, _clear)
    t.daemon = True
    t.start()
    return t

def motion_callback():
    """Called by motion_detector when a rising motion edge is detected."""
    print("⚠️  Motion detected!")
    leds.on("green")
    buzzer_unit.beep(0.2)
    time.sleep(0.1)
    leds.off("green")

    with _motion_lock:
        publish_if_changed("motion", 1)
        global _motion_reset_timer
        if _motion_reset_timer:
            _motion_reset_timer.cancel()
        _motion_reset_timer = motion_mark_clear_after(MOTION_QUIET_REQUIRED)

def sensor_loop():
    print("[LOOP] sensor_loop started")
    while running:
        try:
            t, h = dht_sensor.read()
            if t is not None and h is not None:
                publish_if_changed("temperature", round(t, 2))
                publish_if_changed("humidity", round(h, 2))

                # simple thermostat-led demo on yellow
                if t > target_temp_c:
                    leds.on("yellow");  publish_if_changed("led-yellow", 1)
                else:
                    leds.off("yellow"); publish_if_changed("led-yellow", 0)

                print(f"[SENSORS] {datetime.now().strftime('%H:%M:%S')} "
                      f"T={t:.1f}°C H={h:.1f}% (target {target_temp_c:.1f}°C)")
            else:
                print("[SENSORS] DHT22 read failed; will retry")
        except Exception as e:
            print(f"[SENSORS] Error: {e}")
        time.sleep(sensor_publish_interval)

def heartbeat_loop():
    print("[LOOP] heartbeat_loop started")
    while running:
        try:
            # Publish current states (avoids stale dashboards)
            st = leds.status()
            publish_if_changed("led-yellow", 1 if st["yellow"] == "ON" else 0)
            publish_if_changed("led-green",  1 if st["green"]  == "ON" else 0)
            publish_if_changed("led-red",    1 if st["red"]    == "ON" else 0)
            publish_if_changed("buzzer",     0)  # buzzer is momentary; default 0
            publish_if_changed("servo",      servo_unit.get_angle())
            publish_if_changed("target-temp", target_temp_c)
        except Exception as e:
            print(f"[HEARTBEAT] error: {e}")
        time.sleep(heartbeat_interval)

# ---- Menu UI ----
def show_menu():
    st = leds.status()
    print("\n=== Smart I/O Panel (AIO-enabled) ===")
    print(f"  1. Toggle Yellow LED ({st['yellow']}) — GPIO 21  -> feed: led-yellow")
    print(f"  2. Toggle Green  LED ({st['green']})  — GPIO 25  -> feed: led-green")
    print(f"  3. Toggle Red    LED ({st['red']})    — GPIO 16  -> feed: led-red")
    print(f"  4. Buzzer beep/siren                  — GPIO 5    -> feed: buzzer")
    print(f"  5. Servo angle/sweep ({servo_unit.get_angle()}°)— GPIO 12  -> feed: servo")
    print(f"  6. Motion monitor start/stop          — GPIO 26  -> feed: motion")
    print(f"  7. Read DHT22 (Temp/Hum)              — GPIO 6   -> feeds: temperature, humidity")
    print("Commands:")
    print("  [1-3] Toggle LED  |  a All ON  |  o All OFF  |  p Party once")
    print("  b Beep  |  z Siren  |  v Sweep  |  x Set servo angle 0–180")
    print("  t Read DHT |  m Toggle motion thread |  g Show target-temp |  u Set target-temp")
    print("  s Show status  |  q Quit")

def start_everything():
    leds.init()
    servo_unit.init()
    buzzer_unit.init()
    dht_sensor.init()
    motion_detector.init()
    ensure_required_feeds()

    # MQTT connect (non-blocking loop)
    try:
        aio_mqtt.connect()
        aio_mqtt.loop_background()
    except Exception as e:
        print("[AIO-MQTT] connect failed:", e)

    # Initial publishes
    publish_if_changed("target-temp", target_temp_c)
    publish_if_changed("servo", servo_unit.get_angle())

def stop_everything():
    global running
    running = False
    try: motion_detector.cleanup()
    except Exception: pass
    try: buzzer_unit.cleanup()
    except Exception: pass
    try: servo_unit.cleanup()
    except Exception: pass
    try: leds.cleanup()
    except Exception: pass
    try: dht_sensor.cleanup()
    except Exception: pass
    try: aio_mqtt.disconnect()
    except Exception: pass
    time.sleep(0.2)

def main():
    start_everything()

    # background workers
    t_sensors   = threading.Thread(target=sensor_loop,    daemon=True); t_sensors.start()
    t_heartbeat = threading.Thread(target=heartbeat_loop, daemon=True); t_heartbeat.start()

    motion_running = False

    try:
        while True:
            show_menu()
            choice = input("\nEnter command: ").strip().lower()
            if choice == 'q':
                break
            elif choice in ('1','2','3'):
                key = {'1':'yellow','2':'green','3':'red'}[choice]
                state = leds.toggle(key)
                feed  = f"led-{key}"
                publish_if_changed(feed, 1 if state else 0)
                print(f"✓ {key.capitalize()} -> {'ON' if state else 'OFF'}  (published {feed})")
            elif choice == 'a':
                leds.all_on()
                publish_if_changed("led-yellow", 1)
                publish_if_changed("led-green",  1)
                publish_if_changed("led-red",    1)
                print("✓ All LEDs ON (published)")
            elif choice == 'o':
                leds.all_off()
                publish_if_changed("led-yellow", 0)
                publish_if_changed("led-green",  0)
                publish_if_changed("led-red",    0)
                print("✓ All LEDs OFF (published)")
            elif choice == 'p':
                leds.party_once()
                print("✓ Party wave")
            elif choice == 'b':
                buzzer_unit.beep()
                publish_if_changed("buzzer", 1); time.sleep(0.05); publish_if_changed("buzzer", 0)
                print("✓ Beep (published)")
            elif choice == 'z':
                buzzer_unit.siren()
                publish_if_changed("buzzer", 1); time.sleep(0.1); publish_if_changed("buzzer", 0)
                print("✓ Siren (published)")
            elif choice == 'v':
                servo_unit.sweep_once()
                publish_if_changed("servo", servo_unit.get_angle())
                print("✓ Servo sweep (published angle)")
            elif choice == 'x':
                try:
                    angle = float(input("Angle (0–180): ").strip())
                    servo_unit.set_angle(angle)
                    publish_if_changed("servo", int(round(angle)))
                    print(f"✓ Servo -> {int(angle)}° (published)")
                except ValueError:
                    print("❌ Invalid angle")
            elif choice == 't':
                t, h = dht_sensor.read()
                if t is None:
                    print("DHT read failed; try again.")
                else:
                    print(f"Temp: {t:.1f}°C | Humidity: {h:.1f}%")
                    publish_if_changed("temperature", round(t,2))
                    publish_if_changed("humidity",    round(h,2))
            elif choice == 'm':
                if motion_running:
                    motion_detector.stop()
                    motion_running = False
                    print("✓ Motion monitor stopped")
                else:
                    motion_detector.start(callback_on_motion=motion_callback)
                    motion_running = True
                    print("✓ Motion monitor started")
            elif choice == 's':
                st = leds.status()
                print("\n--- Status ---")
                print(f"Yellow: {st['yellow']} | Green: {st['green']} | Red: {st['red']}")
                print(f"Servo: {servo_unit.get_angle()}° | target-temp: {target_temp_c:.1f}°C")
            elif choice == 'g':
                print(f"Target-temp: {target_temp_c:.1f}°C")
            elif choice == 'u':
                try:
                    newt = float(input("New target-temp (°C): ").strip())
                    globals()['target_temp_c'] = newt
                    publish_if_changed("target-temp", newt)
                    print(f"✓ target-temp set to {newt:.1f}°C (published)")
                except ValueError:
                    print("❌ Invalid number")
            else:
                print("❌ Invalid command")

            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\n[CTRL+C] Exiting...")
    finally:
        stop_everything()
        print("Goodbye!")

if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: stop_everything())
    main()
