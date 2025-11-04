# test_buzzer.py
from gpiozero import Buzzer
from time import sleep

buzzer = Buzzer(5)

print("Testing buzzer... (it will beep 5 times)")
for i in range(5):
    buzzer.on()
    print(f"Beep {i+1}")
    sleep(0.3)
    buzzer.off()
    sleep(0.3)

print("Test complete.")
