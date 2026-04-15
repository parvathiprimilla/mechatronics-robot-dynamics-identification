from machine import Pin
import time
led = Pin(25, Pin.OUT)  # RP2040 built-in LED
print("Starting LED blink test...")
print("LED should blink 5 times")
for i in range(5):
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
    print(f"Blink {i+1}")
print("Test complete!")

