from machine import Pin
import time
print("=== 3pi+ 2040 Encoder Test ===")
print("Rotate wheels by hand to test")
print("Press Ctrl+C to stop\n")
# Encoder pin assignments (experimentally determined)
LEFT_ENC_A = 8
LEFT_ENC_B = 9
RIGHT_ENC_A = 12
RIGHT_ENC_B = 13
# Initialize pins with pull-up resistors
left_a = Pin(LEFT_ENC_A, Pin.IN, Pin.PULL_UP)
left_b = Pin(LEFT_ENC_B, Pin.IN, Pin.PULL_UP)
right_a = Pin(RIGHT_ENC_A, Pin.IN, Pin.PULL_UP)
right_b = Pin(RIGHT_ENC_B, Pin.IN, Pin.PULL_UP)
# Counter variables
left_count = 0
right_count = 0
last_left_a = left_a.value()
last_right_a = right_a.value()
print("Encoders ready! Rotate wheels...\n")
try:
    while True:
        curr_left_a = left_a.value()
        curr_right_a = right_a.value()
        # Rising edge detection for left encoder
        if curr_left_a != last_left_a and curr_left_a == 1:
            left_count += 1
            print(f"LEFT: {left_count:4d}   RIGHT: {right_count:4d}")
        # Rising edge detection for right encoder
        if curr_right_a != last_right_a and curr_right_a == 1:
            right_count += 1
            print(f"LEFT: {left_count:4d}   RIGHT: {right_count:4d}")
        last_left_a = curr_left_a
        last_right_a = curr_right_a
        time.sleep(0.001)
except KeyboardInterrupt:
    print("\n" + "="*40)
    print("Test Complete!")
    print(f"Final LEFT count:  {left_count}")
    print(f"Final RIGHT count: {right_count}")
    print(f"LEFT rotations:  {left_count/360:.2f}")
    print(f"RIGHT rotations: {right_count/360:.2f}")
    print("="*40)
