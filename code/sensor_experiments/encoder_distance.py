from machine import Pin
import time

print("=== Encoder Distance Calculator ===")
print("Rotate wheel exactly 1 full rotation")
print("Press Ctrl+C when done")
print()

# Physical parameters from measurements
WHEEL_DIAMETER_MM = 32
COUNTS_PER_REV = 360  # 12 CPR × 30:1 gear ratio
WHEEL_CIRCUMFERENCE_MM = 3.14159 * WHEEL_DIAMETER_MM
MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV

print(f"Wheel diameter: {WHEEL_DIAMETER_MM}mm")
print(f"Wheel circumference: {WHEEL_CIRCUMFERENCE_MM:.2f}mm")
print(f"Distance per count: {MM_PER_COUNT:.3f}mm")
print()

# Encoder pins 
LEFT_ENC_A = 8
RIGHT_ENC_A = 12

left_a = Pin(LEFT_ENC_A, Pin.IN, Pin.PULL_UP)
right_a = Pin(RIGHT_ENC_A, Pin.IN, Pin.PULL_UP)
left_count = 0
right_count = 0
last_left = left_a.value()
last_right = right_a.value()

print("Counting started...")
print()

try:
    while True:
        curr_left = left_a.value()
        curr_right = right_a.value()
        
        # Detect rising edges
        if curr_left != last_left and curr_left == 1:
            left_count += 1
            left_distance = left_count * MM_PER_COUNT
            print(f"LEFT:  {left_count:4d} counts = {left_distance:6.1f}mm")
        
        if curr_right != last_right and curr_right == 1:
            right_count += 1
            right_distance = right_count * MM_PER_COUNT
            print(f"RIGHT: {right_count:4d} counts = {right_distance:6.1f}mm")
        
        last_left = curr_left
        last_right = curr_right
        time.sleep(0.001)
        
except KeyboardInterrupt:
    print()
    print("="*52)
    print("FINAL RESULTS:")
    
    left_distance = left_count * MM_PER_COUNT
    right_distance = right_count * MM_PER_COUNT
    left_rotations = left_count / COUNTS_PER_REV
    right_rotations = right_count / COUNTS_PER_REV
    
    print(f"LEFT:  {left_count} counts = {left_rotations:.2f} rotations = {left_distance:.1f}mm")
    print(f"RIGHT: {right_count} counts = {right_rotations:.2f} rotations = {right_distance:.1f}mm")
    print("="*52)

