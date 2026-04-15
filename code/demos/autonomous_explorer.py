from pololu_3pi_2040_robot import robot
import time
import random

# Initialize all components
display = robot.Display()
button_a = robot.ButtonA()
button_c = robot.ButtonC()
motors = robot.Motors()
buzzer = robot.Buzzer()
rgb_leds = robot.RGBLEDs()
bump_sensors = robot.BumpSensors()
encoders = robot.Encoders()

# Exploration statistics
stats = {
    'distance_traveled': 0,
    'collisions': 0,
    'turns_left': 0,
    'turns_right': 0,
    'exploration_time': 0
}

# Robot states
STATE_EXPLORING = 0
STATE_AVOIDING = 1
STATE_TURNING = 2

current_state = STATE_EXPLORING

def set_led_color_by_state(state):
    if state == STATE_EXPLORING:
        color = (0, 255, 0)  
    elif state == STATE_AVOIDING:
        color = (255, 0, 0)  
    else:
        color = (0, 0, 255)  
    
    for led in range(6):
        rgb_leds.set(led, color)
    rgb_leds.show()

def update_display():
    display.fill(0)
    display.text("Explorer", 30, 0)
    display.text(f"Dist:{stats['distance_traveled']}", 0, 15)
    display.text(f"Hits:{stats['collisions']}", 0, 30)
    display.text(f"L:{stats['turns_left']} R:{stats['turns_right']}", 0, 45)
    display.show()

def calculate_distance():
    left, right = encoders.get_counts()
    avg = (abs(left) + abs(right)) // 2
    return avg // 50

# Startup sequence
display.text("Autonomous", 20, 10)
display.text("Explorer", 30, 25)
display.text("", 0, 40)
display.text("Press A", 35, 55)
display.show()

print("=== Autonomous Explorer ===")
print("Press Button A to start")
print("Press Button C to stop")

# Wait for start
while not button_a.check():
    time.sleep(0.1)

# Startup animation
buzzer.play("c8 e8 g8 >c4")
for i in range(6):
    rgb_leds.set(i, (0, 255, 0))
    rgb_leds.show()
    time.sleep(0.1)

print("\nExploration started!")
start_time = time.ticks_ms()
encoders.get_counts()  # Reset encoder baseline

running = True
obstacle_cooldown = 0

while running:
    if button_c.check():
        running = False
        break

    stats['distance_traveled'] = calculate_distance()
    stats['exploration_time'] = time.ticks_diff(time.ticks_ms(), start_time) // 1000
    
    left_bump = bump_sensors.left.check()
    right_bump = bump_sensors.right.check()
    
    if (left_bump or right_bump) and obstacle_cooldown == 0:
        current_state = STATE_AVOIDING
        set_led_color_by_state(current_state)
        stats['collisions'] += 1
        
        buzzer.play("a16")
        update_display()
        
        print(f"Collision #{stats['collisions']} detected!")
        
        motors.set_speeds(0, 0)
        time.sleep(0.2)
        
        motors.set_speeds(-3000, -3000)
        time.sleep(0.5)
        motors.set_speeds(0, 0)
        time.sleep(0.2)
       
        current_state = STATE_TURNING
        set_led_color_by_state(current_state)
        
        if left_bump and not right_bump:
            motors.set_speeds(3000, -3000)
            stats['turns_right'] += 1
            print("  Turning right...")
        elif right_bump and not left_bump:
            motors.set_speeds(-3000, 3000)
            stats['turns_left'] += 1
            print("  Turning left...")
        else:
            if random.random() > 0.5:
                motors.set_speeds(3000, -3000)
                stats['turns_right'] += 1
                print("  Turning right (random)...")
            else:
                motors.set_speeds(-3000, 3000)
                stats['turns_left'] += 1
                print("  Turning left (random)...")
        
        turn_time = random.uniform(0.3, 0.8)
        time.sleep(turn_time)
        
        motors.set_speeds(0, 0)
        time.sleep(0.2)
        
        buzzer.play("c16")
        obstacle_cooldown = 10
        
    else: 
        current_state = STATE_EXPLORING
        set_led_color_by_state(current_state)
        base_speed = 2500
        left_speed = base_speed + random.randint(-200, 200)
        right_speed = base_speed + random.randint(-200, 200)
        motors.set_speeds(left_speed, right_speed)
        
        if obstacle_cooldown > 0:
            obstacle_cooldown -= 1
    
    if stats['exploration_time'] % 2 == 0:
        update_display()
    
    time.sleep(0.1)
motors.set_speeds(0, 0)

for led in range(6):
    rgb_leds.set(led, (0, 0, 0))
rgb_leds.show()

buzzer.play("o5 c8 e8 g8 c4")

display.fill(0)
display.text("COMPLETE!", 25, 5)
display.text(f"Time: {stats['exploration_time']}s", 10, 20)
display.text(f"Distance: {stats['distance_traveled']}", 10, 32)
display.text(f"Obstacles: {stats['collisions']}", 10, 44)
display.text(f"L:{stats['turns_left']} R:{stats['turns_right']}", 10, 56)
display.show()

print("\n=== Exploration Complete ===")
print(f"Time: {stats['exploration_time']} seconds")
print(f"Distance: ~{stats['distance_traveled']} units")
print(f"Obstacles encountered: {stats['collisions']}")
print(f"Left turns: {stats['turns_left']}")
print(f"Right turns: {stats['turns_right']}")
print("\nPress reset to run again")
