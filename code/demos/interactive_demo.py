from pololu_3pi_2040_robot import robot
import time

# Setup
display = robot.Display()
button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()
motors = robot.Motors()
buzzer = robot.Buzzer()
rgb_leds = robot.RGBLEDs()

display.text("Simple Demo", 20, 10)
display.text("A=Forward", 20, 25)
display.text("B=Lights", 20, 40)
display.text("C=Music", 20, 55)
display.show()

print("Simple Robot Demo!")
print("Press A, B, or C")

while True:
    # Button A - Drive forward and back
    if button_a.check():
        display.fill(0)
        display.text("Going!", 40, 25)
        display.show()
        
        buzzer.play("c8")
        
        # Forward
        motors.set_speeds(3000, 3000)
        time.sleep(1.5)
        
        # Stop
        motors.set_speeds(0, 0)
        time.sleep(0.3)
        
        # Backward
        motors.set_speeds(-3000, -3000)
        time.sleep(1.5)
        
        # Stop
        motors.set_speeds(0, 0)
        
        buzzer.play("g8")
        
        display.fill(0)
        display.text("Done!", 40, 25)
        display.show()
        time.sleep(1)
        
        display.fill(0)
        display.text("Press again!", 15, 25)
        display.show()
    
    # Button B - Light show
    elif button_b.check():
        display.fill(0)
        display.text("Lights!", 35, 25)
        display.show()
        
        colors = [
            (255, 0, 0),    
            (0, 255, 0),    
            (0, 0, 255),  
            (255, 255, 0),
            (255, 0, 255), 
            (0, 255, 255), 
        ]
        
        for color in colors:
            # Set all 6 LEDs to the same color
            for led in range(6):
                rgb_leds.set(led, color)
            rgb_leds.show()
            buzzer.play("c16")
            time.sleep(0.3)
        
        # Turn off all LEDs
        for led in range(6):
            rgb_leds.set(led, (0, 0, 0))
        rgb_leds.show()
        
        display.fill(0)
        display.text("Done!", 40, 25)
        display.show()
        time.sleep(1)
        
        display.fill(0)
        display.text("Press again!", 15, 25)
        display.show()
    
    # Button C - Play music WITH lights!
    elif button_c.check():
        display.fill(0)
        display.text("Music+Lights!", 15, 25)
        display.show()
        
        # Colors for the show
        colors = [
            (255, 0, 0),    
            (255, 128, 0),  
            (255, 255, 0),  
            (0, 255, 0),    
            (0, 255, 255),  
            (0, 0, 255),    
            (255, 0, 255),  
            (255, 255, 255),
        ]
        
        notes = ["c8", "d8", "e8", "f8", "g8", "a8", "b8", ">c8"]
        
        # Play each note with ALL 6 lights changing color
        for i in range(len(notes)):
            color = colors[i]
            # Light up ALL 6 LEDs
            for led in range(6):
                rgb_leds.set(led, color)
            rgb_leds.show()
            
            buzzer.play(notes[i])
            time.sleep(0.4)
        
        # Turn off ALL 6 lights
        for led in range(6):
            rgb_leds.set(led, (0, 0, 0))
        rgb_leds.show()
        
        display.fill(0)
        display.text("Done!", 40, 25)
        display.show()
        time.sleep(1)
        
        display.fill(0)
        display.text("Press again!", 15, 25)
        display.show()
    
    time.sleep(0.1)

