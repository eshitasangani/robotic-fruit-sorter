import pigpio
import time

# Define the GPIO pins for each servo
SHOULDER_PIN = 5
ELBOW_PIN = 13
ROTATE_PIN = 16

# Function to move a servo through its full range
def test_servo(pwm, pin, name):
    print(f"Testing {name} servo...")
    
    # Move from the minimum to maximum position
    for pulse_width in range(750, 2251, 50):
        pwm.set_servo_pulsewidth(pin, pulse_width)
        time.sleep(0.1)

    # Move back to the neutral position
    pwm.set_servo_pulsewidth(pin, 1500)
    time.sleep(1)

# Initialize pigpio
pi = pigpio.pi()

try:
    # Set the mode for each servo
    pi.set_mode(SHOULDER_PIN, pigpio.OUTPUT)
    pi.set_mode(ELBOW_PIN, pigpio.OUTPUT)
    pi.set_mode(ROTATE_PIN, pigpio.OUTPUT)

    # Test each servo
    test_servo(pi, SHOULDER_PIN, "shoulder")
    test_servo(pi, ELBOW_PIN, "elbow")
    test_servo(pi, ROTATE_PIN, "rotate")

finally:
    # Stop the servos
    pi.set_servo_pulsewidth(SHOULDER_PIN, 0)
    pi.set_servo_pulsewidth(ELBOW_PIN, 0)
    pi.set_servo_pulsewidth(ROTATE_PIN, 0)

    # Disconnect from pigpio
    pi.stop()
