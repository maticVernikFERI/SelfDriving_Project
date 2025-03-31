import pigpio
import time

SERVO_PIN = 18  # Change to your GPIO pin
pi = pigpio.pi()

# Define pulse width limits (adjust for precision)
MIN_PULSE_WIDTH = 500   # Usually ~1278µs (0 degrees)
MAX_PULSE_WIDTH = 2500  # Usually ~2500µs (180 degrees)

def set_servo_angle(angle):
    """Convert angle (0-180) to pulse width (500-2500µs)"""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
    print(f"Angle: {angle}° -> Pulse Width: {pulse_width:.0f}µs")
    time.sleep(0.5)  # Allow servo to reach position

def main():
    try:
        while True:
            angle = float(input("Enter angle (0-180): "))
            if 0 <= angle <= 180:
                set_servo_angle(angle)
            else:
                print("Invalid angle! Enter a value between 0 and 180.")

    except KeyboardInterrupt:
        print("\nExiting...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Turn off servo
        pi.stop()


if __name__ == '__main__':
    main()