import RPi.GPIO as GPIO
import time

# Pin Definitions
input_pin = 18  # BCM pin 18, BOARD pin 12

def main():
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            value = GPIO.input(input_pin)
            print(value)
            
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()