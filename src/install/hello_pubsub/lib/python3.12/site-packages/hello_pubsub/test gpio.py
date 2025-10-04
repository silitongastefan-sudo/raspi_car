import RPi.GPIO as GPIO
import time

led_pin = 17  # GPIO pin to which a LED is connected

GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

while True:
    GPIO.output(led_pin, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(led_pin, GPIO.LOW)
    time.sleep(1)

GPIO.cleanup()