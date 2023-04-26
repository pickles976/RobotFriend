from RPIO import PWM
from time import sleep

servo = PWM.Servo()

# 1000 -> 1500 -> 2000

# Set servo on GPIO32 to 2000µs (2ms)
servo.set_servo(32, 2000)

# Set servo on GPIO17 to 2000µs (2.0ms)
servo.set_servo(17, 2000)

while True:
    for i in range(0,10):
        servo.set_servo(32, 1000 + (i * 100))
        sleep(5)

# Clear servo on GPIO17
servo.stop_servo(17)