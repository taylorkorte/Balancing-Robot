#install these (terminal) before running code
#sudo pip3 install adafruit-circuitpython-pca9685
#sudo pip3 install adafruit-circuitpython-servokit
#(unless running on a pi where this was already done)

import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit


kit = ServoKit(channels=16)

#on servo hat, plug the servos into the 0 and 1 pin sections
continuous = kit.continuous_servo[0]
kickstand = kit.servo[1]

#full throttle of the servo, 1= fwd, -1 = reverse
continuous.throttle = 1
while True:
    kickAngle = input("Hi! please specify kickstand position angle. :) ")
    kickstand.angle = kickAngle