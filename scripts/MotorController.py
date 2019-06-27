#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

class MotorDC:
    def __init__(self, pin_a, pin_b):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin_a, GPIO.OUT)
        GPIO.setup(pin_b, GPIO.OUT)

        self.pinA = GPIO.PWM(pin_a, 100)
        self.pinB = GPIO.PWM(pin_b, 100)

        self.pinA.start(0)
        self.pinB.start(0)

    def drive(self, speed):
        if speed >= 0:
            self.pinA.ChangeDutyCycle(speed)
            self.pinB.ChangeDutyCycle(0)
        else:
            self.pinA.ChangeDutyCycle(0)
            self.pinB.ChangeDutyCycle(-speed)

motor = 0

def callback(data):
    motor.drive(data.data)

def listener():
    rospy.init_node('dc_motor')

    rospy.Subscriber("motor_speed", Int16, callback)

    global motor
    motor = MotorDC(15, 16)

    rospy.spin()

    GPIO.cleanup()

if __name__ == '__main__':
    listener()