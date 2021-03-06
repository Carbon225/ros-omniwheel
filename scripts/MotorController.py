#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

class MotorDC:
    def __init__(self, pin_a, pin_b, pin_en):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin_a, GPIO.OUT)
        GPIO.setup(pin_b, GPIO.OUT)
        GPIO.setup(pin_en, GPIO.OUT)

        self.pinA = GPIO.PWM(pin_a, 100)
        self.pinB = GPIO.PWM(pin_b, 100)
        self.pinEn = pin_en

        self.pinA.start(0)
        self.pinB.start(0)

    def drive(self, speed):
        if speed > 0:
            GPIO.output(self.pinEn, 1)
            self.pinA.ChangeDutyCycle(speed)
            self.pinB.ChangeDutyCycle(0)
        elif speed < 0:
            GPIO.output(self.pinEn, 1)
            self.pinA.ChangeDutyCycle(0)
            self.pinB.ChangeDutyCycle(abs(speed))
        else:
            GPIO.output(self.pinEn, 0)
            self.pinA.ChangeDutyCycle(0)
            self.pinB.ChangeDutyCycle(0)

motor = 0

def callback(data):
    motor.drive(data.data)

def listener():
    rospy.init_node('dc_motor')

    rospy.Subscriber("motor_speed", Int16, callback)

    pin_a = rospy.get_param('~pin_a')
    pin_b = rospy.get_param('~pin_b')
    pin_en = rospy.get_param('~pin_en')

    global motor
    motor = MotorDC(pin_a, pin_b, pin_en)

    rospy.spin()

    GPIO.cleanup()

if __name__ == '__main__':
    listener()