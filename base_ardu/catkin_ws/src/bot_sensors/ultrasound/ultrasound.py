#!/usr/bin/env python

import time
import math
from datetime import datetime
import rospy
from std_msgs.msg import String,Float32
import RPi.GPIO as GPIO

PUBLISH_RATE = 10 # Hz
GPIO_PIN_SIG = 0
EMIT_LOW_DURATION = 0.2
EMIT_HIGH_DURATION = 0.5
SOUND_SPEED_CM = 34300 # (cm/s)
TOPIC_NAME = ""

def measurementInCM():

    # setup the GPIO_PIN_SIG as output
    GPIO.setup(GPIO_PIN_SIG, GPIO.OUT)

    GPIO.output(GPIO_PIN_SIG, GPIO.LOW)
    time.sleep(EMIT_LOW_DURATION)
    GPIO.output(GPIO_PIN_SIG, GPIO.HIGH)
    time.sleep(EMIT_HIGH_DURATION)
    GPIO.output(GPIO_PIN_SIG, GPIO.LOW)
    start = time.time()

    # setup GPIO_PIN_SIG as input
    GPIO.setup(GPIO_PIN_SIG, GPIO.IN)

    # get duration from Ultrasonic SIG pin
    while GPIO.input(GPIO_PIN_SIG) == 0:
        start = datetime.now()

    while GPIO.input(GPIO_PIN_SIG) == 1:
        stop = datetime.now()

    return measurementPulse(start, stop)


def measurementPulse(start, stop):

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s)
    distance = math.floor(elapsed.total_seconds()) + (1.0 * elapsed.microseconds / pow(10, 6)) * SOUND_SPEED_CM

    # That was the distance there and back so halve the value
    return distance / 2.0

def ultrasound():

    rospy.init_node('ultrasound', anonymous=True)

    global PUBLISH_RATE
    PUBLISH_RATE = rospy.get_param('~PUBLISH_RATE', 10)
    global GPIO_PIN_SIG
    GPIO_PIN_SIG = rospy.get_param('~GPIO_PIN_SIG')
    global TOPIC_NAME
    TOPIC_NAME = rospy.get_param('~TOPIC_NAME', 'bot_sensors_ultrasound')

    pub = rospy.Publisher(TOPIC_NAME, Float32, queue_size=10)

    rospy.loginfo(">>> Ultrasound initialized <<<")
    rospy.loginfo("GPIO_PIN_SIG %d" % GPIO_PIN_SIG)
    rospy.loginfo("TOPIC_NAME %s" % TOPIC_NAME)
    rospy.loginfo("PUBLISH_RATE %d" % PUBLISH_RATE)

    rate = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        distance = measurementInCM()
        if distance is not None:
            pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    try:
        ultrasound()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
