#!/usr/bin/env python3

""" mobile base utils """

__author__ = "Antonio Sapuppo"
__copyright__ = "Copyright 2021"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Antonio Sapuppo"
__email__ = "antoniosapuppo@yahoo.it"
__status__ = "Development"

import os
import rospy
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Bool, String, Int16

# ---------------------------------------------------------------------
# sites info:
# https://askubuntu.com/questions/168879/shutdown-from-terminal-without-entering-password
# https://github.com/halofx/rpi-shutdown/blob/master/shutdown.py
#


class BaseOnBoard:
    def __init__(self):
        self.node_name = 'base_rpi'
        # registering node in ros master
        rospy.init_node(self.node_name, log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        # pub
        self.pub_display8x8 = rospy.Publisher('/sensehat/led_panel', String, queue_size=10)
        self.pub_stick = rospy.Publisher('/base/stick', String, queue_size=1)
        self.pub_cam_light_led = rospy.Publisher('/base/cam_light_led', Bool, queue_size=10)
        self.pub_cam_pan = rospy.Publisher('base/cam_pan', Int16, queue_size=10)
        self.pub_cam_tilt = rospy.Publisher('base/cam_tilt', Int16, queue_size=10)
        # sub - Don't subscribe until everything has been initialized.
        rospy.Subscriber("/base/btn_shutdown", Bool, btn_shutdown)
        rospy.Subscriber("/sensehat/stick", String, btn_stick)

    @staticmethod
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)

    def servo_publish(self, pan, tilt):
        self.pub_cam_pan.publish(pan)
        self.pub_cam_tilt.publish(tilt)

    def home_off(self):
        self.servo_publish(pan=90, tilt=185)

    def home_on(self):
        self.servo_publish(pan=90, tilt=90)

    def shutdown(self, data):
        rospy.loginfo(f'{rospy.get_caller_id()} shutdown button {data.data}')
        status = str(data.data) == 'True'
        if status:
            msg = 'shutdown in progress'
            rospy.loginfo(f'{rospy.get_caller_id()} {msg}')
            self.pub_display8x8.publish(msg)
            self.pub_cam_light_led.publish(False)
            self.home_off()
            os.system("sudo shutdown -h 1")

    def btn_stick(self, data):
        # change axis reference
        status = str(data.data)
        self.pub_stick = rospy.Publisher('/base/stick', String, queue_size=1)
        if status == 'up':
            self.pub_stick.publish('right')
        elif status == 'down':
            self.pub_stick.publish('left')
        elif status == 'right':
            self.pub_stick.publish('down')
        elif status == 'left':
            self.pub_stick.publish('up')

    def spin(self):
        rospy.loginfo(f'{self.node_name} Starting: please, arm motors')
        rospy.sleep(20)
        # begin node code
        rospy.loginfo(f'{self.node_name} Started')
        self.home_on()
        rate = rospy.Rate(10)  # hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    base_rpi = BasePc()
    try:
        base_rpi.spin()
    except Exception as error:
        rospy.logerr(f'{base_pc.node_name} Error on Main: {error}')
    except rospy.ROSInterruptException:
        rospy.loginfo(f'{base_pc.node_name} Shutdown')
