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
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------
# sites info:
# https://askubuntu.com/questions/168879/shutdown-from-terminal-without-entering-password
# https://github.com/halofx/rpi-shutdown/blob/master/shutdown.py
#


def btn_shutdown(data):
    rospy.loginfo(f'{rospy.get_caller_id()} shutdown button {data.data}')
    status = str(data.data) == 'True'
    if status:
        msg = 'shutdown in progress'
        rospy.loginfo(f'{rospy.get_caller_id()} {msg}')
        pub_display8x8 = rospy.Publisher('sensehat/led_panel', String, queue_size=10)
        pub_display8x8.publish(msg)
        pub_cam_light_led = rospy.Publisher('/base/cam_light_led', Bool, queue_size=10)
        pub_cam_light_led.publish(False)
        os.system("sudo shutdown -h 1")


if __name__ == '__main__':
    node_name = rospy.get_caller_id()
    # registering node in ros master
    rospy.init_node(node_name, log_level=rospy.INFO)
    # begin node code
    rospy.loginfo(f'{node_name} Starting ')
    #
    rospy.Subscriber("/base/btn_shutdown", Bool, btn_shutdown)
    #
    pub_display8x8 = rospy.Publisher('sensehat/led_panel', String, queue_size=10)
    #
    rate = rospy.Rate(1)  # hz
    try:
        while not rospy.is_shutdown():

            rate.sleep()
    except Exception as error:
        rospy.logerr(f'{node_name} Error on Main: {error}')
    except rospy.ROSInterruptException:
        rospy.loginfo(f'{node_name} Shutdown')
