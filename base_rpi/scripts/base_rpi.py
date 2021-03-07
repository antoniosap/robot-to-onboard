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
from std_msgs.msg import Bool

# ---------------------------------------------------------------------
# sites info:
# https://askubuntu.com/questions/168879/shutdown-from-terminal-without-entering-password
# https://github.com/halofx/rpi-shutdown/blob/master/shutdown.py
#


def btn_shutdown(data):
    rospy.loginfo(rospy.get_caller_id() + " shutdown button ", data.data)
    if data.data == 'True':
        rospy.loginfo(rospy.get_caller_id() + " shutdown request")
        os.system("sudo shutdown -h now")


if __name__ == '__main__':
    node_name = 'base_rpi'
    # registering node in ros master
    rospy.init_node(node_name, log_level=rospy.INFO)
    # begin node code
    rospy.loginfo(f'Starting {node_name}')
    #
    rospy.Subscriber("/base/btn_shutdown", Bool, btn_shutdown)
    rate = rospy.Rate(1)  # hz
    try:
        while not rospy.is_shutdown():

            rate.sleep()
    except Exception as error:
        rospy.logerr("Error on Main: "+str(error))
    except rospy.ROSInterruptException:
        rospy.loginfo(f'Shutdown {node_name}')
