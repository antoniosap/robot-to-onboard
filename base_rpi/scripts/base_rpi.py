#!/usr/bin/env python3

""" mobile base utils """

__author__ = "Antonio Sapuppo"
__copyright__ = "Copyright 2021"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Antonio Sapuppo"
__email__ = "antoniosapuppo@yahoo.it"
__status__ = "Development"

import rospy
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Bool

# ---------------------------------------------------------------------


def btn_shutdown(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


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
