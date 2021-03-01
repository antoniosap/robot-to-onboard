#!/usr/bin/env python
#
# original from github
# revisions antonio s:
#   28.12.2018 - code refactory
#
# string publishing
# sites https://answers.ros.org/question/170836/why-cant-i-send-123-via-rostopic-pub/
#

import time
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from sense_hat import SenseHat


# --------------------------------------------------------------------------
def pix_next_color(colorarray):
    r = colorarray[0]
    g = colorarray[1]
    b = colorarray[2]
    if (r == 255 and g < 255 and b == 0):
        g += 1

    if (g == 255 and r > 0 and b == 0):
        r -= 1

    if (g == 255 and b < 255 and r == 0):
        b += 1

    if (b == 255 and g > 0 and r == 0):
        g -= 1

    if (b == 255 and r < 255 and g == 0):
        r += 1

    if (r == 255 and b > 0 and g == 0):
        b -= 1

    colorarray[0] = r
    colorarray[1] = g
    colorarray[2] = b


def initialize_plain_color():
    return [255, 0, 0]


def initialize_rainbow_color():
    return [
        [255, 0, 0], [255, 0, 0], [255, 87, 0], [255, 196, 0], [205, 255, 0], [95, 255, 0], [0, 255, 13], [0, 255, 122],
        [255, 0, 0], [255, 96, 0], [255, 205, 0], [196, 255, 0], [87, 255, 0], [0, 255, 22], [0, 255, 131],
        [0, 255, 240],
        [255, 105, 0], [255, 214, 0], [187, 255, 0], [78, 255, 0], [0, 255, 30], [0, 255, 140], [0, 255, 248],
        [0, 152, 255],
        [255, 223, 0], [178, 255, 0], [70, 255, 0], [0, 255, 40], [0, 255, 148], [0, 253, 255], [0, 144, 255],
        [0, 34, 255],
        [170, 255, 0], [61, 255, 0], [0, 255, 48], [0, 255, 157], [0, 243, 255], [0, 134, 255], [0, 26, 255],
        [83, 0, 255],
        [52, 255, 0], [0, 255, 57], [0, 255, 166], [0, 235, 255], [0, 126, 255], [0, 17, 255], [92, 0, 255],
        [201, 0, 255],
        [0, 255, 66], [0, 255, 174], [0, 226, 255], [0, 117, 255], [0, 8, 255], [100, 0, 255], [210, 0, 255],
        [255, 0, 192],
        [0, 255, 183], [0, 217, 255], [0, 109, 255], [0, 0, 255], [110, 0, 255], [218, 0, 255], [255, 0, 183],
        [255, 0, 74]
    ]


# --------------------------------------------------------------------------
def get_temperature(sense):
    return sense.get_temperature()


def get_humidity(sense):
    return sense.get_humidity()


def get_pressure(sense):
    return sense.get_pressure()


def get_gyroscope(sense):
    return sense.get_gyroscope()


def get_stick(sense):
    stickEvents = sense.stick.get_events()
    if (len(stickEvents) > 0):
        return stickEvents[-1]
    else:
        return None


def get_accelerometer(sense):
    return sense.get_accelerometer_raw()


def get_compass(sense):
    return sense.get_compass()


def get_gyroscope(sense):
    return sense.get_gyroscope_raw()


def get_magnetometer(sense):
    return sense.get_compass_raw()


def get_orientation(sense):
    return sense.get_orientation()


def get_orientation_radians(sense):
    return sense.get_orientation_radians()


def get_orientation_degrees(sense):
    return sense.get_orientation_degrees()


def sensorsDataToVector3(data):
    return Vector3(data['x'], data['y'], data['z'])


# --------------------------------------------------------------------------
# info sites:
# https://pymotw.com/2/threading/

def cbLedPanel(msg):
    senseCb = SenseHat()
    senseCb.set_rotation(90)
    if len(msg.data) > 0:
       senseCb.show_message(sense.show_message(msg.data))


if __name__ == '__main__':
    sense = SenseHat()
    rosFrequency = 10
    rosNodeName = "sensehat"
    rospy.init_node(rosNodeName, log_level=rospy.INFO)
    rospy.loginfo(rospy.get_caller_id() + " Starting @ frequency %s Hz", rosFrequency)
    rate = rospy.Rate(rosFrequency)
    # publishers
    pubHumidity = rospy.Publisher('sensehat/humidity', Float64, queue_size=10)
    pubTemperature = rospy.Publisher('sensehat/temperature', Float64, queue_size=10)
    pubPressure = rospy.Publisher('sensehat/pressure', Float64, queue_size=10)
    pubAccelerometer = rospy.Publisher('sensehat/accelerometer', Vector3, queue_size=10)
    pubGyroscope = rospy.Publisher('sensehat/gyroscope', Vector3, queue_size=10)
    pubMagnetometer = rospy.Publisher('sensehat/magnetometer', Vector3, queue_size=10)
    pubCompass = rospy.Publisher('sensehat/compass', Float64, queue_size=10)
    pubStick = rospy.Publisher('sensehat/stick', String, queue_size=10)
    # listeners
    rospy.Subscriber("sensehat/led_panel", String, cbLedPanel, queue_size=10)
    #
    try:
        rospy.loginfo(rospy.get_caller_id() + " Starting")
        while not rospy.is_shutdown():
            pubHumidity.publish(get_humidity(sense))
            pubTemperature.publish(get_temperature(sense))
            pubPressure.publish(get_pressure(sense))

            pubAccelerometer.publish(sensorsDataToVector3(get_accelerometer(sense)))
            pubGyroscope.publish(sensorsDataToVector3(get_gyroscope(sense)))
            pubMagnetometer.publish(sensorsDataToVector3(get_magnetometer(sense)))
            pubCompass.publish(get_compass(sense))

            stickEvent = get_stick(sense)
            if (stickEvent is not None):
                pubStick.publish(stickEvent.direction)

            rate.sleep()

        rospy.loginfo(rospy.get_caller_id() + " exit")
    except rospy.ROSInterruptException:
        pass
