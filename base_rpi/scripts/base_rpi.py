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
import asyncio
import numpy as np
from pyscurve import ScurvePlanner
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Bool, String, Int16

# ---------------------------------------------------------------------
# sites info:
# https://askubuntu.com/questions/168879/shutdown-from-terminal-without-entering-password
# https://github.com/halofx/rpi-shutdown/blob/master/shutdown.py
#


class SCurve1AxisMotion(ScurvePlanner):
    ACCELERATION_ID = 0
    SPEED_ID = 1
    POSITION_ID = 2

    def __init__(self, axis_name, debug=False, dt=0.3, callback=print):
        super().__init__(debug)
        self._axis_name = axis_name
        self._enable = True
        self._q0 = [32.]
        self._q1 = [120.]
        self._v0 = [1.]
        self._v1 = [5.]
        self._v_max = 20.
        self._a_max = 15.
        self._j_max = 100.
        self._dt = dt
        self._profile = None
        self._store = {}
        self._callback = callback

    @property
    def enable(self):
        return self._enable

    @enable.setter
    def enable(self, value):
        self._enable = value

    @property
    def q0(self):
        return self._q0[0]

    @q0.setter
    def q0(self, value):
        self._q0[0] = value

    @property
    def q1(self):
        return self._q1[0]

    @q1.setter
    def q1(self, value):
        self._q1[0] = value

    @property
    def v0(self):
        return self._v0[0]

    @v0.setter
    def v0(self, value):
        self._v0[0] = value

    @property
    def v1(self):
        return self._v1[0]

    @v1.setter
    def v1(self, value):
        self._v1[0] = value

    @property
    def v_max(self):
        return self._v_max

    @v_max.setter
    def v_max(self, value):
        self._v_max = value

    @property
    def a_max(self):
        return self._a_max

    @a_max.setter
    def a_max(self, value):
        self._a_max = value

    @property
    def j_max(self):
        return self._j_max

    @j_max.setter
    def j_max(self, value):
        self._j_max = value

    @property
    def dt(self):
        return self._dt

    @dt.setter
    def dt(self, value):
        self._dt = value

    def recall(self, label):
        return self._store[label]

    def store(self, label):
        self._store[label] = self._profile

    @property
    def profile(self):
        return self._profile

    def do_plan_trajectory(self, q0, q1):
        if q0 is not None:
            self.q0 = q0
        if q1 is not None:
            self.q1 = q1
        if q0 == q1:
            self._enable = False
            self._profile = None
            return
        self._enable = True
        traj = self.plan_trajectory(self._q0, self._q1, self._v0, self._v1, self._v_max, self._a_max, self._j_max)
        dof = traj.dof
        timesteps = int(max(traj.time) / self.dt)
        time = np.linspace(0, max(traj.time), timesteps)

        # NOW
        # profiles[t]           --- profiles for each DOF at time x[t]
        # profiles[t][d]        --- profile for d DOF at time x[t]
        # profiles[t][d][k]     --- accel/vel/pos profile for d DOF at time x[t]
        p_list = [traj(t) for t in time]
        profiles = np.asarray(p_list)

        # NEED
        # profiles[d]       --- profiles for each DOF 0 <= d <= DOF number
        # profiles[d][k]    --- accel/vel/pos profile for DOF d where j
        # profiles[d][k][t] --- accel/vel/pos at time x[k] for DOF i
        # profiles = np.reshape(profiles, (dof, 3, timesteps))
        r_profiles = np.zeros((dof, 3, timesteps))
        for d in range(dof):
            for p in range(3):
                r_profiles[d, p, :] = profiles[:, d, p]
        # print(self._timesteps)
        # print(time)
        # print(max(traj.time))
        self._profile = np.round(r_profiles[0][self.POSITION_ID]).astype(np.int16)

    def reverse_trajectory(self):
        self._profile = self._profile[::-1]

    async def syn_exec_trajectory(self):
        await asyncio.create_task(self._exec_t())

    async def _exec_t(self):
        if self._profile is not None and self._enable:
            for t in range(0, len(self._profile)):
                self._callback(self._axis_name, self._profile[t])
                await asyncio.sleep(self._dt)

    async def asy_exec_trajectory(self):
        await self._exec_t()


class SCurve2AxisMotion:
    def __init__(self, axis1_name, axis2_name, debug=False, dt=0.3,
                 axis1_callback=print, axis2_callback=print):
        self._axis1 = SCurve1AxisMotion(axis1_name, dt=dt, callback=axis1_callback)
        self._axis2 = SCurve1AxisMotion(axis2_name, dt=dt, callback=axis2_callback)

    @property
    def axis1(self):
        return self._axis1

    @property
    def axis2(self):
        return self._axis2

    def recall(self, label):
        return self._axis1.recall(label), self._axis2.recall(label)

    def store(self, label):
        self._axis1.store(label)
        self._axis2.store(label)

    def do_plan_trajectory(self, pan_q0, pan_q1, tilt_q0, tilt_q1):
        self._axis1.do_plan_trajectory(pan_q0, pan_q1)
        self._axis2.do_plan_trajectory(tilt_q0, tilt_q1)

    async def asy_exec_trajectory(self):
        await asyncio.gather(self._axis1.asy_exec_trajectory(),
                             self._axis2.asy_exec_trajectory())


class BaseOnBoard:
    cam_pan_range = (32, 120)
    cam_tilt_range = (70, 185)
    cam_off = (90, 90)
    cam_on = (90, 185)
    spin_hertz = 10

    def __init__(self):
        self.node_name = 'base_rpi'
        # registering node in ros master
        rospy.init_node(self.node_name, log_level=rospy.INFO)
        self.shutdown_request = False
        self.shutdown_counter = self.spin_hertz * 60  # 10 hz * 60 sec
        # begin node code
        self.cam_cur = self.cam_off       # off position [pan, tilt]
        self.two_axis = SCurve2AxisMotion('pan', 'tilt', dt=0.3, debug=True,
                                          axis1_callback=self.axis1_cb, axis2_callback=self.axis2_cb)
        # off to on pose
        self.two_axis.do_plan_trajectory(pan_q0=self.cam_cur[0], pan_q1=self.cam_on[0],
                                         tilt_q0=self.cam_cur[1], tilt_q1=self.cam_on[1])
        self.two_axis.store('home_on')
        self.axis_move_exec = False
        # pub
        self.pub_display8x8 = rospy.Publisher('/sensehat/led_panel', String, queue_size=10)
        self.pub_stick = rospy.Publisher('/base/stick', String, queue_size=1)
        self.pub_cam_light_led = rospy.Publisher('/base/cam_light_led', Bool, queue_size=10)
        self.pub_cam_pan = rospy.Publisher('base/cam_pan', Int16, queue_size=10)
        self.pub_cam_tilt = rospy.Publisher('base/cam_tilt', Int16, queue_size=10)
        #
        rospy.sleep(20)  # wait processes spinning and board card started
        self.home_on()
        # sub - Don't subscribe until everything has been initialized.
        rospy.Subscriber("/base/btn_shutdown", Bool, self.btn_shutdown)
        rospy.Subscriber("/sensehat/stick", String, self.btn_stick)

    @staticmethod
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)

    def servo_publish(self, pan, tilt):
        self.pub_cam_pan.publish(pan)
        self.pub_cam_tilt.publish(tilt)

    # pan axis
    def axis1_cb(self, axis_name, axis_position):
        self.pub_cam_pan.publish(axis_position)

    # tilt axis
    def axis2_cb(self, axis_name, axis_position):
        self.pub_cam_tilt.publish(axis_position)

    def home_off(self):
        self.two_axis.do_plan_trajectory(pan_q0=self.cam_cur[0], pan_q1=self.cam_off[0],
                                         tilt_q0=self.cam_cur[1], tilt_q1=self.cam_off[1])
        self.axis_move_exec = True  # move to off pose

    def home_on(self):
        self.two_axis.recall('home_on')
        self.axis_move_exec = True  # move to on pose

    def btn_shutdown(self, data):
        rospy.logdebug(f'{rospy.get_caller_id()} shutdown button {data}')
        if data and not self.shutdown_request:
            self.shutdown_request = True
            msg = 'shutdown in progress'
            rospy.loginfo(f'{rospy.get_caller_id()} {msg}')
            self.pub_display8x8.publish(msg)
            self.pub_cam_light_led.publish(False)
            self.home_off()

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

    async def asy_axis_spin(self):
        try:
            rospy.loginfo(f'{self.node_name} Starting: asy_axis_spin')
            rospy.sleep(10)   # wait processes spinning and board card 'base_ardu' started
            rospy.loginfo(f'{self.node_name} Started: asy_axis_spin')
            rate = rospy.Rate(self.spin_hertz)  # hz
            while not rospy.is_shutdown():
                if self.axis_move_exec:
                    rospy.loginfo(f'{self.node_name} -----> 1')
                    await self.two_axis.asy_exec_trajectory()
                    rospy.loginfo(f'{self.node_name} -----> 2 {self.two_axis.axis1.q1} {self.two_axis.axis2.q2}')
                    self.cam_cur = (self.two_axis.axis1.q1, self.two_axis.axis2.q1)
                    rospy.loginfo(f'{self.node_name} -----> 3')
                    self.axis_move_exec = False
                rate.sleep()
        except Exception as error:
            rospy.logerr(f'{base_rpi.node_name} asy_axis_spin: {error}')
        except rospy.ROSInterruptException:
            rospy.loginfo(f'{base_rpi.node_name} Shutdown asy_axis_spin')

    async def asy_ros_spin(self):
        try:
            rospy.loginfo(f'{self.node_name} Starting: asy_ros_spin')
            rospy.sleep(5)
            rospy.loginfo(f'{self.node_name} Started: asy_ros_spin')
            rate = rospy.Rate(self.spin_hertz)  # hz
            while not rospy.is_shutdown():
                if self.shutdown_request:
                    rospy.loginfo(f'{self.node_name} {self.shutdown_counter}')
                    if self.shutdown_counter > 0:
                        self.shutdown_counter -= 1
                    else:
                        exit(1)
                        # os.system("sudo shutdown -h 1")
                rate.sleep()
        except Exception as error:
            rospy.logerr(f'{base_rpi.node_name} asy_ros_spin: {error}')
        except rospy.ROSInterruptException:
            rospy.loginfo(f'{base_rpi.node_name} Shutdown asy_ros_spin')

    async def asy_run(self):
        await asyncio.gather(self.asy_axis_spin(),  # critical order (1)
                             self.asy_ros_spin())   # critical order (2)


if __name__ == '__main__':
    base_rpi = BaseOnBoard()
    asyncio.run(base_rpi.asy_run())
