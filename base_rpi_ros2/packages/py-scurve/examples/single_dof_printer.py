import asyncio
import numpy as np
import logging
from matplotlib import pyplot as plt
from pyscurve import ScurvePlanner, plot_trajectory


class SCurve1AxisMotion(ScurvePlanner):
    ACCELERATION_ID = 0
    SPEED_ID = 1
    POSITION_ID = 2

    def __init__(self, axis_name, debug=False, dt=0.3):
        super().__init__(debug)
        self._axis_name = axis_name
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
        for t in range(0, len(self._profile)):
            print(self._axis_name, self._profile[t])
            await asyncio.sleep(self._dt)

    async def asy_exec_trajectory(self):
        await self._exec_t()


class SCurve2AxisMotion:
    def __init__(self, axis1_name, axis2_name, debug=False, dt=0.3):
        self._axis1 = SCurve1AxisMotion(axis1_name)
        self._axis2 = SCurve1AxisMotion(axis2_name)

    def recall(self, label):
        return self._axis1.recall(label), self._axis2.recall(label)

    def store(self, label):
        self._axis1.store(label)
        self._axis2.store(label)

    def do_plan_trajectory(self, a_q0, a_q1, b_q0, b_q1):
        self._axis1.do_plan_trajectory(a_q0, a_q1)
        self._axis2.do_plan_trajectory(b_q0, b_q1)

    async def asy_exec_trajectory(self):
        await asyncio.gather(self._axis1.asy_exec_trajectory(),
                             self._axis2.asy_exec_trajectory())


if __name__ == "__main__":
    # cam_pan_range = (32, 120)
    # cam_tilt_range = (70, 185)
    # sequential
    print("sequential exec test")
    axis_pan = SCurve1AxisMotion('pan')
    axis_pan.do_plan_trajectory(32, 120)
    asyncio.run(axis_pan.syn_exec_trajectory())
    axis_tilt = SCurve1AxisMotion('tilt')
    axis_tilt.do_plan_trajectory(70, 185)
    asyncio.run(axis_tilt.syn_exec_trajectory())
    # parallel
    print("parallel exec test")
    two_axis = SCurve2AxisMotion('pan', 'tilt')
    two_axis.do_plan_trajectory(a_q0=32, a_q1=120, b_q0=70, b_q1=185)
    asyncio.run(two_axis.asy_exec_trajectory())
    # test profile storage
    two_axis.store('test')
    print(two_axis.recall('test'))
