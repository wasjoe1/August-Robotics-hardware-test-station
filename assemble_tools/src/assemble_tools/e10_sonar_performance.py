#!/usr/bin/env python
# encoding=utf-8

import numpy as np
import rospy

logger = rospy


class FirstOrderKalmanFilter(object):
    def __init__(self, Q, R, x0=0, p0=0, A=1, H=1):
        self.param = {
            "Q": Q,
            "R": R,
            "A": A,
            "H": H,
            "X": x0,
            "P": p0,
        }
        self.K = None

    @staticmethod
    def kf(z, X, P, Q, R, A=1, H=1, BU=0, W=0):
        X10 = A * X + BU + W
        P10 = A * P * A + Q

        K = P10 * H / (H * P10 * H + R)
        X1 = X10 + K * (z - H * X10)
        P1 = (1 - K * H) * P10
        return X1, P1, K

    def update_observed(self, z):
        x, p, k = self.kf(z, **self.param)
        self.param.update(
            {
                "X": x,
                "P": p,
            }
        )
        self.K = k
        return x

    def get_current_value(self):
        return self.param["X"]


from boothbot_msgs.ros_interfaces import (
    DRIVERS_SONARS_F_01,
    DRIVERS_SONARS_F_02,
    DRIVERS_SONARS_F_03,
    DRIVERS_SONARS_LEFT_01,
    DRIVERS_SONARS_LEFT_02,
    DRIVERS_SONARS_R_01,
    DRIVERS_SONARS_R_02,
    DRIVERS_SONARS_R_03,
    DRIVERS_SONARS_RIGHT_01,
    DRIVERS_SONARS_RIGHT_02,
    DRIVERS_SONARS_REAR_ON,
)


class SonarPerformanceChecker(object):
    INTERFACE_LIST = [
        DRIVERS_SONARS_F_01,
        DRIVERS_SONARS_F_02,
        DRIVERS_SONARS_F_03,
        DRIVERS_SONARS_LEFT_01,
        DRIVERS_SONARS_LEFT_02,
        DRIVERS_SONARS_R_01,
        DRIVERS_SONARS_R_02,
        DRIVERS_SONARS_R_03,
        DRIVERS_SONARS_RIGHT_01,
        DRIVERS_SONARS_RIGHT_02,
    ]
    NAME_LIST = [
        "f_01",
        "f_02",
        "f_03",
        "l_01",
        "l_02",
        "b_01",
        "b_02",
        "b_03",
        "r_01",
        "r_02",
    ]
    RESULT_TEMPLATE = """
    Sonar {0} filtered distance is {1}, calculated by {2} data
        Standard deviation  : {3}
        Outliners with sigma: {4}
    """

    def __init__(self, test_duration, Q=1e-5, R=1e-3):
        super(SonarPerformanceChecker, self).__init__()
        self.test_duration = test_duration
        logger.logwarn("Test duration is: {}".format(self.test_duration))
        self.timeout = rospy.Time.now()
        self.Q = Q
        self.R = R
        self.runtime_data = {
            # "name": [FOKF, RAW_DATA_LIST]
        }

    def _sonars_common_cb(self, msg, runtime_data):
        range = msg.range
        runtime_data[0].update_observed(range)
        runtime_data[1].append(range)

    def initialize(self):
        for name, inf in zip(self.NAME_LIST, self.INTERFACE_LIST):
            self.runtime_data[name] = [FirstOrderKalmanFilter(Q=self.Q, R=self.R), []]
            logger.logwarn("Subscribing: {} from {}".format(name, inf))
            inf.Subscriber(
                callback=self._sonars_common_cb, callback_args=self.runtime_data[name]
            )

    def run(self):
        self.timeout = rospy.Time.now() + rospy.Duration(self.test_duration)
        logger.logwarn("Test starting at: {}".format(rospy.Time.now().to_sec()))
        while not rospy.is_shutdown() and rospy.Time.now() < self.timeout:
            # Count all received msgs
            # using kf to find the best stable result
            rospy.sleep(5.0)
            logger.logwarn("Now at: {}".format(rospy.Time.now().to_sec()))
        logger.logwarn("Test ending at: {}".format(rospy.Time.now().to_sec()))
        self.show_result()

    def show_result(self):
        logger.logwarn("Results are:")
        for name, var in self.runtime_data.items():
            data_array = np.array(var[1])
            std = data_array.std(dtype=np.float64)
            outliners = (np.abs(data_array - data_array.mean()) > std).sum()
            logger.logwarn(
                self.RESULT_TEMPLATE.format(
                    name, var[0].get_current_value(), data_array.size, std, outliners
                )
            )

    def start(self):
        self.initialize()
        self.run()


if __name__ == "__main__":
    rospy.init_node("test_sonars_performance", log_level=rospy.INFO)
    test_duration = rospy.get_param("~test_duration", 5)
    spc = SonarPerformanceChecker(test_duration=test_duration)
    spc.start()
