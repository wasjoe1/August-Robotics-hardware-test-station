#!/usr/bin/python
# Power monitor unit setting
# AT+MODE+OVER(V/P/C)           set alarm mode
#         UNDER(V/P/C)
# AT+OVER(V/P/C)ERR+(number)    set alarm threshold
# AT+PLOAR+(0/1)                set relay switch
# AT+V                          check voltage

import re
import rospy
import serial
import std_msgs.msg as stmsgs
from common import Logging, errcode
from common.ros_topics import MB_VOLTAGE

# NOTE: 
# - This code is copied from boothbot_driver/scripts/battery_monitor_node.py with minor changes. 
# - This node currently assumes that the battery voltage remains constant and above the threshold.

class Battery_monitor(Logging):
    connected = False
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, rate=1):
        rospy.init_node('battery_node', anonymous=False)
        super(Battery_monitor, self).__init__("Battery")

        self.port = rospy.get_param("~voltage_port", port)
        self.vol_warning_level = float(rospy.get_param("~voltage_warning", 24.1))

        rospy.sleep(0.1)
        self.loginfo("Battery monitor connect.")
        if self.setup():
            self.connected = True

        self.rate = float(rospy.get_param('~rate', rate))
        self.r = rospy.Rate(self.rate)
        self.vol_pub = rospy.Publisher(MB_VOLTAGE, stmsgs.Float32, queue_size=5)
        self._voltage = 0.

    def run(self):
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown() and self.connected:
            self.voltage_update()
            self.r.sleep()

    def setup(self):
        rospy.sleep(0.1)
        self.loginfo('Set mode successful.')
        rospy.sleep(0.2)
        rospy.sleep(0.1)
        self.loginfo('Set current threshold successful.')
        rospy.sleep(0.2)
        rospy.sleep(0.1)
        self.loginfo('Set relay switch successful.')
        rospy.sleep(0.2)
        return True

    def voltage_update(self):
        try:
            self._voltage = self.vol_warning_level + 1.0

            if self._voltage < self.vol_warning_level:
                self.logwarn_throttle(2., 'Low voltage: {:0.3f} V!!'.format(self._voltage))
                self.logerrcode(errcode.CH_FID,
                                errcode.ErrCode.CH_ERR_LOW_BATTERY.value,
                                throttle_period=2.)
            self.vol_pub.publish(float(self._voltage))

        except Exception as e:
            self.logwarn("Read battery voltage failure")

    def shutdown(self):
        self.loginfo('Shutting down.')
        pass

if __name__ == '__main__':
    bm = Battery_monitor()
    bm.run()
