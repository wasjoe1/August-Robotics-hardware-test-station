#!/usr/bin/env python
#encoding=utf-8
from __future__ import division, print_function

import time
import math
import threading
import rospy
import tf
import numpy as np
import std_msgs.msg as stmsgs
import geometry_msgs.msg as gemsgs
import nav_msgs.msg as nvmsgs
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt16
from boothbot_driver.cfg import MobileBaseConfig
import dynamic_reconfigure.server as dyn
from fake_cq2_driver import FakeIMDRDriverSerial, ODOM_POSE_COVARIANCE, ODOM_TWIST_COVARIANCE

from common import Logging, errcode
from boothbot_msgs.ros_topics import ODOM_ENCODER, ODOM_IMU, MB_VOLTAGE, MB_HB, IMU_RESET_YAW_CMD, \
    MB_USING_ENCODER_YAW, IMU_WIT, VEL_SMOOTH_CMD, CHASSIS_RESET_CMD, CHASSIS_STATS, \
    EMERGENCY_STOP
from boothbot_msgs.ros_interfaces import DRIVERS_CHASSIS_STATUS, DRIVERS_CHASSIS_CMD_VEL, \
    DRIVERS_CHASSIS_ODOM, DRIVERS_USING_IMU_ODOM
from boothbot_msgs.msg import ErrCode, ChassisStats

from boothbot_msgs.ros_interfaces import DRIVERS_CHASSIS_SRV_CMD, DRIVERS_CHASSIS_SRV_IO
import augustbot_msgs.srv as absrvs

IMU_CONNECTION_TIMEOUT = 0.5

# NOTE: This code is copied from boothbot_driver/scripts/mobile_base_node.py with only minor changes.

""" Class to receive Twist commands and publish Odometry data """
class BaseController(Logging):
    def __init__(self, driver, odom_pub_cb, vol_pub_cb):
        super(BaseController, self).__init__('mb_0')
        self._driver = driver
        self.stopped = False
        self.fail_count = 0

        # Internal data
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.then = None
        self.v_des = np.array([0., 0., 0.])
        self.v_grab = np.array([0., 0., 0.])
        self.v_set = np.array([0., 0., 0.])
        # self.v_cur = np.array([0., 0., 0.])
        self.voltage = 0.
        self.vol_pub_rate = 1.
        self.chassis_stats = ChassisStats()

        self.MAX_FILTER = 3
        self.v_max = np.array([0.7, 0.4, 1.5])
        self.a_max = np.array([0.75, 0.4, 1.5])
        self._odom_pub = odom_pub_cb
        self._vol_pub = vol_pub_cb
        self._vol_pub_time = 0.

        # By Max, use IMU yaw to calculate X Y integration
        rospy.Subscriber(IMU_WIT, Imu, self._imu_cb)
        self.yaw_rate = 0
        # By shan, publish hangfa mileage
        self.mileage_pub = rospy.Publisher(CHASSIS_STATS, ChassisStats, queue_size = 5)



    # by Max, update IMU yaw
    def _imu_cb(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    def poll(self, use_imu_odom):
        ''' A funtion that update the status of the base, should be call cyclically
        '''
        try:
            ret_get_twist, v_cur = self._driver.get_twist()
        except Exception as e:
            self.logwarn(e)
            ret_get_twist = self._driver.FAIL

        if ret_get_twist == self._driver.SUCCESS:
            now = time.time()
            dt = now - self.then
            self.then = now
            v_cur = list(v_cur)
            if dt > 0.025:
                self.logwarn('dt is too big: {}s'.format(dt))

            dx = v_cur[0] * dt
            dy = v_cur[1] * dt
            if use_imu_odom:
                v_cur[2] = self.yaw_rate
            dth = v_cur[2] * dt

            # TODO: maybe there is a better way
            self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
            self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
            self.th += dth

            self._odom_pub((self.x, self.y, self.th), v_cur)

            # By shan, calculate mileage
            step = math.sqrt(dx * dx + dy * dy + 0.45*dth * 0.45*dth)
            self.chassis_stats.total_mileage.data += step
            self.mileage_pub.publish(self.chassis_stats)

            if not self.stopped:
                self.v_set = self._limiting(self.v_des, self.v_set, self.v_max, self.a_max*dt)
                self._driver.drive_twist(*self.v_set)

            if now > self._vol_pub_time:
                try:
                    ret, res = self._driver.get_voltage()
                    if ret == self._driver.SUCCESS:
                        self.voltage = res[0]*1e-3
                        self._vol_pub_time = now + 1./self.vol_pub_rate
                        self._vol_pub(self.voltage)
                except Exception as e:
                    self.logwarn('got vol but {}'.format(e))

    def _limiting(self, v_des, v_set, v_max, a_max):
        v_u_acc = [min(s, m) for s, m in zip(v_set+a_max, v_max)]
        v_l_dec = [max(s, m) for s, m in zip(v_set-a_max, -v_max)]
        v_des = [min(d, u) for d, u in zip(v_des, v_u_acc)]
        v_set = [max(d, l) for d, l in zip(v_des, v_l_dec)]
        return np.array(v_set)

    def stop(self):
        ret, res = self._driver.stop()
        if ret == self._driver.SUCCESS:
            self.stopped = True

    def set_twist(self, x, y, rz):
        self.v_des[0] = x
        self.v_des[1] = y
        self.v_des[2] = rz

    def set_dynamics(self, dynamics):
        if dynamics is not None:
            self.v_max[0] = dynamics['max_x_vel']
            self.v_max[1] = dynamics['max_y_vel']
            self.v_max[2] = dynamics['max_rz_vel']
            self.a_max[0] = dynamics['max_x_acc']
            self.a_max[1] = dynamics['max_y_acc']
            self.a_max[2] = dynamics['max_rz_acc']
            self.MAX_FILTER = dynamics['MAX_FILTER']
        return {
            'max_x_vel': float(self.v_max[0]),
            'max_y_vel': float(self.v_max[1]),
            'max_rz_vel': float(self.v_max[2]),
            'max_x_acc': float(self.a_max[0]),
            'max_y_acc': float(self.a_max[1]),
            'max_rz_acc': float(self.a_max[2]),
            'MAX_FILTER': int(self.MAX_FILTER),
        }

class MobileBaseNode(Logging):
    _mutex = threading.RLock()
    def __init__(self):
        super(MobileBaseNode, self).__init__('mb_0')
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/mobile_base")
        self.baud = int(rospy.get_param("~baudrate", 115200))
        self.base_frame = rospy.get_param("~base_frame", 'base_footprint')
        self.rate = float(rospy.get_param("~rate", 100))
        self.vol_warning_level = float(rospy.get_param("~voltage_warning", 24.1))
        self.loop = rospy.Rate(self.rate)

        self._use_imu_odom = False
        self.imu_is_connected = False
        self._error = False
        self._imu_yaw = 0
        self._imu_last_time = time.time()

        if self._use_imu_odom:
            rospy.Subscriber(CHASSIS_RESET_CMD, stmsgs.String, self._reset_cb)
            # Reset Yaw to zero inorder to use IMU to calculate Odom
            while not self._init_odom_imu():
                rospy.logwarn("resetting IMU...")

        self.driver = FakeIMDRDriverSerial(self.port, self.baud)

        # Make the connection
        self.logerrcode(errcode.CH_FID,
                        errcode.ErrCode.OK.value)

        rospy.loginfo("Connected to mobile base on port " + self.port + " at " + str(self.baud) + " baud")

        self.base_ctrl = BaseController(self.driver, self.publish_odom, self.publish_vol)

        # Command handling
        rospy.Service(DRIVERS_CHASSIS_SRV_CMD.name, absrvs.Command, self._srv_cb)

        # cmd_vel handling
        rospy.Subscriber(DRIVERS_CHASSIS_CMD_VEL.name,
                         DRIVERS_CHASSIS_CMD_VEL.type,
                         self._cmd_vel_cb)

        # Set up the odometry broadcaster
        self.odom_pub = rospy.Publisher(DRIVERS_CHASSIS_ODOM.name,
                                        DRIVERS_CHASSIS_ODOM.type,
                                        queue_size=1)
        self.last_odom = nvmsgs.Odometry()
        self.last_odom.header.frame_id = "odom_encoder"
        self.last_odom.child_frame_id = self.base_frame
        self.vol_pub = rospy.Publisher(MB_VOLTAGE, stmsgs.Float32, queue_size=1)
        self.voltage = stmsgs.Float32()
        self.tfb = tf.broadcaster.TransformBroadcaster()

        heartbeat_pub = rospy.Publisher(MB_HB, stmsgs.Empty, queue_size=1)
        using_imu_odom_pub = rospy.Publisher(
            DRIVERS_USING_IMU_ODOM.name, 
            DRIVERS_USING_IMU_ODOM.type, 
            queue_size=1
        )

        # Set up chassis status broadcaster.
        self.status_pub = rospy.Publisher(
            DRIVERS_CHASSIS_STATUS.name, DRIVERS_CHASSIS_STATUS.type, queue_size=1)

        # Set up service for handling IO commands.
        rospy.Service(DRIVERS_CHASSIS_SRV_IO.name, DRIVERS_CHASSIS_SRV_IO.type, self._srv_io_cb)

        # By Max, use encoder angular velocity z to calculate yaw for short distance moves
        rospy.Subscriber(MB_USING_ENCODER_YAW, Bool, self._short_dist_cb)
        self.short_dist_imu_odom = False

        # estop command
        rospy.Subscriber(EMERGENCY_STOP, Bool, self._cmd_estop_cb)
        self.estop_ = False

        # setting up dynamic reconfigure
        dyn.Server(MobileBaseConfig, self._reconfigure_cb)

        self.base_ctrl.then = time.time()

        estop_t = threading.Thread(target=self._estop_loop,name="Estop loop")
        estop_t.start()
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            with self._mutex:
                try:
                    self.base_ctrl.poll(not self.short_dist_imu_odom and self.imu_is_connected)
                    heartbeat_pub.publish(stmsgs.Empty())
                    using_imu_odom_pub.publish(self._use_imu_odom)
                    self.publish_status()
                    self.loop.sleep()
                except Exception as e:
                    print(e)
        estop_t.join()

    def _getYawDeg(self,msg):
        ''' IMU callback function
        Get current IMU yaw angle in degrees

        '''
        last_imu_state = self.imu_is_connected
        now = time.time()
        if now - self._imu_last_time > IMU_CONNECTION_TIMEOUT:
            self.imu_is_connected = False
        else:
            self.imu_is_connected = True
        self._imu_last_time = now

        if last_imu_state != self.imu_is_connected:
            if self.imu_is_connected:
                rospy.loginfo("IMU node is connected")
                self.logerrcode(errcode.IMU_FID, errcode.ErrCode.OK.value)
            else:
                rospy.loginfo("IMU node connection is lost")
                self._error = True
                self.logerrcode(errcode.IMU_FID, errcode.ErrCode.LO_ERR_IMU.value)
                rospy.logwarn("IMU lost, will switch to encoder odom...")

        _,_,_yaw =  tf.transformations.euler_from_quaternion([ msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self._imu_yaw = round(math.degrees(_yaw),2)

    def _init_odom_imu(self):
        ''' Return: true = IMU yaw reset to zero | false = IMU yaw failed to reset to zero
        Reset Yaw to zero, inorder to use IMU to calculate Odom

        '''
        sub = rospy.Subscriber(IMU_WIT,Imu,self._getYawDeg)
        pub = rospy.Publisher(IMU_RESET_YAW_CMD,Bool,queue_size=1)
        reset_yaw = Bool()
        reset_yaw.data = True
        rospy.loginfo("Resetting IMU for Odom_imu calculation. Current Yaw is {} deg".format(self._imu_yaw))
        pub.publish(reset_yaw)
        rospy.sleep(0.1)
        rospy.loginfo("Reset Done! Current Yaw is {} deg".format(self._imu_yaw))
        if abs(self._imu_yaw) < 0.1:
            return True
        else:
            return False

    def _reset_cb(self,msg):
        if msg.data == "RESET":
            rospy.loginfo("Resetting Chassis")
            self._error = False

    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.base_ctrl.stop()
            # rospy.sleep(1)
            self.driver.close()
            # rospy.sleep(1)
        except Exception:
            pass
        rospy.loginfo("Shutting down CompassQ2 Node...")

    def _cmd_vel_cb(self, msg):
        self.base_ctrl.set_twist(
            msg.linear.x,        # m/s
            msg.linear.y,        # m/s
            msg.angular.z,)       # rad/s

    def _reconfigure_cb(self, config, level):
        self.loginfo('receive reconfigure with level: {}'.format(level))
        # print(config)
        if level != -1:
            cur_config = self.base_ctrl.set_dynamics(config)
            self.rate = config['rate']
            self.loop = rospy.Rate(self.rate)
        else:
            cur_config = self.base_ctrl.set_dynamics(None)
        # print(cur_config)
        config['max_x_vel'] = cur_config['max_x_vel']
        config['max_y_vel'] = cur_config['max_y_vel']
        config['max_rz_vel'] = cur_config['max_rz_vel']
        config['max_x_acc'] = cur_config['max_x_acc']
        config['max_y_acc'] = cur_config['max_y_acc']
        config['max_rz_acc'] = cur_config['max_rz_acc']
        config['MAX_FILTER'] = cur_config['MAX_FILTER']
        config['rate'] = self.rate
        # print(config)
        return config

    def publish_odom(self, pose, v_cur):
        translates = (pose[0], pose[1], 0)
        quaternion = (0., 0., math.sin(pose[2] / 2.0), math.cos(pose[2] / 2.0))
        now = rospy.Time.now()
        self.tfb.sendTransform(
            translates,
            quaternion,
            now,
            self.base_frame,
            "odom"
        )

        self.last_odom.header.stamp = now
        self.last_odom.pose.pose.position = gemsgs.Point(*translates)
        self.last_odom.pose.pose.orientation = gemsgs.Quaternion(*quaternion)
        self.last_odom.twist.twist.linear.x = v_cur[0]
        self.last_odom.twist.twist.linear.y = v_cur[1]
        self.last_odom.twist.twist.angular.z = v_cur[2]
        self.last_odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.last_odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(self.last_odom)

    def publish_vol(self, voltage):
        self.voltage.data = voltage
        self.vol_pub.publish(self.voltage)
        if voltage < self.vol_warning_level:
            self.logwarn_throttle(2., 'Low voltage: {:0.3f} V!!'.format(voltage))
            self.logerrcode(errcode.CH_FID,
                            errcode.ErrCode.CH_ERR_LOW_BATTERY.value,
                            throttle_period=2.)

    def _cmd_estop_cb(self,msg):
        self.estop_ = msg.data

        # Publish new status
        self.publish_status()

    # by Max, update short distance odom flag when imu odom is used
    def _short_dist_cb(self,msg):
        self.short_dist_imu_odom = msg.data

    def _estop_loop(self):
        while not rospy.is_shutdown():
            if self.estop_:
                self.base_ctrl.stop()
            rospy.Rate(50).sleep()

    def _srv_cb(self, request):
        self.loginfo('Received request: {} {}'.format(request.command, request.parameter))
        self.loginfo('No effect in simulation.')
        return absrvs.CommandResponse(True)

    def publish_status(self):
        msg = DRIVERS_CHASSIS_STATUS.type()
        msg.stamp = rospy.Time.now()
        msg.dyn_power = True
        msg.enabled = True
        msg.imu_odom = True
        msg.side_sonar = True
        msg.pc_online = True

        msg.rc_online = True
        msg.imu_online = True
        msg.estop_off = not self.estop_
        msg.motors_online = [True for i in range(4)]
        msg.sonars_online = [True for i in range(6)]

        self.status_pub.publish(msg)

    def _srv_io_cb(self, request):
        self.loginfo('Received request:')
        self.loginfo('reset_io: {}'.format(request.reset_io))
        self.loginfo('set_io: {}'.format(request.set_io))
        self.loginfo('reset_power_io: {}'.format(request.reset_power_io))
        self.loginfo('set_power_io: {}'.format(request.set_power_io))
        self.loginfo('reset_status_io: {}'.format(request.reset_status_io))
        self.loginfo('set_status_io: {}'.format(request.set_status_io))

        return absrvs.ChassisIOControlResponse(True)

if __name__ == '__main__':
    rospy.init_node('mobile_base', log_level=rospy.INFO)
    mobile_base = MobileBaseNode()
