#!/usr/bin/python3

import blessed, sys, atexit, signal
import time

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from swivelbot_base.srv import Disinfection, DisinfectionRequest

help_msg = """
Control SwivelBot!
---------------------------
Moving around:      UVC control:
   q    w    e
   a    s    d            s
        x             z       c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s   : zero out angular velocity
o   : toogle cmd_vel enable state
z/c : start/stop UVC&door
space: (emergency) stop

CTRL-C to quit
"""


class TeleOpBase(object):

    def __init__(self, cmd_vel_topic="/cmd_vel"):
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=5)
        self.target_vel_x = 0
        self.target_vel_y = 0
        self.target_vel_rz = 0
        self.control_vel_x = 0
        self.control_vel_y = 0
        self.control_vel_rz = 0
        self.trans_step = 0.05
        self.rotat_step = 0.1
        self.acc_trans = 0.1/4
        self.acc_rotate = 0.2/4
        self.twist = Twist()
        self.blocked = False
        self.enable = True

    def cmd_vel_str(self):
        return "x_vel: %.2f  y_vel: %.2f  rz_vel: %.2f  blocked: %s  output: %s" % \
            (self.target_vel_x, self.target_vel_y, self.target_vel_rz,
             "True" if self.blocked else "False",
             "True" if self.enable else "False",)

    def update(self):
        raise NotImplementedError("Use update() in derived class")

    def get_twist(self, key):
        """
        Update status and values of speed after getting key,
        then return Twist according to status.

        Parameters
        ----------
        key: str
            Specific key value to operate car robot.
        msg: str
            Will print msg after getting 14 times update, better HMI when running
            from command line. (From history code)

        Return
        ------
        Twist
            New Twist based on status.
        """
        if key == 'w' :
            self.target_vel_x = self.target_vel_x + self.trans_step
        elif key == 'x' :
            self.target_vel_x = self.target_vel_x - self.trans_step
        elif key == 'a' :
            self.target_vel_y = self.target_vel_y + self.trans_step
        elif key == 'd' :
            self.target_vel_y = self.target_vel_y - self.trans_step
        elif key == 'q' :
            self.target_vel_rz = self.target_vel_rz + self.rotat_step
        elif key == 'e' :
            self.target_vel_rz = self.target_vel_rz - self.rotat_step
        elif key == 't' :
            self.trans_step = self.trans_step + 0.01
            print(f"self.trans_step is {self.trans_step:.3f}")
        elif key == 'g' :
            self.trans_step = self.trans_step - 0.01
            print(f"self.trans_step is {self.trans_step:.3f}")
        elif key == 'y' :
            self.rotat_step = self.rotat_step + 0.005
            print(f"self.rotat_step is {self.rotat_step:.3f}")
        elif key == 'h' :
            self.rotat_step = self.rotat_step - 0.005
            print(f"self.rotat_step is {self.rotat_step:.3f}")
        elif key == 'r' :
            self.acc_trans = self.acc_trans * 2.0
            self.acc_rotate = self.acc_rotate * 2.0
            print(f"self.acc_trans is: {self.acc_trans:.4f} self.acc_rotate is: {self.acc_rotate:.4f}")
        elif key == 'f' :
            self.acc_trans = self.acc_trans * 0.5
            self.acc_rotate = self.acc_rotate * 0.5
            print(f"self.acc_trans is: {self.acc_trans:.4f} self.acc_rotate is: {self.acc_rotate:.4f}")
        elif key == 's' :
            self.target_vel_rz = 0
            self.control_vel_rz = 0
        elif key == ' ':
            self.enable = True
            self.target_vel_x = 0
            self.target_vel_y = 0
            self.target_vel_rz = 0
            self.control_vel_x = 0
            self.control_vel_y = 0
            self.control_vel_rz = 0
        elif key == 'o':
            if self.enable:
                self.stop()
                self.enable = False
            else:
                self.enable = True
        else:
            key = 'NONE'

        sign_x = 1 if self.target_vel_x >= 0 else -1
        sign_y = 1 if self.target_vel_y >= 0 else -1
        sign_rz = 1 if self.target_vel_rz >= 0 else -1
        target_vel_x_abs = abs(self.target_vel_x)
        target_vel_y_abs = abs(self.target_vel_y)
        target_vel_rz_abs = abs(self.target_vel_rz)

        if target_vel_x_abs > self.control_vel_x:
            self.control_vel_x = min(
                target_vel_x_abs, self.control_vel_x + self.acc_trans)
        else:
            self.control_vel_x = target_vel_x_abs

        if target_vel_y_abs > self.control_vel_y:
            self.control_vel_y = min(
                target_vel_y_abs, self.control_vel_y + self.acc_trans)
        else:
            self.control_vel_y = target_vel_y_abs

        if target_vel_rz_abs > self.control_vel_rz:
            self.control_vel_rz = min(
                target_vel_rz_abs, self.control_vel_rz + self.acc_rotate)
        else:
            self.control_vel_rz = target_vel_rz_abs

        self.twist = Twist()
        self.twist.linear.x = self.control_vel_x * sign_x
        self.twist.linear.y = self.control_vel_y * sign_y
        self.twist.angular.z = self.control_vel_rz * sign_rz

        if self.blocked:
            self.twist.linear.x = 0 if self.twist.linear.x>0 else self.twist.linear.x

        return self.twist

    def stop(self):
        self.target_vel_x = 0
        self.target_vel_y = 0
        self.target_vel_rz = 0
        self.control_vel_x = 0
        self.control_vel_y = 0
        self.control_vel_rz = 0
        self.twist = Twist()
        self.pub.publish(self.twist)

    def set_blocked(self, lidar_blocked):
        if lidar_blocked and not self.blocked:
            self.target_vel_x = 0
            self.target_vel_y = 0
            self.target_vel_rz = 0
            self.control_vel_x = 0
            self.control_vel_y = 0
            self.control_vel_rz = 0
            self.twist = Twist()
            self.pub.publish(self.twist)
        self.blocked = lidar_blocked

class TeleOpRepeated(TeleOpBase):

    def update(self, key):
        twist = self.get_twist(key)
        if self.enable:
            self.pub.publish(twist)

class UVCControlMode:
    ON      = {'door': 2, 'fan': 2, 'uvc': 3, 'magnent': 0, 'timer': 0, 'chassis_fan': 2, 'chassis_uvc': 2}
    OFF     = {'door': 1, 'fan': 1, 'uvc': 1, 'magnent': 0, 'timer': 2, 'chassis_fan': 1, 'chassis_uvc': 1}
    PREPARE = {'door': 1, 'fan': 2, 'uvc': 3, 'magnent': 0, 'timer': 0, 'chassis_fan': 2, 'chassis_uvc': 1}
    PAUSE   = {'door': 1, 'fan': 1, 'uvc': 1, 'magnent': 0, 'timer': 0, 'chassis_fan': 1, 'chassis_uvc': 1}
    ESTOP   = {'door': 1, 'fan': 1, 'uvc': 1, 'magnent': 1, 'timer': 0, 'chassis_fan': 1, 'chassis_uvc': 1}

class TeleOpUVC:

    def __init__(self, srv_name="/cmd_disinfection"):
        rospy.wait_for_service(srv_name)
        self._fn_disinfect = rospy.ServiceProxy(srv_name, Disinfection)

    def update(self, key):
        if key == 'z':
            self._c_disinfect(**UVCControlMode.PREPARE)
            time.sleep(1.0)
            self._c_disinfect(**UVCControlMode.ON)
        if key == 'c':
            self._c_disinfect(**UVCControlMode.OFF)
        if key == ' ':
            self._c_disinfect(**UVCControlMode.ESTOP)

    def stop(self):
        self._c_disinfect(**UVCControlMode.ESTOP)

    # Control the hardware
    def _c_disinfect(self, door=0, fan=0, uvc=0, magnent=0, timer=0, chassis_fan=0, chassis_uvc=0):
        try:
            rospy.logdebug('c_disinfect with: d:{}, f:{}, u:{}, m:{}, t:{}, cf:{}, cu:{}'.format(
                door, fan, uvc, magnent, timer, chassis_fan, chassis_uvc
            ))
            res = self._fn_disinfect(DisinfectionRequest(
                door, fan, uvc, magnent, timer, chassis_fan, chassis_uvc))
            rospy.logdebug('c_disinfect ret:{}'.format(res.received))
            return res.received
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
            return False

class GimbalControl:
    def __init__(self, cmd_gimbal_topic="/cmd_gimbal"):
        self.pub = rospy.Publisher(cmd_gimbal_topic, Float32, queue_size=5)

    def goto_zero_pose(self):
        time.sleep(1)   # wait for connection
        self.pub.publish(Float32(0.0))


class LidarControl:

    def __init__(self, scan_topic="/scan", threshold=0.3, blocked_cb=None):
        self.threshold = threshold
        self.blocked_cb = blocked_cb
        self.sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_cb)

    def scan_cb(self, msg):
        pt_length = len(msg.ranges)
        front45deg = msg.ranges[int(pt_length/8):int(pt_length*3/8)]
        ranging_results = [ r<self.threshold and r!=0 for r in front45deg ]
        if self.blocked_cb is not None:
                self.blocked_cb(True in ranging_results)


def echo(text):
    sys.stdout.write(u'{}'.format(text))
    sys.stdout.flush()

def term_clear():
    term = blessed.Terminal()
    print(f'{term.normal}{term.clear}Cleanup Done!')

def sigint_handler(signum, frame):
    print('Ctrl+C Detected! Start cleaning up.', file=sys.stderr)
    sys.exit(0)

if __name__=="__main__":

    signal.signal(signal.SIGINT, sigint_handler)
    atexit.register(term_clear)
    rospy.init_node("teleop_node")

    cmd_vel_topic_name = "/cmd_vel_mux/input/keyop" if rospy.get_param('~use_mux', False) else "/cmd_vel"
    vel = TeleOpRepeated(cmd_vel_topic=cmd_vel_topic_name)
    rospy.on_shutdown(vel.stop)
    uvc = TeleOpUVC()
    rospy.on_shutdown(uvc.stop)

    gimbal = GimbalControl()
    gimbal.goto_zero_pose()

    lidar = LidarControl(blocked_cb=vel.set_blocked)
    #rospy.spin()

    term = blessed.Terminal()
    print(f"{term.green_reverse_blink}ALL SYSTEMS GO{term.normal}")
    print(f"{term.home}{term.normal}{term.clear}")
    print(f"{term.black_on_white}teleop_raw.py{term.clear_eol}")
    print(f"{term.normal}{term.clear_eol}")
    print(f"{help_msg}{term.move_down}")

    with term.cbreak():
        while not rospy.is_shutdown():
            with term.location():
                inp = term.inkey(timeout=0.5)
                vel.update(inp.lower())
                uvc.update(inp.lower())
                print(f"{term.black_on_darkkhaki}{vel.cmd_vel_str()}"
                        f"{term.clear_eol}{term.normal}")
