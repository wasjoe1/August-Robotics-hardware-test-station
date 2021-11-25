#!/usr/bin/python3

import sys, atexit, signal
import logging, time
from datetime import datetime
from math import copysign

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from swivelbot_base.msg import BatteryInfo, ChassisStatus, GimbalInfo
from swivelbot_base.srv import PowerCtrl, PowerCtrlRequest
from swivelbot_msgs.msg import RobotCommand, RobotError
from swivelbot_msgs.srv import RequestCommand, RequestCommandRequest
from swivelbot_common.errhandle_ros.error_code import ErrorCode

from blessed import Terminal

def echo(text):
    sys.stdout.write(u'{}'.format(text))
    sys.stdout.flush()

def term_clear():
    term = Terminal()
    # print(f'{term.normal}{term.clear}Cleanup Done!')

def sigint_handler(signum, frame):
    print('Ctrl+C Detected! Start cleaning up.', file=sys.stderr)
    sys.exit(0)


class RobotStatusMsg:
    def __init__(self):
        self.battery = None
        self.chassis = None
        self.gimbal = None
        self.cmd_vel = None
        self.errorcode = None
        self.fsm_state = None

msg_store = RobotStatusMsg()

def battery_cb(msg):
    global msg_store
    msg_store.battery = msg

def chassis_cb(msg):
    global msg_store
    msg_store.chassis = msg

def gimbal_cb(msg):
    global msg_store
    msg_store.gimbal = msg
    relay_rc_press(msg)

def twist_cb(msg):
    global msg_store
    msg_store.cmd_vel = msg

def errorcode_cb(msg):
    global msg_store
    msg_store.errorcode = msg

def fsm_state_cb(msg):
    global msg_store
    msg_store.fsm_state = msg

def relay_rc_press(msg):
    if msg.RC_stop:
        try:
            rospy.wait_for_service("/cmd_power_control", timeout=0.1)
            power_ctrl = rospy.ServiceProxy("/cmd_power_control", PowerCtrl)
            return power_ctrl(int(0))
        except (rospy.ROSException, rospy.ServiceException) as e:
            return False

def toggle_power_mode():
    if msg_store is not None and msg_store.chassis is not None:
        rospy.wait_for_service("/cmd_power_control", timeout=0.1)
        try:
            rospy.wait_for_service("/cmd_power_control", timeout=0.1)
            power_ctrl = rospy.ServiceProxy("/cmd_power_control", PowerCtrl)
            return power_ctrl(not bool(msg_store.chassis.Power_on))
        except (rospy.ROSException, rospy.ServiceException) as e:
            return False

def gimbal_zero_yaw(pub):
    pub.publish(Float32(0.0))

def localizer_reset(pub):
    msg = RobotCommand()
    msg.command = "RESET_ERRORCODE"
    msg.parameter = "5601"
    pub.publish(msg)

def reset_as_gui():
    try:
        rospy.wait_for_service("/gui_cmd", timeout=0.1)
        gui_cmd = rospy.ServiceProxy("/gui_cmd", RequestCommand)
        return gui_cmd('reset','')
    except (rospy.ROSException, rospy.ServiceException) as e:
        return False

def display_help(duration=2):

    term = Terminal() # term.height, term.width
    echo(term.home()+term.clear())
    echo(f"{term.black_on_white}Diego Onboard Status{term.clear_eol}\n")
    echo(f"{term.normal}{term.move_down(4)}")

    echo(f"{term.black_on_darkkhaki}")
    echo(f"Currently the following shortcut keys are avalible{term.clear_eol}\n{term.clear_eol}\n")
    echo(f"h: Disaply this help string{term.clear_eol}\n")
    echo(f"p: Toggle chassis power{term.clear_eol}\n")
    echo(f"y: Zero out gimbal angle (P3){term.clear_eol}\n")
    echo(f"l: Reset localization error{term.clear_eol}\n")
    echo(f"r: Reset all error pretending GUI click{term.clear_eol}\n")
    echo(f"z: Pseudo-pirouetting, +ve direction{term.clear_eol}\n")
    echo(f"x: Pseudo-pirouetting, -ve direction{term.clear_eol}\n")
    echo(f"q: Quit this dashboard{term.clear_eol}\n{term.clear_eol}\n")

    time.sleep(duration)
    echo(f"{term.normal}{term.clear}")

def pirouette(pub, rz_vel=0.3, dir=1, duration=0.5):
    msg = Twist()
    msg.angular.z = copysign(abs(rz_vel),dir)
    pub.publish(msg)
    time.sleep(duration)
    pub.publish(Twist())

refresh_interval = 0.5
def refresh_screen():

    term = Terminal() # term.height, term.width
    echo(term.home()+term.clear())
    echo(f"{term.black_on_white}Diego Onboard Status{term.clear_eol}\n")
    echo(f"{term.normal}{term.move_down()}")
    dt_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    echo(f"Current time: {dt_string}, refreshing every {refresh_interval:.1f}s, press 'h' for help\n")
    echo(term.normal+term.move_down())

    global msg_store
    if msg_store is None:
        echo(f"{term.bold}{term.underline}Waiting for ROS Messages\n")
        return

    row, col = term.get_location()
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}Battery\n")
        msg = msg_store.battery
        if msg is None:
            echo(f"{term.normal}Waiting for battery info")
        else:
            echo(f"{term.normal}Batt level: {msg.SOC:.1f}, Temp: {msg.temperature:.1f}\n")
            if msg.charger_on and msg.state==2:
                echo(f"{term.normal}Votage: {msg.charger_voltage:.1f}V, Current: {msg.charger_current:.1f}A\n")
                echo(f"{term.normal}Power: +{msg.voltage*msg.current:.1f}W\n")
            else:
                echo(f"{term.normal}Votage: {msg.voltage:.1f}V, Current: {msg.current:.1f}A\n")
                echo(f"{term.normal}Power: -{msg.voltage*msg.current:.1f}W\n")

            if msg.error or msg.charger_error:
                echo(f"{term.normal}Status: {term.firebrick2_reverse_blink}ERROR")
                echo(f"{term.normal}{term.clear_eol}")
            elif msg.SOC<10:
                echo(f"{term.normal}Status: {term.darkorange1_reverse}BATTERY_LOW")
                echo(f"{term.normal}{term.clear_eol}")
            else:
                echo(f"{term.normal}Status: {term.green_reverse}OK")
                echo(f"{term.normal}{term.clear_eol}")

    row += 6
    cmd_vel_str = "None"
    if msg_store.cmd_vel is not None:
        cmd_vel_str = f"{msg_store.cmd_vel.linear.x:.2f}, {msg_store.cmd_vel.linear.y:.2f}, {msg_store.cmd_vel.angular.z:.2f}"
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}Chassis\n")
        msg = msg_store.chassis
        if msg is None:
            echo(f"{term.normal}Waiting for chassis info")
        else:
            if msg.Power_on:
                colored_pow_str = f"{term.green_reverse}ON"
            else:
                colored_pow_str = f"{term.firebrick2_reverse}OFF"

            echo(f"{term.normal}Power: {colored_pow_str}\n")
            echo(f"{term.normal}Fan: {msg.Fan_state}, UVC: {msg.UVC_state}\n")
            echo(f"{term.normal}cmd_vel: {cmd_vel_str}\n")
            echo(f"{term.normal}\nStatus:\n")
            if True in [bool(msg.Motor_offline), bool(msg.Motor_offline), bool(msg.Motor_offline)]:
                if msg.Motor_offline:
                    echo(f"{term.firebrick2_reverse}Motor offline\n")
                if msg.IMU_offline:
                    echo(f"{term.firebrick2_reverse}IMU offline\n")
                if msg.Battery_offline:
                    echo(f"{term.firebrick2_reverse}Battery offline\n")
            else:
                echo(f"{term.nomral}No error\n")

    row += 8
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(term.darkkhaki("─"*int(term.width/1.2)))
        echo(f"{term.clear_eol}\n{term.normal}")
        echo(f"{term.bold}{term.underline}Error Code / FSM state{term.normal}\n")
        msg = msg_store.errorcode
        if msg is None:
            echo(f"{term.normal}Waiting for error status\n")
        else:
            echo(f"{term.normal}Code: ")
            if not msg.error_codes:
                echo(f"{term.green_reverse}ALL GREEN")
            else:
                for e in msg.error_codes:
                    echo(f"{term.firebrick2_reverse}{ErrorCode(e).name}")
                    echo(f"{term.normal}, ")
            echo(f"{term.normal}\n")

        msg = msg_store.fsm_state
        if msg is None:
            echo(f"{term.normal}Waiting for fsm state\n")
        else:
            echo(f"{term.normal}FSM: ")
            for s in msg.data.split("."):
                if s == "ERROR":
                    echo(f"{term.firebrick2_reverse}{s}")
                    echo(f"{term.normal}.")
                else:
                    echo(f"{term.normal}{s}")
                    echo(f"{term.normal}.")

    row, col = term.get_location()
    col += 40
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}Gimbal\n")
        tab_move = f"{term.normal}{term.move_x(col)}"
        msg = msg_store.gimbal
        if msg is None:
            echo(f"{tab_move}Waiting for gimbal info")
        else:
            if msg.Safety_Shutdown:
                colored_saf_str = f"{term.firebrick2_reverse}SAF_SHUTDOWN"
            else:
                colored_saf_str = f"{term.green_reverse}NORMAL"

            if not msg.Estop:
                colored_estop_str = f"{term.firebrick2_reverse}PRESSED"
            else:
                colored_estop_str = f"{term.green_reverse}RELESED"

            if msg.RC_stop:
                colored_rc_str = f"{term.firebrick2_reverse}PRESSED"
            else:
                colored_rc_str = f"{term.green_reverse}RELESED"

            echo(f"{tab_move}SAF:\t{colored_saf_str}\n")
            echo(f"{tab_move}ESTOP:\t{colored_estop_str}\n")
            echo(f"{tab_move}RC_SW:\t{colored_rc_str}\n")
            echo(f"{tab_move}Yaw angle: {msg.yaw_angle:.2f} rad, Yaw rate: {msg.yaw_rate:.2f}\n")
            echo(f"{tab_move}UVC: {msg.UVC:d}, Fan: {msg.Fan:d}, Door: {msg.Door:d}, Magnet: {msg.Magnet:d}\n")
            echo(f"{tab_move}Temp: {msg.temp[0]:.1f}, {msg.temp[1]:.1f}, {msg.temp[2]:.1f}, {msg.temp[3]:.1f}\n")

            echo(f"{tab_move}\n{tab_move}Status:\n")
            if True in [bool(msg.IR_offline), bool(msg.Stepper_offline), bool(msg.Encoder_offline), bool(msg.Thermometer_offline)]:
                if msg.IR_offline:
                    echo(f"{tab_move}{term.firebrick2_reverse}IR offline\n")
                if msg.Stepper_offline:
                    echo(f"{tab_move}{term.firebrick2_reverse}Stepper offline\n")
                if msg.Encoder_offline:
                    echo(f"{tab_move}{term.firebrick2_reverse}Encoder offline\n")
                if msg.Thermometer_offline:
                    echo(f"{tab_move}{term.firebrick2_reverse}Thermoneter offline\n")
            else:
                echo(f"{tab_move}No error\n")




if __name__=="__main__":

    signal.signal(signal.SIGINT, sigint_handler)
    atexit.register(term_clear)
    rospy.init_node("dashboad", anonymous=True)
    # rospy.on_shutdown()

    # swivelbot_base -> base_node
    rospy.Subscriber("/battery", BatteryInfo, battery_cb)
    rospy.Subscriber("/chassis_status", ChassisStatus, chassis_cb)
    rospy.Subscriber("/gimbal_info", GimbalInfo, gimbal_cb)
    rospy.Subscriber("/cmd_vel", Twist, twist_cb)
    gimbal_pub = rospy.Publisher("/cmd_gimbal", Float32, queue_size=1)
    vel_pub = rospy.Publisher("/cmd_vel_mux/input/keyop", Twist, queue_size=1)

    # swivelbot_main -> FSM, backoffice
    rospy.Subscriber("/robot_error", RobotError, errorcode_cb)
    rospy.Subscriber("/robot_state", String, fsm_state_cb)
    robocom_pub = rospy.Publisher("/robot_cmd", RobotCommand, queue_size=1)
    # TOPIC_ROBOT_INFO = '/robot_info'        # Type: swivelbot_msgs/RobotInfo
    # TOPIC_ROBOT_POSE = '/robot_pose'        # Type: nav_msgs/Odometry


    term = Terminal()
    print(f"{term.green_reverse_blink}ALL SYSTEMS GO{term.normal}")
    print(f"{term.home}{term.normal}{term.clear}")


    with term.fullscreen(), term.cbreak():
        refresh_screen()
        while not rospy.is_shutdown():
            inp = term.inkey(timeout=refresh_interval)
            inp = inp.lower()
            if not inp:
                pass # no keypress, timeout
            elif inp == 'q':
                break # quit
            elif inp == 'p':
                # toggle chassis power mode
                toggle_power_mode()
            elif inp == 'y':
                # set swivelbot (P3) gimbal yaw to zero
                gimbal_zero_yaw(gimbal_pub)
            elif inp == 'l':
                # reset localization error
                localizer_reset(robocom_pub)
            elif inp == 'r':
                # pretend to be gui & attempt resetting all errors
                reset_as_gui()
            elif inp == 'h':
                # display help string for a few seconds
                display_help(duration=5)
            elif inp == 'z':
                # do rotation(pirouetting), +ve direction
                pirouette(vel_pub, dir=+1)
            elif inp == 'x':
                # do rotation(pirouetting), -ve direction
                pirouette(vel_pub, dir=-1)


            refresh_screen()
