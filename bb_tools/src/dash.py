#!/usr/bin/python3

import sys, atexit, signal
import logging, time
from datetime import datetime

import rospy
from std_msgs.msg import UInt16, Int32, Float32, String, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from boothbot_msgs.msg import ChassisOffline, ChassisPower, ChassisStats,\
                                ChassisWheelRPM, ErrCode

from blessed import Terminal
from rostopic_helpers import BufferedTopicSubscriber
from rostopic_helpers import rpy_from_pose, rpy_from_quaternion
from errorcode_helper import BufferedErrorCodeSubscriber

def echo(text):
    sys.stdout.write(u'{}'.format(text))
    sys.stdout.flush()

def term_clear():
    term = Terminal()
    print(f'{term.normal}{term.clear}Cleanup Done!')

def sigint_handler(signum, frame):
    print('Ctrl+C Detected! Start cleaning up.', file=sys.stderr)
    sys.exit(0)

def display_help(duration=2):

    term = Terminal() # term.height, term.width
    echo(term.home()+term.clear())
    echo(f"{term.black_on_white}Lionel Debug TUI{term.clear_eol}\n")
    echo(f"{term.normal}{term.move_down(4)}")

    echo(f"{term.black_on_darkkhaki}")
    echo(f"Currently the following shortcut keys are avalible{term.clear_eol}\n")
    echo(f"{term.clear_eol}\n")
    echo(f"h: Disaply this help string{term.clear_eol}\n")
    echo(f"q: Quit this dashboard{term.clear_eol}\n")
    echo(f"{term.clear_eol}\n")

    time.sleep(duration)
    echo(f"{term.normal}{term.clear}")

cached_store = {}
refresh_interval = 1.0
def refresh_screen():

    term = Terminal() # term.height, term.width
    dt_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    echo(term.home()+term.clear())
    echo(f"{term.black_on_white}Lionel Debug TUI{term.clear_eol}\n{term.normal}")
    echo(f"Current time: {dt_string}, refreshing every {refresh_interval:.1f}s, "
            f"press 'h' for help\n")
    echo(term.normal+term.move_down())

    global cached_store

    row, col = term.get_location()

    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}Localization & Navigation\n")

        indent = 20
        echo(term.move_x(indent))
        echo(f"{term.normal}{str('x(m)'):>10}, {str('y(m)'):>10}, {str('rz(rad)'):>10}\n")

        for store in [cached_store.get("odom"), cached_store.get("odom_enc")]:
            msg, freshness = store.get_latest_msg(expire_interval=10)
            if msg is None:
                echo(f"{term.normal}Waiting for {store.info[0]}...\n")
            else:
                echo(f"{term.normal}{store.info[0]}")
                if freshness is False:
                    echo(f"{term.normal} {term.firebrick2_reverse}(E)")
                echo(f"{term.normal}:\t\t")

                pose = msg.pose.pose
                x_field = f"{pose.position.x:+4.3f}"
                y_field = f"{pose.position.x:+4.3f}"
                z_field = f"{rpy_from_pose(pose)[2]:+4.3f}"
                
                echo(term.move_x(indent))
                echo(f"{term.normal}{x_field:>10}, {y_field:>10}, {z_field:>10}\n")
                echo(f"{term.normal}{term.clear_eol}")

        echo(term.move_x(indent-len("(rad)")))
        echo(f"{term.normal}(rad){str('pitch'):>10}, {str('roll'):>10}, {str('yaw'):>10}\n")

        for store in [cached_store.get("imu"), cached_store.get("incli")]:
            msg, freshness = store.get_latest_msg(expire_interval=10)
            if msg is None:
                echo(f"{term.normal}Waiting for {store.info[0]}...\n")
            else:
                echo(f"{term.normal}{store.info[0]}")
                if freshness is False:
                    echo(f"{term.normal} {term.firebrick2_reverse}(E)")
                echo(f"{term.normal}:\t\t")

                pitch, roll, yaw = rpy_from_quaternion(msg.orientation)
                x_field = f"{pitch:+2.5f}"
                y_field = f"{roll:+2.5f}"
                z_field = f"{yaw:+2.5f}"
                echo(term.move_x(indent))
                echo(f"{term.normal}{x_field:>10}, {y_field:>10}, {z_field:>10}\n")
                echo(f"{term.normal}{term.clear_eol}")

    row += 8
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}Hardware\n")

        indent = 35

        store = cached_store.get("stats")
        msg, freshness = store.get_latest_msg(expire_interval=None)
        if msg is None:
            echo(f"{term.normal}Waiting for {store.info[0]}...\n")
        else:
            echo(f"{term.normal}{store.info[0]}")
            if freshness is False:
                echo(f"{term.normal} {term.firebrick2_reverse}(E)")
            echo(f"{term.normal}:")

            echo(term.move_x(indent))
            echo(f"{term.normal}{msg.firmware_version.data.lstrip('Firmware Version:')}, ")
            echo(f"{term.normal}total_mileage:{msg.total_mileage.data}")
            
            echo(f"{term.normal}{term.clear_eol}\n")

        store = cached_store.get("battery")
        msg, freshness = store.get_latest_msg(expire_interval=None)
        if msg is None:
            echo(f"{term.normal}Waiting for {store.info[0]}...\n")
        else:
            echo(f"{term.normal}{store.info[0]}")
            if freshness is False:
                echo(f"{term.normal} {term.firebrick2_reverse}(E)")
            echo(f"{term.normal}:")

            echo(term.move_x(indent))
            echo(f"{term.normal}{msg.battery_voltage_V:.1f}V, {msg.battery_current_A:.1f}A, ")
            echo(f"{term.normal}{-msg.battery_power_W:+.1f}W")
            
            echo(f"{term.normal}{term.clear_eol}\n")

        store = cached_store.get("motors")
        msg, freshness = store.get_latest_msg(expire_interval=None)
        if msg is None:
            echo(f"{term.normal}Waiting for {store.info[0]}...\n")
        else:
            echo(f"{term.normal}{store.info[0]}")
            if freshness is False:
                echo(f"{term.normal} {term.firebrick2_reverse}(E)")
            echo(f"{term.normal}:")

            echo(term.move_x(indent))
            rpm_str = f"{msg.front_left_wheel_rpm:.1f}"
            echo(f"fl: {rpm_str:>7}, ")
            rpm_str = f"{msg.front_right_wheel_rpm:.1f}"
            echo(f"fr: {rpm_str:>7}\n")
            echo(term.move_x(indent))
            rpm_str = f"{msg.rear_left_wheel_rpm:.1f}"
            echo(f"rl: {rpm_str:>7}, ")
            rpm_str = f"{msg.rear_right_wheel_rpm:.1f}"
            echo(f"rr: {rpm_str:>7}")
            echo(f"{term.normal}\n")

        store = cached_store.get("offline")
        msg, freshness = store.get_latest_msg(expire_interval=None)
        if msg is None:
            echo(f"{term.normal}Waiting for {store.info[0]}...\n")
        else:
            echo(f"{term.normal}{store.info[0]}")
            if freshness is False:
                echo(f"{term.normal} {term.firebrick2_reverse}(E)")
            echo(f"{term.normal}:")

            echo(term.move_x(indent))

            color_mod = f"{term.green_reverse}" if msg.RC_online else f"{term.firebrick2_reverse}"
            echo(f"{color_mod}RC{term.normal}  ")
            color_mod = f"{term.green_reverse}" if msg.IMU_online else f"{term.firebrick2_reverse}"
            echo(f"{color_mod}IMU{term.normal}  ")
            color_mod = f"{term.green_reverse}" if msg.SONAR_online else f"{term.firebrick2_reverse}"
            echo(f"{color_mod}SONAR{term.normal}  ")
            color_mod = f"{term.green_reverse}" if msg.PC_online else f"{term.firebrick2_reverse}"
            echo(f"{color_mod}PC{term.normal}  ")
            for i, m in enumerate(msg.MOTOR_online):
                color_mod = f"{term.green_reverse}" if m else f"{term.firebrick2_reverse}"
                echo(f"{color_mod}m_{i+1:d}{term.normal} ")
            
            echo(f"{term.normal}{term.clear_eol}\n")

        for store in [
            cached_store.get("cb0_camera"),
            cached_store.get("cb0_laser"),
            cached_store.get("cb0_servo"),
        ]:
            msg, freshness = store.get_latest_msg(expire_interval=None)
            if msg is None:
                echo(f"{term.normal}Waiting for {store.info[0]}...\n")
            else:
                echo(f"{term.normal}{store.info[0]}")
                if freshness is False:
                    echo(f"{term.normal} {term.firebrick2_reverse}(E)")
                echo(f"{term.normal}:")

                echo(term.move_x(indent))
                echo(f"{term.normal}{msg.data.upper()}")
                echo(f"{term.normal}{term.clear_eol}\n")


    row += 10
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}Status Report\n")

        indent = 40

        for store in [
            cached_store.get("bbc_cmd"),
            cached_store.get("gui_bbc_state"),
            cached_store.get("gui_nav_state"),
            cached_store.get("mark_cmd"),
            cached_store.get("gui_mark_state"),
            cached_store.get("gbm_cmd"),
            cached_store.get("gbm_state"),
            cached_store.get("gui_cb0_cmd"),
            cached_store.get("gui_cb0_state"),
        ]:
            msg, freshness = store.get_latest_msg(expire_interval=None)
            if msg is None:
                echo(f"{term.normal}Waiting for {store.info[0]}...\n")
            else:
                echo(f"{term.normal}{store.info[0]}")
                if freshness is False:
                    echo(f"{term.normal} {term.firebrick2_reverse}(E)")
                echo(f"{term.normal}:")

                echo(term.move_x(indent))
                echo(f"{term.normal}{msg.data}")
                echo(f"{term.normal}{term.clear_eol}\n")
    
    row += 11
    with term.location():
        echo(term.move_x(col)+term.move_y(row))
        echo(f"{term.bold}{term.underline}ErrorCode\n{term.normal}")

        store = cached_store.get("errorcode")
        ec_list = store.get_readable_ec()
        if ec_list:
            for ec in ec_list:
                echo(f"{term.firebrick2_reverse}{ec}{term.normal}, ")
        else:
             echo(f"{term.green_reverse}ALL GREEN{term.normal}")


if __name__=="__main__":

    # atexit.register(term_clear)
    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node("dashboard", anonymous=True)

    cached_store["battery"] = BufferedTopicSubscriber("/chassis/battery_info", ChassisPower)
    cached_store["offline"] = BufferedTopicSubscriber("/chassis/offline_status", ChassisOffline)
    cached_store["stats"]   = BufferedTopicSubscriber("/chassis/statistics", ChassisStats)
    cached_store["motors"]  = BufferedTopicSubscriber("/chassis/wheel_rpm", ChassisWheelRPM)

    cached_store["cmd_vel"]  = BufferedTopicSubscriber("/cmd_vel", Twist)
    cached_store["odom"]     = BufferedTopicSubscriber("/odom", Odometry)
    cached_store["odom_enc"] = BufferedTopicSubscriber("/odom_encoder", Odometry)
    cached_store["imu"]      = BufferedTopicSubscriber("/imu_wit", Imu)
    cached_store["incli"]    = BufferedTopicSubscriber("/inclinometer", Imu)

    cached_store["mb0_led"]   = BufferedTopicSubscriber("/mb_0/led_feedback", Bool)
    cached_store["mb0_volt"]  = BufferedTopicSubscriber("/mb_0/voltage", Float32)
    
    cached_store["bbc_cmd"]    = BufferedTopicSubscriber("/boothbot_controller/cmdword", String)
    cached_store["mark_cmd"]   = BufferedTopicSubscriber("/marking/cmdword", String)
    cached_store["mark_state"] = BufferedTopicSubscriber("/marking/state", Int32)
    cached_store["gbm_cmd"]    = BufferedTopicSubscriber("/gbm/cmdword", String)
    cached_store["gbm_state"]  = BufferedTopicSubscriber("/gbm/state", String)
    cached_store["cb0_cmd"]    = BufferedTopicSubscriber("/cb_0/cmdword", String)
    cached_store["cb0_state"]  = BufferedTopicSubscriber("/cb_0/state", String)
    cached_store["nav_state"]  = BufferedTopicSubscriber("/nav/state", String)

    cached_store["cb0_camera"]  = BufferedTopicSubscriber("/cb_0/hardware_status/cameras", String)
    cached_store["cb0_laser"]   = BufferedTopicSubscriber("/cb_0/hardware_status/laser", String)
    cached_store["cb0_servo"]   = BufferedTopicSubscriber("/cb_0/hardware_status/servos", String)
    # cached_store["cb0_incli"]   = BufferedTopicSubscriber("/cb_0/hardware_status/inclination", Float64MultiArray)
    # cached_store["cb0_srad"]    = BufferedTopicSubscriber("/cb_0/hardware_status/search_rad", Float32)
    # cached_store["cb0_joint"]   = BufferedTopicSubscriber("/cb_0/hardware_status/joint", JointState)

    cached_store["sonar_conf"]      = BufferedTopicSubscriber("/sonar_rear_on", Bool)
    cached_store["imu_odom_conf"]   = BufferedTopicSubscriber("/using_imu_odom", Bool)

    cached_store["gui_bbc_state"]   = BufferedTopicSubscriber("/gui/boothbot_controller/state", String)
    cached_store["gui_cb0_cmd"]     = BufferedTopicSubscriber("/gui/cb_0/cmdword_fb", String)
    cached_store["gui_cb0_state"]   = BufferedTopicSubscriber("/gui/cb_0/state", String)
    cached_store["gui_mark_state"]  = BufferedTopicSubscriber("/gui/marking/state", String)
    cached_store["gui_nav_state"]   = BufferedTopicSubscriber("/gui/nav/state", String)

    cached_store["io_in"]   = BufferedTopicSubscriber("/io_driver/inputs", UInt16)
    cached_store["io_out"]  = BufferedTopicSubscriber("/io_driver/ouputs", UInt16)
    cached_store["io_set"]  = BufferedTopicSubscriber("/io_driver/set_do", UInt16)

    cached_store["errorcode"] = BufferedErrorCodeSubscriber()

    term = Terminal()
    print(f"{term.green_reverse_blink}ALL SYSTEMS GO{term.normal}")
    print(f"{term.home}{term.normal}{term.clear}")


    with term.fullscreen(), term.hidden_cursor(), term.cbreak():
        refresh_screen()
        while not rospy.is_shutdown():
            inp = term.inkey(timeout=refresh_interval)
            inp = inp.lower()
            if not inp:
                pass # no keypress, timeout
            elif inp == 'q':
                break # quit
            elif inp == 'h':
                # display help string for a few seconds
                display_help(duration=5)

            refresh_screen()
