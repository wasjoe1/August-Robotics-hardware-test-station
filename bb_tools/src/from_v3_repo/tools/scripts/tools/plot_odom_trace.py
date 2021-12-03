#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

import rospy
# odom msg
from nav_msgs.msg import Odometry

# robot_pose_ekf msg
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf
import math

Version = "Ver 1.0, last update Sep 10, 2019"

# CSV file trace plot
LOG_LINE = None

ODOM_X = list()
ODOM_Y = list()
ODOM_LINE = None
HEADING_U = list()
HEADING_V = list()
ODOM_current_distance = 0
ODOM_current_yaw = 0

ODOM_1_X = list()
ODOM_1_Y = list()
ODOM_1_LINE = None
HEADING_1_U = list()
HEADING_1_V = list()
ODOM_1_current_distance = 0
ODOM_1_current_yaw = 0

EKF_X = list()
EKF_Y = list()
HEADING_EKF_U = list()
HEADING_EKF_V = list()
EKF_LINE = None
EKF_current_distance = 0
EKF_current_yaw = 0

def getYaw(odom_q):
    euler_angle =  tf.transformations.euler_from_quaternion([odom_q.x,odom_q.y,odom_q.z,odom_q.w])

    return euler_angle[-1] #yaw

def odom_callback(odom_data):
    global ODOM_X
    global ODOM_Y
    global HEADING_U
    global HEADING_V
    global ODOM_current_distance 
    global ODOM_current_yaw
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = getYaw(odom_data.pose.pose.orientation)
    u = math.cos(yaw)
    v = math.sin(yaw)
    
    if (odom_callback.initialize_flag == 0):    
        odom_callback.last_x = x
        odom_callback.last_y = y
        odom_callback.initialize_flag = 1

    # update travelled distance
    dist = math.sqrt((x - odom_callback.last_x)**2 + (y - odom_callback.last_y)**2)
    ODOM_current_distance += round(dist ,3)
    ODOM_current_yaw = round(math.degrees(yaw),3)

    # update last x and y
    odom_callback.last_x = x
    odom_callback.last_y = y
    ODOM_X.append(x)
    ODOM_Y.append(y)
    HEADING_U.append(u)
    HEADING_V.append(v)



def odom_1_callback(odom_data):
    global ODOM_1_X
    global ODOM_1_Y
    global HEADING_1_U
    global HEADING_1_V
    global ODOM_1_current_distance 
    global ODOM_1_current_yaw
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = getYaw(odom_data.pose.pose.orientation)
    u = math.cos(yaw)
    v = math.sin(yaw)
    
    if (odom_1_callback.initialize_flag == 0):    
        odom_1_callback.last_x = x
        odom_1_callback.last_y = y
        odom_1_callback.initialize_flag = 1

    # update travelled distance
    dist = math.sqrt((x - odom_1_callback.last_x)**2 + (y - odom_1_callback.last_y)**2)
    ODOM_1_current_distance += round(dist ,3)
    ODOM_1_current_yaw = round(math.degrees(yaw),3)

    # update last x and y
    odom_1_callback.last_x = x
    odom_1_callback.last_y = y
    
    ODOM_1_X.append(x)
    ODOM_1_Y.append(y)
    HEADING_1_U.append(u)
    HEADING_1_V.append(v)

def ekf_callback(odom_data):
    global EKF_X
    global EKF_Y
    global HEADING_EKF_U
    global HEADING_EKF_V
    global EKF_current_distance 
    global EKF_current_yaw
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = getYaw(odom_data.pose.pose.orientation)
    u = math.cos(yaw)
    v = math.sin(yaw)
    
    if (ekf_callback.initialize_flag == 0):    
        ekf_callback.last_x = x
        ekf_callback.last_y = y
        ekf_callback.initialize_flag = 1

    # update travelled distance
    dist = math.sqrt((x - ekf_callback.last_x)**2 + (y - ekf_callback.last_y)**2)
    EKF_current_distance += round(dist ,3)
    EKF_current_yaw = round(math.degrees(yaw),3)

    # update last x and y
    ekf_callback.last_x = x
    ekf_callback.last_y = y
    
    EKF_X.append(x)
    EKF_Y.append(y)
    HEADING_EKF_U.append(u)
    HEADING_EKF_V.append(v)


def update(frame_number):
    update.counter += 1
    deg = u'\xb0'

    # draw odom
    if len(ODOM_X) > 1:
        min_len = min(len(ODOM_X), len(ODOM_Y)) - 1
        ODOM_LINE.set_data(ODOM_X[-min_len:], ODOM_Y[-min_len:])
        ODOM_LINE.set_label(
                args.odom + " XY coords:[ {}, {} ]  Yaw:{}".format(ODOM_X[-1],ODOM_Y[-1],ODOM_current_yaw) + deg + "\n"
                +args.odom + " Mileage " + str(ODOM_current_distance)+" m \n") 
    
    # draw odom_1
    if len(ODOM_1_X) > 1:
        min_len = min(len(ODOM_1_X), len(ODOM_1_Y)) - 1
        ODOM_1_LINE.set_data(ODOM_1_X[-min_len:], ODOM_1_Y[-min_len:])
        ODOM_1_LINE.set_label(
                args.odom_1 + " XY coords:[ {}, {} ]  Yaw:{}".format(ODOM_1_X[-1],ODOM_1_Y[-1],ODOM_1_current_yaw) + deg + "\n"
                +args.odom_1 + " Mileage " + str(ODOM_1_current_distance)+" m \n") 
    
    # draw ekf 
    if len(EKF_X) > 1:
        min_len = min(len(EKF_X), len(EKF_Y)) - 1
        EKF_LINE.set_data(EKF_X[-min_len:], EKF_Y[-min_len:])
        EKF_LINE.set_label(
                args.ekf + " XY coords:[ {}, {} ]  Yaw:{}".format(EKF_X[-1],EKF_Y[-1],EKF_current_yaw) + deg + "\n"
                +args.ekf + " Mileage " + str(EKF_current_distance)+" m \n") 
    
    # draw heading
    if update.counter == update.heading_draw_interval and update.enable:
        if len(ODOM_X) > 1:
            ax.quiver(ODOM_X[-1],ODOM_Y[-1],HEADING_U[-1],HEADING_V[-1],color='r',scale=15,linewidth=2)
        if len(ODOM_1_X) > 1:
            ax.quiver(ODOM_1_X[-1],ODOM_1_Y[-1],HEADING_1_U[-1],HEADING_1_V[-1],color='k',scale=15,linewidth=2)
        if len(EKF_X) > 1:
            ax.quiver(EKF_X[-1],EKF_Y[-1],HEADING_EKF_U[-1],HEADING_EKF_V[-1],color='b',scale=15,linewidth=2)
        update.counter = 0 #reset

    #rescale the plot     
    ax.relim()
    ax.autoscale()
    #ax.autoscale_view(True,True,True)
    ax.legend()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description="""A visualization tool that can plot a trace produced by
        lionel log, and plot the trace in real time.
        The lionel log trace is the blue lines, and the live trace is the red lines.
        It is visualization a way to verify the precision of the dead reckoning.
        If you have a ros bag file, you can play the bag and the tool will plot
        the received odom message in realtime. To do that, start this tool first with a log trace, and then
        play record use another terminal with the following command [replace
        your_file.bag to your own bag file]: rosbag play your_file.bag
        """)
    parser.add_argument(
        "-log",
        action='store',
        type=str,
        help='the odom log csv file produced by lionel',
        default=None)

    parser.add_argument(
        "-odom",
        action='store',
        type=str,
        help='the ros topic name for odom to plot,rosmsg type should be nav_msgs.Odometry',default=None)

    parser.add_argument(
        "-odom_1",
        action='store',
        type=str,
        help='the ros topic name for odom_1 to plot,rosmsg type should be nav_msgs.Odometry',default=None)

    parser.add_argument(
        "-ekf",
        action='store',
        type=str,
        help='the ros topic name for fused odom to plot,rosmsg type should be geometry_msgs.PoseWithCovarianceStamped',default=None)
    
    parser.add_argument(
        "-heading",
        action='store',
        type=bool,
        help='draw heading on odom plot',default=False)

    args = parser.parse_args()
    
    #set figure size 18x14, inches maybe?
    fig, ax = plt.subplots(figsize=(18,14))
    
    ax.set_xlabel("Odom - X (unit:Meters)",fontsize=20)
    ax.set_ylabel("Odom - Y (unit:Meters)",fontsize=20)
    ax.set_title("Lionel Odom Plot {}".format(Version),fontsize=24)

    # initialize static vars in functions 
    odom_callback.last_x = 0
    odom_callback.last_y = 0
    odom_callback.initialize_flag = 0
    odom_1_callback.last_x = 0
    odom_1_callback.last_y = 0
    odom_1_callback.initialize_flag = 0
    ekf_callback.last_x = 0
    ekf_callback.last_y = 0
    ekf_callback.initialize_flag = 0
    update.enable = args.heading
    update.counter = 0
    update.heading_draw_interval = 10


    if args.log is not None:
        with open(args.log, 'r') as fp:
        #TODO: to define a standard csv file
            trace_data = np.genfromtxt(fp, delimiter=',',usecols=(1,2),skip_header=1,max_rows=100)
    
            LOG_LINE, = ax.plot(trace_data[:,0], trace_data[:,1], 'k-',alpha=0.5,linewidth=1,label=args.log)

    rospy.init_node("plot_trace")
    

    if args.odom is not None: 
        rospy.Subscriber(args.odom, Odometry, odom_callback)
        ODOM_LINE, = ax.plot(ODOM_X, ODOM_Y, 'r', linewidth=3)
    
    if args.ekf is not None:
        rospy.Subscriber(args.ekf, PoseWithCovarianceStamped, ekf_callback)
        EKF_LINE, = ax.plot(EKF_X, EKF_Y, 'b', linewidth=1,label=args.ekf)
    
    if args.odom_1 is not None: 
        rospy.Subscriber(args.odom_1, Odometry, odom_1_callback)
        ODOM_1_LINE, = ax.plot(ODOM_1_X, ODOM_1_Y, 'k', linewidth=3,label=args.odom_1)

    ani = animation.FuncAnimation(fig, update, interval=100)
    
    plt.legend()
    plt.grid(True)
    plt.show()

