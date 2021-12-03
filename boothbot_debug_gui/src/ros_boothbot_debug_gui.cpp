/**
 * ________________________________________________________________________
 * August Robotics    CONFIDENTIAL
 * ________________________________________________________________________
 *
 * This file is part of August Robotics
 *
 * Copyright (c) 2021 August Robotics,
 * All Rights Reserved.
 *
 * Unauthorized copying of this file and software, partly or in whole, via any
 * medium is strictly prohibited, including all derived formats such as object
 * code or binaries.
 *
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of August Robotics. The intellectual and technical concepts
 * contained herein are proprietary to August Robotics. and may be covered by
 * U.S and Foreign Patents, patents in process, and are protected by
 * trade secret or copyright law. Dissemination of this information
 * or reproduction of this material is strictly forbidden unless prior
 * written permission is obtained from August Robotics.
 *
 * @file   boothbot_debug_gui.cpp
 * @author Roy Zhang (roy.zhang@augustrobotics.com)
 * @date   25-10-2021
 *
 * @brief
 *
 */
#include "ros_boothbot_debug_gui/ros_boothbot_debug_gui.h"
#include "ros_boothbot_debug_gui/ncurses_utils.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <math.h>
namespace august
{
enum topicPosition
{
    GO_TO_MARK_STATUS_POSITION = 2,
    TASK_MANAGER_STATUS_POSITION = 3,
    CB_STATUS_POSITION = 4,
    GBM_STATUS_POSTION = 5,
    MARKING_STATUS_POSITION = 6,
    NAV_STATUS_POSITION = 7,
    MOVEBASE_STATUS_POSITION = 8,
    ROBOT_STATUS_POSITION = 9,
    SONAR_STATUS_POSITION = 10,
    CHASSIS_STATUS_POSITION = 12
};
BoothbotDebugGuiNode::BoothbotDebugGuiNode(const ros::NodeHandle& nh)
    : nh_(nh)
    , ros_master_uri_(std::getenv("ROS_MASTER_URI"))
    , ros_ip_(std::getenv("ROS_IP"))
    , ros_hostname_(std::getenv("ROS_HOSTAME"))
{
    initGui();
    createPoseSubscriber();
    createStatusSubscriber();
    createSpeedSubscriber();
    //Todo
    // createPublisher();
    // createServiceCall();
}

BoothbotDebugGuiNode::~BoothbotDebugGuiNode()
{
    delwin(main_window_);
    delwin(pose_window_);
    delwin(status_window_);
    endwin();
}

bool BoothbotDebugGuiNode::initGui()
{
    initscr();
    cbreak();
    // raw();
    // keypad(stdscr, FALSE);
    noecho();
    curs_set(0);
    timeout(0);
    refresh();
    getmaxyx(stdscr, screen_rows_, screen_cols_);

    pose_window_ = newwin(9, screen_cols_, 0, 0);
    wrefresh(pose_window_);

    main_window_ = newwin(screen_rows_ - 12, screen_cols_, 10, 0);
    wrefresh(main_window_);

    status_window_ = newwin(2, screen_cols_, screen_rows_ - 1, 0);
    wrefresh(status_window_); /* 初始化屏幕 */
    start_color();
    use_default_colors();
    init_pair(1, COLOR_RED, -1);
    init_pair(2, COLOR_GREEN, -1);
    init_pair(3, COLOR_YELLOW, -1);
    box(pose_window_, '.', '.');  //画框
    box(main_window_, '.', '.');
    // box(status_window_, '.', '.');
    ncurses_utils_.addTitle(2, "RosTopic");
    ncurses_utils_.addTitle(42, "x");
    ncurses_utils_.addTitle(12, "y");
    ncurses_utils_.addTitle(12, "theta (degree)");
    // title 1
    ncurses_utils_.paintWindowHeaderRow(pose_window_, 1);
    ncurses_utils_.clearTitle();
    ncurses_utils_.addTitle(2, "RosTopic");
    ncurses_utils_.addTitle(32, "Status");
    ncurses_utils_.addTitle(22, "Task");
    ncurses_utils_.paintWindowHeaderRow(main_window_, 1);
    // title 2
    ncurses_utils_.clearTitle();
    ncurses_utils_.addTitle(2, " ");
    ncurses_utils_.addTitle(32, "linear_x");
    ncurses_utils_.addTitle(12, "linear_y");
    ncurses_utils_.addTitle(12, "angular_x");
    ncurses_utils_.addTitle(12, "angular_y");
    ncurses_utils_.paintWindowHeaderRow(pose_window_, 5);
    // title 3
    ncurses_utils_.clearTitle();
    ncurses_utils_.addTitle(2, " ");
    ncurses_utils_.addTitle(24, "power_on");
    ncurses_utils_.addTitle(10, "enable");
    ncurses_utils_.addTitle(10, "imu_odom");
    ncurses_utils_.addTitle(10, "pc_on");
    ncurses_utils_.addTitle(10, "m_1");
    ncurses_utils_.addTitle(10, "m_2");
    ncurses_utils_.addTitle(10, "m_3");
    ncurses_utils_.addTitle(10, "m_4");
    ncurses_utils_.paintWindowHeaderRow(main_window_, CHASSIS_STATUS_POSITION - 1);
    //get status
    std::string ros_status = ros::master::check() ? std::string("True ") : std::string("False");
    std::string ros_master_uri_str = "";
    std::string ros_ip_str = "";
    std::string ros_hostname_str = "";
    //avoid core dump if void string
    if (ros_master_uri_)
    {
        ros_master_uri_str = std::string(ros_master_uri_);
    }
    if (ros_ip_)
    {
        ros_ip_str = std::string(ros_ip_);
    }
    if (ros_hostname_)
    {
        ros_hostname_str = std::string(ros_hostname_);
    }
    std::string status = "ROS: " + ros_status + ", ROS_MASTER_URI: " + ros_master_uri_str + ", ROS_IP: " + ros_ip_str +
                         ", ROS_HOSTNAME: " + ros_hostname_str;
    writeStatus(status);
    return true;
}
void BoothbotDebugGuiNode::writeStatus(const std::string& status)
{
    mvwprintw(status_window_, 0, 0, status.c_str());
    wrefresh(status_window_);
}
void BoothbotDebugGuiNode::input2Gui()
{
    bool running = true;
    while (running)
    {
        //TODO
    }
}

void BoothbotDebugGuiNode::createPoseSubscriber()
{
    std::string odometry_topic;
    //get the topic name
    nh_.param<std::string>("odometry_topic", odometry_topic, "/drivers/chassis/odom");
    //get queue size
    int odometry_queue_size;
    nh_.param<int>("odometry_queue_size", odometry_queue_size, 10);
    //create subscriber
    odometry_sub_ = nh_.subscribe("/drivers/chassis/odom", 1, &BoothbotDebugGuiNode::odometryCallback, this);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 2, 0, "%s", "/drivers/chassis/odom", 1, false);

    gps_sub_ = nh_.subscribe("/modules/gbm/gps", 0, &BoothbotDebugGuiNode::gpsCallback, this);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 3, 0, "%s", "/modules/gbm/gps", 1, false);

    ekf_poses_sub_ = nh_.subscribe("/drivers/ekf/robot_pose", 1, &BoothbotDebugGuiNode::ekfPosesCallback, this);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 4, 0, "%s", "/drivers/ekf/robot_pose", 1, false);
}

void BoothbotDebugGuiNode::createStatusSubscriber()
{
    int queue_size = 10;

    apps_goto_mark_status_sub_ =
        nh_.subscribe("/apps/goto_mark/status", 1, &BoothbotDebugGuiNode::appsGotomarkStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, GO_TO_MARK_STATUS_POSITION, 0, "%s", "/apps/goto_mark/status", 1, false);

    apps_task_manager_status_sub_ =
        nh_.subscribe("/apps/task_manager/status", 1, &BoothbotDebugGuiNode::appsTaskManagerStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, TASK_MANAGER_STATUS_POSITION, 0, "%s", "/apps/task_manager/status", 1, false);

    drivers_chassis_status_sub_ =
        nh_.subscribe("/drivers/chassis/status", 1, &BoothbotDebugGuiNode::driversChassisStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, CHASSIS_STATUS_POSITION, 0, "%s", "/drivers/chassis/status", 1, false);

    drivers_sonars_status_sub_ =
        nh_.subscribe("/drivers/sonars/status", 1, &BoothbotDebugGuiNode::driversSonarStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(main_window_, SONAR_STATUS_POSITION, 0, "%s", "/drivers/sonar/status", 1, false);

    modules_cb_status_sub_ =
        nh_.subscribe("/modules/cb/status", 1, &BoothbotDebugGuiNode::modulesCbStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(main_window_, CB_STATUS_POSITION, 0, "%s", "/modules/cb/status", 1, false);

    modules_gbm_status_sub_ =
        nh_.subscribe("/modules/gbm/status", 1, &BoothbotDebugGuiNode::modulesGbmStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(main_window_, GBM_STATUS_POSTION, 0, "%s", "/modules/gbm/status", 1, false);

    modules_marking_status_sub_ =
        nh_.subscribe("/modules/marking/status", 1, &BoothbotDebugGuiNode::modulesMarkingStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, MARKING_STATUS_POSITION, 0, "%s", "/modules/marking/status", 1, false);

    modules_nav_status_sub_ =
        nh_.subscribe("/modules/nav/status", 1, &BoothbotDebugGuiNode::modulesNavStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(main_window_, NAV_STATUS_POSITION, 0, "%s", "/modules/nav/status", 1, false);

    movebase_status_sub_ = nh_.subscribe("/move_base/status", 1, &BoothbotDebugGuiNode::movebaseStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(main_window_, MOVEBASE_STATUS_POSITION, 0, "%s", "/move_base/status", 1, false);

    robot_status_sub_ = nh_.subscribe("/robot/status", 1, &BoothbotDebugGuiNode::robotStatusCallback, this);
    ncurses_utils_.ifWindowColorPrint(main_window_, ROBOT_STATUS_POSITION, 0, "%s", "/robot/status", 1, false);
}

void BoothbotDebugGuiNode::createSpeedSubscriber()
{
    drivers_chassis_act_vel_stamped_ = nh_.subscribe(
        "/drivers/chassis/act_vel_stamped", 1, &BoothbotDebugGuiNode::driversChassisActvelstampedCallback, this);

    ncurses_utils_.ifWindowColorPrint(pose_window_, 6, 0, "%s", "/drivers/chassis/act_vel_stamped", 1, false);

    drivers_chassis_cmd_vel_stamped_ = drivers_chassis_cmd_vel_stamped_ = nh_.subscribe(
        "/drivers/chassis/cmd_vel_stamped", 1, &BoothbotDebugGuiNode::driversChassiscmdvelstampedCallback, this);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 7, 0, "%s", "/drivers/chassis/cmd_vel_stamped", 1, false);
}

void BoothbotDebugGuiNode::createPublisher()
{
}

void BoothbotDebugGuiNode::createServiceCall()
{
    ros::ServiceClient client = nh_.serviceClient<boothbot_msgs::Command>("/move_base_node/reset");
    boothbot_msgs::Command command_srv;
    command_srv.request.command = "RESET";
    if (client.call(command_srv))
    {
        ROS_INFO("Success");
    }
    else
    {
        ROS_INFO("Unsuccess");
    }
}

void BoothbotDebugGuiNode::odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    nav_msgs::Odometry odometry_trans;
    double odom_time = odometry_msg->header.stamp.toSec();
    double x = odometry_msg->pose.pose.position.x;
    double y = odometry_msg->pose.pose.position.y;
    double yaw;
    quantanion2YawDegree(odometry_msg, yaw);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 2, 42, "%f", odometry_msg->pose.pose.position.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 2, 54, "%f", odometry_msg->pose.pose.position.y, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 2, 66, "%f", yaw, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::gpsCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    nav_msgs::Odometry odometry_trans;
    double odom_time = odometry_msg->header.stamp.toSec();
    double x = odometry_msg->pose.pose.position.x;
    double y = odometry_msg->pose.pose.position.y;
    double yaw;
    quantanion2YawDegree(odometry_msg, yaw);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 3, 42, "%f", odometry_msg->pose.pose.position.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 3, 54, "%f", odometry_msg->pose.pose.position.y, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 3, 66, "%f", yaw, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::ekfPosesCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odometry_msg)
{
    double x = odometry_msg->pose.pose.position.x;
    double y = odometry_msg->pose.pose.position.y;

    double yaw;
    quantanion2YawDegree(odometry_msg, yaw);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 4, 42, "%f", odometry_msg->pose.pose.position.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 4, 54, "%f", odometry_msg->pose.pose.position.y, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 4, 66, "%f", yaw, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::appsGotomarkStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, GO_TO_MARK_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, GO_TO_MARK_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::appsTaskManagerStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, TASK_MANAGER_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, TASK_MANAGER_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::driversChassisStatusCallback(const augustbot_msgs::ChassisStatus& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      26,
                                      "%s",
                                      status_msg.dyn_power ? "On" : "Off",
                                      status_msg.dyn_power ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      36,
                                      "%s",
                                      status_msg.enabled ? "On" : "Off",
                                      status_msg.enabled ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      48,
                                      "%s",
                                      status_msg.imu_odom ? "On" : "Off",
                                      status_msg.imu_odom ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      56,
                                      "%s",
                                      status_msg.pc_online ? "On" : "Off",
                                      status_msg.pc_online ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      66,
                                      "%s",
                                      status_msg.motors_online[0] ? "On" : "Off",
                                      status_msg.motors_online[0] ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      76,
                                      "%s",
                                      status_msg.motors_online[1] ? "On" : "Off",
                                      status_msg.motors_online[1] ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      86,
                                      "%s",
                                      status_msg.motors_online[2] ? "On" : "Off",
                                      status_msg.motors_online[2] ? COLOR_GREEN : COLOR_RED,
                                      true);
    ncurses_utils_.ifWindowColorPrint(main_window_,
                                      CHASSIS_STATUS_POSITION,
                                      96,
                                      "%s",
                                      status_msg.motors_online[3] ? "On" : "Off",
                                      status_msg.motors_online[3] ? COLOR_GREEN : COLOR_RED,
                                      true);
}

void BoothbotDebugGuiNode::driversSonarStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, SONAR_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, SONAR_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::modulesCbStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, CB_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, CB_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::modulesGbmStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, GBM_STATUS_POSTION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, GBM_STATUS_POSTION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::modulesMarkingStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, MARKING_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, MARKING_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::modulesNavStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, NAV_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, NAV_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}

void BoothbotDebugGuiNode::movebaseStatusCallback(const actionlib_msgs::GoalStatusArray& status_msg)
{
    if (!status_msg.status_list.empty())
    {
        ncurses_utils_.ifWindowColorPrint(main_window_,
                                          MOVEBASE_STATUS_POSITION,
                                          34,
                                          "%d",
                                          int(status_msg.status_list[0].status),
                                          checkTheStatusColor(status_msg),
                                          true);
    }
    else
    {
        ncurses_utils_.ifWindowColorPrint(main_window_, MOVEBASE_STATUS_POSITION, 34, "%s", "NULL", COLOR_YELLOW, true);
    }
    ncurses_utils_.ifWindowColorPrint(main_window_, MOVEBASE_STATUS_POSITION, 59, "%s", "NULL", COLOR_GREEN, true);
}
void BoothbotDebugGuiNode::robotStatusCallback(const boothbot_msgs::Status& status_msg)
{
    ncurses_utils_.ifWindowColorPrint(
        main_window_, ROBOT_STATUS_POSITION, 34, "%s", status_msg.state, checkTheStatusColor(status_msg), true);
    ncurses_utils_.ifWindowColorPrint(
        main_window_, ROBOT_STATUS_POSITION, 59, "%s", status_msg.current_task, COLOR_GREEN, true);
}
void BoothbotDebugGuiNode::driversChassisActvelstampedCallback(
    const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg)
{
    ncurses_utils_.ifWindowColorPrint(pose_window_, 6, 34, "%f", twist_msg->twist.twist.linear.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 6, 46, "%f", twist_msg->twist.twist.linear.y, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 6, 58, "%f", twist_msg->twist.twist.angular.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 6, 70, "%f", twist_msg->twist.twist.angular.y, COLOR_GREEN, true);
}
void BoothbotDebugGuiNode::driversChassiscmdvelstampedCallback(const geometry_msgs::TwistStampedConstPtr& twist_msg)
{
    ncurses_utils_.ifWindowColorPrint(pose_window_, 7, 34, "%f", twist_msg->twist.linear.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 7, 46, "%f", twist_msg->twist.linear.y, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 7, 58, "%f", twist_msg->twist.angular.x, COLOR_GREEN, true);
    ncurses_utils_.ifWindowColorPrint(pose_window_, 7, 70, "%f", twist_msg->twist.angular.y, COLOR_GREEN, true);
}

int BoothbotDebugGuiNode::checkTheStatusColor(const boothbot_msgs::Status& status)
{
    if (status.state == "INIT" || status.state == "ERROR" || status.state == "RESETING" || status.state == "STOPED")
    {
        return COLOR_RED;
    }
    return COLOR_GREEN;
}
int BoothbotDebugGuiNode::checkTheStatusColor(const actionlib_msgs::GoalStatusArray& status)
{
    //PENDING = 0u,
    // ACTIVE = 1u,
    // PREEMPTED = 2u,
    // SUCCEEDED = 3u,
    // ABORTED = 4u,
    // REJECTED = 5u,
    // PREEMPTING = 6u,
    // RECALLING = 7u,
    // RECALLED = 8u,
    // LOST = 9u,
    if (status.status_list[0].status == 4 || status.status_list[0].status == 5 || status.status_list[0].status == 9)
    {
        return COLOR_RED;
    }
    return COLOR_GREEN;
}

void BoothbotDebugGuiNode::quantanion2YawDegree(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odometry_msg,
                                                double& yaw_degree)
{
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x,
                     odometry_msg->pose.pose.orientation.y,
                     odometry_msg->pose.pose.orientation.z,
                     odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw_degree = yaw / M_PI * 180;
}
void BoothbotDebugGuiNode::quantanion2YawDegree(const nav_msgs::OdometryConstPtr& odometry_msg, double& yaw_degree)
{
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x,
                     odometry_msg->pose.pose.orientation.y,
                     odometry_msg->pose.pose.orientation.z,
                     odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw_degree = yaw / M_PI * 180;
}

}  // namespace august
