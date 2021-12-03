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
 * @file   boothbot_debug_gui.h
 * @author Roy Zhang (roy.zhang@augustrobotics.com)
 * @date   25-10-2021
 *
 * @brief
 *
 */
#ifndef ROS_BOOTHBOT_DEBUG_GUI_H
#define ROS_BOOTHBOT_DEBUG_GUI_H
#include <string>
#include <memory>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <boothbot_msgs/BoothbotTaskAction.h>
#include <ncurses.h>
#include <thread>
#include "ros_boothbot_debug_gui/ncurses_utils.h"
#include <nav_msgs/Odometry.h>
#include <boothbot_msgs/GuidingStationInfo.h>
#include <boothbot_msgs/Command.h>
#include <boothbot_msgs/Status.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <augustbot_msgs/ChassisStatus.h>
namespace august
{
/**
 * @brief boothbot debug GUI ROS node
 *
 */
class BoothbotDebugGuiNode
{
public:
    /**
     * @brief  Construct a new BoothbotDebugGuiNode.
     *
     * @param  private_nh :
     */
    BoothbotDebugGuiNode(const ros::NodeHandle& private_nh);

    /**
     * @brief  Destroy the BoothbotDebugGuiNode.
     *
     */
    ~BoothbotDebugGuiNode();

private:
    /**
     * @brief
     *
     * @return bool :
     */
    bool initGui();

    /**
     * @brief
     *
     */
    void input2Gui();

    /**
     * @brief  Create a PoseSubscriber.
     *
     */
    void createPoseSubscriber();

    /**
     * @brief  Create a StatusSubscriber.
     *
     */
    void createStatusSubscriber();

    /**
     * @brief  Create a SpeedSubscriber.
     *
     */
    void createSpeedSubscriber();

    /**
     * @brief  Create a Publisher.
     *
     */
    void createPublisher();

    /**
     * @brief
     *
     * @param  status :
     */
    void writeStatus(const std::string& status);

    /**
     * @brief  Create a ServiceCall.
     *
     */
    void createServiceCall();

    // Pose callback function
    /**
     * @brief
     *
     * @param  odometry_msg :
     */
    void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    /**
     * @brief
     *
     * @param  odometry_msg :
     */
    void gpsCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    /**
     * @brief
     *
     * @param  odometry_msg :
     */
    void ekfPosesCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odometry_msg);

    // Status callback function
    /**
     * @brief
     *
     * @param  status_msg :
     */
    void appsGotomarkStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void appsTaskManagerStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void driversChassisStatusCallback(const augustbot_msgs::ChassisStatus& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void driversSonarStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void modulesCbStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void modulesGbmStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void modulesMarkingStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void modulesNavStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray& status_msg);

    /**
     * @brief
     *
     * @param  status_msg :
     */
    void robotStatusCallback(const boothbot_msgs::Status& status_msg);

    /**
     * @brief
     *
     * @param  twist_msg :
     */
    void driversChassisActvelstampedCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg);

    /**
     * @brief
     *
     * @param  twist_msg :
     */
    void driversChassiscmdvelstampedCallback(const geometry_msgs::TwistStampedConstPtr& twist_msg);

    /**
     * @brief
     *
     * @param  status :
     * @return int :
     */
    int checkTheStatusColor(const actionlib_msgs::GoalStatusArray& status);

    /**
     * @brief
     *
     * @param  status :
     * @return int :
     */
    int checkTheStatusColor(const boothbot_msgs::Status& status);

    /**
     * @brief
     *
     * @param  odometry_msg :
     * @param  yaw_degree :
     */
    void quantanion2YawDegree(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odometry_msg, double& yaw_degree);

    /**
     * @brief
     *
     * @param  odometry_msg :
     * @param  yaw_degree :
     */
    void quantanion2YawDegree(const nav_msgs::OdometryConstPtr& odometry_msg, double& yaw_degree);

    //ROS node handle.
    ros::NodeHandle nh_;
    august::NcursesUtils ncurses_utils_;
    std::thread input_;
    //odometry sub
    ros::Subscriber odometry_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber ekf_poses_sub_;
    std::vector<nav_msgs::Odometry> odometry_buffer_;
    std::vector<nav_msgs::Odometry> teb_poses_buffer_;
    std::vector<nav_msgs::Odometry> ekf_poses_buffer_;

    //status sub
    ros::Subscriber apps_goto_mark_status_sub_;
    ros::Subscriber apps_task_manager_status_sub_;
    ros::Subscriber drivers_chassis_status_sub_;
    ros::Subscriber drivers_sonars_status_sub_;
    ros::Subscriber modules_cb_status_sub_;
    ros::Subscriber modules_gbm_status_sub_;
    ros::Subscriber modules_marking_status_sub_;
    ros::Subscriber modules_nav_status_sub_;
    ros::Subscriber movebase_status_sub_;
    ros::Subscriber robot_status_sub_;

    //speed sub
    ros::Subscriber drivers_chassis_act_vel_stamped_;
    ros::Subscriber drivers_chassis_cmd_vel_stamped_;

    //pub
    ros::Publisher output_cloud_publisher_;
    ros::Publisher odo_pub_;

    int screen_rows_;
    int screen_cols_;
    const char* ros_master_uri_;
    const char* ros_ip_;
    const char* ros_hostname_;
    WINDOW* main_window_;
    WINDOW* pose_window_;
    WINDOW* status_window_;
    WINDOW* input_window_;
};

}  // namespace august

#endif
