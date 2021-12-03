

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
 * @file   ros_node.cpp
 * @author Roy Zhang (roy.zhang@augustrobotics.com)
 * @date   25-10-2021
 *
 * @brief
 *
 */
#include <signal.h>
#include <execinfo.h>
#include <stdexcept>
#include <memory>
#include <ros/ros.h>
#include "ros_boothbot_debug_gui/ros_boothbot_debug_gui.h"
void handler(int sig)
{
    void* array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

/* Main node entry point. */
int main(int argc, char** argv)
{
    signal(SIGSEGV, handler);
    ros::init(argc, argv, "boothbot_debug_gui");
    ros::NodeHandle nh("~");
    august::BoothbotDebugGuiNode debug_gui_node(nh);
    double rate_Hz = 0;
    nh.param<double>("rate_Hz", rate_Hz, 10);
    ros::Rate r(rate_Hz);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
