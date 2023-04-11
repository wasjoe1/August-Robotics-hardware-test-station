#include <ros/ros.h>
#include <ros/package.h>
#include "rosgraph_msgs/Clock.h"
#include <string>


/* The implementation of SimClock.h */
//namespace KCL_rosplan {
//
//	/* constructor */
//	SimClock::SimClock(ros::NodeHandle &nh) {
//        time_server = nh.advertiseService("set_time_scale", &KCL_rosplan::SimClock::setTmeScale, this);
//        time_scale = 1;
//	}
//
//    /**
//     * Set the time scale to speed a simulation.
//     */
//    bool SimClock::setTmeScale(rosplan_knowledge_msgs::SetInt::Request &req, rosplan_knowledge_msgs::SetInt::Response &res) {
//        time_scale = req.value;
//        res.success = true;
//        return true;
//    }
//}

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "sim_clock");
		ros::NodeHandle nh("~");
        int default_rate;
        default_rate = 200;
		// create PDDL action subscriber
//		KCL_rosplan::SimClock sc(nh);
        std::string time_scale_temp;
        nh.getParam("set_time_scale",time_scale_temp);
        int time_scale = std::stoi(time_scale_temp);
        ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1, true);

        rosgraph_msgs::Clock msg;
        ros::Time time_now = ros::Time(ros::WallTime::now().toSec());
        std::cout << time_now << std::endl;
        ros::WallTime realStart = ros::WallTime::now();
        ros::WallTime realCurrent = ros::WallTime::now();
        ros::WallRate loop_rate(default_rate * time_scale);
        msg.clock = time_now;
        while(ros::ok()) {
            realCurrent = ros::WallTime::now();
            time_now = time_now + ros::Duration((realCurrent - realStart).toSec()) * time_scale;
            msg.clock = time_now;
            clock_pub.publish(msg);
            realStart = ros::WallTime::now();
            ros::spinOnce();
            loop_rate.sleep();
        }

		return 0;
	}