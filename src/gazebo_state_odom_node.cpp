#include <string>
#include <iostream>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Time.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"

#include <time.h>
#include <cmath>

// #include <Eigen/Core>
// #include <Eigen/Geometry>

class ground_truth
{
private:
	ros::NodeHandle n;
	ros::Subscriber sub1, sub2;
	ros::Publisher pub1, pub2;

	nav_msgs::Path path;
	geometry_msgs::PoseArray posearray;

	bool first;
	bool readyforpose;		// ~1 hz
	bool readyforpath;		// ~ 20 hz, include distance cal
	double distance;

	geometry_msgs::Pose lastpose;
	geometry_msgs::PoseStamped dyn_res;
	double lasttime;

public:
	ground_truth(int argc, char** argv) {
		sub1 = n.subscribe<sensor_msgs::Imu>("/mobile_base/sensors/imu_data", 10, &ground_truth::imu_callback, this);
		sub2 = n.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, &ground_truth::the_callback, this);

		pub1 = n.advertise<geometry_msgs::PoseArray>("poses", 10);
		pub2 = n.advertise<nav_msgs::Path>("path", 10);

		first = true;
		readyforpath = true;
		readyforpose = true;
		distance = 0;
		lasttime = 0;

		path.header.frame_id = "the_path";
		path.header.seq = 0;

		posearray.header.frame_id = "pose_array";
		posearray.header.seq = 0;

	}

	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
		if (msg->header.seq % 50 == 0) {
			readyforpath = true;
			// lasttime = msg->header.stamp.toSec();
			path.header.stamp = msg->header.stamp;

			geometry_msgs::PoseStamped dyn_res;
			dyn_res.header = msg->header;
			dyn_res.header.frame_id = "path_node";

			// path.poses.emplace_back(dyn_res);

			if (msg->header.seq % 1000 == 0) {
				readyforpose = true;
				posearray.header.stamp = msg->header.stamp;
			}
		}
	}

	void the_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
		if(msg->pose.size()<1)
			return;
		if (readyforpose) {
			posearray.poses.emplace_back(msg->pose.back());
			posearray.header.seq++;
			pub1.publish(posearray);
			readyforpose = false;
			ROS_INFO("posearray updated");
		}
		if (readyforpath) {
			dyn_res.pose = msg->pose.back();
			path.poses.emplace_back(dyn_res);
			path.poses.back().pose = msg->pose.back();
			
			if (first)
				first = false;
			else {
				double x_ = msg->pose.back().position.x;
				double y_ = msg->pose.back().position.y;
				double z_ = msg->pose.back().position.z;
				distance += std::sqrt(
				                (x_ - lastpose.position.x) * (x_ - lastpose.position.x) +
				                (y_ - lastpose.position.y) * (y_ - lastpose.position.y) +
				                (z_ - lastpose.position.z) * (z_ - lastpose.position.z));
			}
			ROS_INFO("Gazebo Ground true odom: %6.2f m", distance);

			lastpose = msg->pose.back();
			path.header.seq++;
			pub2.publish(path);
			readyforpath = false;
		}

	}

};
int main(int argc, char** argv) {
	ros::init(argc, argv, "show_ground_truth");
	ground_truth c(argc, argv);
	ros::spin();
	return 0;
}