#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0),
        position(0, 0, 0),
        yaw(0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : waiting_time(t), 
        position(x, y, z), 
        yaw(_yaw)  {
  }

  double waiting_time;
  Eigen::Vector3d position;
  double yaw;
  
};

// Variables to aquire waypoints
 int flag_aqquired = 0;
 double t = 0;
 trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
 
 // Variable to get ready 
 int flag_ready = 0;
 
 void gotwaypoint_callback(const geometry_msgs::PoseStampedConstPtr& msg)
 {
	//trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();
	Eigen::Vector3d desired_position( msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    double desired_yaw = msg->pose.orientation.z;
        
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
	t = msg->pose.orientation.x;
	
	flag_aqquired = 1;
	ROS_INFO ("waypoint aquired in callback");

 }


int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_example");
  ros::NodeHandle nh;
  
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("bebop/command/trajectory", 100);
      
  ros::Subscriber sub = nh.subscribe("target_waypoints", 100, gotwaypoint_callback);
  

  ROS_INFO("Started waypoint example.");
  

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for t seconds to let the Gazebo GUI show up.
  double t1 = 0.5;
  ros::Duration(t1).sleep();
  ros::Rate loop_rate(t1);

  ROS_INFO("Waiting for target");
  while (ros::ok()){
	  
	  ros::spinOnce();
	  
	  if (flag_aqquired){
		  
		  ROS_INFO("Waypoint aquired");
		  flag_aqquired = 0;
		  trajectory_pub.publish(trajectory_msg);
		  ROS_INFO("Going to waypoint"); 
		  
	 }
	 
	// Wait for t seconds to let the Gazebo GUI show up.  
	ros::Duration(t).sleep();
	t = 0;
	
	loop_rate.sleep();
 }
 
}


