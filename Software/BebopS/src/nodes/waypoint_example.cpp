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
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Int16.h>

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
//Variable to aquire drone pose
double drone_pose [4] = {};
double desired_pose [4] = {};
double tolerance = 0.1;

// Variables to aquire waypoints
 int flag_aqquired = 0;
 double t = 0;
 trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
 
 // Variable to get ready 
 int flag_ready = 1;
ros::Publisher pub_check;
 
 void gotwaypoint_callback(const geometry_msgs::PoseStampedConstPtr& msg)
 {
	if (flag_ready){
		//trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		trajectory_msg.header.stamp = ros::Time::now();
		Eigen::Vector3d desired_position( msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
		double desired_yaw = msg->pose.orientation.z;
		
		//Save to global variable desired pose
		desired_pose[0] = msg->pose.position.x;
		desired_pose[1] = msg->pose.position.y;
		desired_pose[2] = msg->pose.position.z;
		desired_pose[3] = desired_yaw;    
		
			
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
		t = msg->pose.orientation.x;
		
		flag_aqquired = 1;
		ROS_INFO ("Waypoint aquired");
		std_msgs::Int16 msg;
		msg.data = flag_aqquired;
		pub_check.publish(msg);	
	}
	else{
		ROS_INFO ("Can not go to next waypoint. Drone is busy");
	}
	

 }
 
 void check_arrived(){
	
	double error [4] = {};
	error [0] = drone_pose[0] - desired_pose [0];
	error [1] = drone_pose[1] - desired_pose [1];
	error [2] = drone_pose[2] - desired_pose [2];
	double abs_error = 0;
	abs_error = sqrtf(pow(error[0],2)+pow(error[1],2)+pow(error[1],3));
	//printf("Error is %lf",abs_error);
	if (abs_error <= tolerance && flag_ready == 0){
		flag_ready = 1;	
		std_msgs::Int16 msg;
		msg.data = 2;
		ROS_INFO ("Waypoint reached");
		pub_check.publish(msg);		
	}
	
}
 
  void pose_callback(const geometry_msgs::TransformStampedConstPtr& msg)
 {

	// Save data to drone_pose	
	  drone_pose[0] = msg->transform.translation.x;
	  drone_pose[1] = msg->transform.translation.y;
	  drone_pose[2] = msg->transform.translation.z;
	  
	  double q1 = msg->transform.rotation.x;
	  double q2 = msg->transform.rotation.y;
	  double q3 = msg->transform.rotation.z;
	  double q0 = msg->transform.rotation.w;

	  // Conversion quaternion to euler. Only yaw!=0.
	  double siny = +2.0 * (q0 * q3 + q1 * q2);
	  double cosy = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
	  drone_pose[3] = atan2(siny,cosy);
	   
	 //Check if drone has arrived to pose
	 check_arrived();
	  
	        
 }


int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_example");
  ros::NodeHandle nh;
  
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("bebop/command/trajectory", 100);
  pub_check = nh.advertise<std_msgs::Int16>("/drone_check", 10);
      
  ros::Subscriber sub = nh.subscribe("target_waypoints", 100, gotwaypoint_callback);
  ros::Subscriber pose_sub = nh.subscribe("bebop/transform", 100, pose_callback);
  

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
		  if (flag_ready){			  
			  trajectory_pub.publish(trajectory_msg);
			  flag_aqquired = 0;
			  flag_ready = 0;
			  ROS_INFO("Going to waypoint"); 
		  }	  
	}
	 
	loop_rate.sleep();
 }
 
}


