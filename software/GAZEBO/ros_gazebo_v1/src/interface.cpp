#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <sstream>
#include <iostream>

using namespace std;

void centralCallback(const std_msgs::Int16::ConstPtr& msg)
{
	cout << "Cycle ended\n";
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "interface");
	ros::NodeHandle interface;
	ros::Publisher order_pub = interface.advertise<std_msgs::Int16>("/interface", 10);
	ros::Subscriber state_sub = interface.subscribe("/central_ok", 10, centralCallback);
	ros::Rate loop_rate(100);
	ros::Publisher waypoints_pub = interface.advertise<geometry_msgs::PoseStamped>("target_waypoints", 100);

	cout << "Press 1 for beginning the cycle\n";

	int a;
	cin >> a;

	while (a != 1){
		cout <<	"Not valid number. Try again.\n";
		cin >> a;
	}

	std_msgs::Int16 msg;
	msg.data = a;
	order_pub.publish(msg);
  
	geometry_msgs::PoseStamped msg_point;
	 
	float point1 []= {10, -10 ,1, 0, 20};
	float point2 []= {0, -10, 20, 0 ,20};
	float point3 []= {-10, -10, 10, 0, 20};
	 
	 //First point
	
	 msg_point.pose.position.x = point1[0];
	 msg_point.pose.position.y = point1[1];
	 msg_point.pose.position.z = point1[2];
	 msg_point.pose.orientation.x = point1[4];
	 msg_point.pose.orientation.z = point1[3];
	
	 printf("Sended point 1\n");
     waypoints_pub.publish(msg_point);
     
  cin >> a;
	 //Second point
	 msg_point.pose.position.x = point2[0];
	 msg_point.pose.position.y = point2[1];
	 msg_point.pose.position.z = point2[2];
	 msg_point.pose.orientation.x = point2[4];
	 msg_point.pose.orientation.z = point2[3];
	
	printf("Sended point 2\n");
    waypoints_pub.publish(msg_point);
   cin >> a;  
	 //third point
	 msg_point.pose.position.x = point3[0];
	 msg_point.pose.position.y = point3[1];
	 msg_point.pose.position.z = point3[2];
	 msg_point.pose.orientation.x = point3[4];
	 msg_point.pose.orientation.z = point3[3];
  
	printf("Sended point 3\n");
    waypoints_pub.publish(msg_point);

	printf("End of program");
	

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

