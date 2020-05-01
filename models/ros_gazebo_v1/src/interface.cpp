#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

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

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

