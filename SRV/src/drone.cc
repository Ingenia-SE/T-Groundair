#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros_gazebo_v1/balls.h"
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <functional>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

int info_uav = 1;
int info_ugv = 1;
int imagen = 0;

using namespace std;
using namespace cv;

//ros::MultiThreadedSpinner spinner;

namespace gazebo {
	class drone : public ModelPlugin {
		
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
			this->model = _parent;
			if (!ros::isInitialized()){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv,"drone_node",ros::init_options::NoSigintHandler);
			}

			ROS_INFO("Hello World!");
			//this->drone_node.reset(new ros::NodeHandle("drone_node"));
			//ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/check",10,
      			//	boost::bind(&drone::callbackCheck, this, _1),ros::VoidPtr(), &this->rosQueue);
			ros::SubscribeOptions so2 = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("target_waypoints",100,
      				boost::bind(&drone::callbackWaypoints, this, _1),ros::VoidPtr(), &this->rosQueue);
			ros::SubscribeOptions so3 = ros::SubscribeOptions::create<geometry_msgs::Point>("/ugv_waypoints",10,
      				boost::bind(&drone::callbackUGV, this, _1),ros::VoidPtr(), &this->rosQueue);	
	
			this->pub_check = this->drone_node.advertise<std_msgs::Int16>("/drone_check",10);
			this->pub_check_ugv = this->drone_node.advertise<std_msgs::Int16>("/ugv_check",10);

			image_transport::ImageTransport it(this->drone_node);
			//this->sub = this->drone_node.subscribe(so);
			this->pub = it.advertise("/image_drone",1);
			this->waypoints_sub = this->drone_node.subscribe(so2);
			this->ugv_sub = this->drone_node.subscribe(so3);
			this->rosQueueThread = std::thread(std::bind(&drone::QueueThread, this));
			//spinner.spin();
		}
		
		//public: void callbackCheck(const std_msgs::Float32ConstPtr &_msg){
			//Podria plantearse que en caso contrario se volviera a enviar la imagen		
		//	float info=0.0;
		//	info=_msg->data;
			
		//	if (info == 0) ROS_INFO("Cycle ended successfully");
		//	else ROS_INFO("Cycle NOT ended successfully");
		//}

		public: void callbackWaypoints(const geometry_msgs::PoseStampedConstPtr &_msg){		

			if(imagen == 0){
				cv::Mat image = cv::imread("/home/alvaro/gazebo_ros_ws/src/ros_gazebo_v1/src/pistaentera.jpg", cv::IMREAD_UNCHANGED);
				sensor_msgs::ImagePtr new_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();	
				this->pub.publish(new_msg);
				imagen = 1;
			}

			std_msgs::Int16 msg;
			msg.data = info_uav;
			this->pub_check.publish(msg);

			info_uav = 2;
			msg.data = info_uav;
			this->pub_check.publish(msg);
			info_uav = 1;
		}

		public: void callbackUGV(const geometry_msgs::PointConstPtr &_msg){
			
			cout << "OBJETIVO X" << _msg->x << "\n";	
			cout << "OBJETIVO Y" << _msg->y << "\n";	

			std_msgs::Int16 msg;
			msg.data = info_ugv;
			this->pub_check_ugv.publish(msg);

			info_ugv = 2;
			msg.data = info_ugv;
			this->pub_check_ugv.publish(msg);
			info_ugv = 1;
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread(){
			static const double timeout = 0.01;
			while (this->drone_node.ok()){
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
				ros::spinOnce();
			}
		}

		//public: void OnUpdate(){
		//	this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		//}

		private: physics::ModelPtr model;
		private: ros::NodeHandle drone_node;
		//private: std::unique_ptr<ros::NodeHandle> drone_node;
		//private: ros::Subscriber sub;
		private: ros::Subscriber waypoints_sub;
		private: ros::Subscriber ugv_sub;
		private: ros::Publisher pub_check;
		private: ros::Publisher pub_check_ugv;
		private: image_transport::Publisher pub;
		private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;

		// Pointer to the update event connection
		// private: event::ConnectionPtr updateConnection;
	};

GZ_REGISTER_MODEL_PLUGIN(drone)
}
