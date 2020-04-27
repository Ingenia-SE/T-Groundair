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
			ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/check",10,
      				boost::bind(&drone::callbackCheck, this, _1),ros::VoidPtr(), &this->rosQueue);
			image_transport::ImageTransport it(this->drone_node);
			this->pub = it.advertise("/image_drone", 1);
			cv::Mat image = cv::imread("/home/alvaro/gazebo_ros_ws/src/ros_gazebo_v1/src/tennis5.jpg", cv::IMREAD_UNCHANGED);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			this->sub = this->drone_node.subscribe(so);
			this->rosQueueThread = std::thread(std::bind(&drone::QueueThread, this));
			this->pub.publish(msg);
			//spinner.spin();
		}
		
		public: void callbackCheck(const std_msgs::Float32ConstPtr &_msg){
			float info=0.0;
			info=_msg->data;
			ROS_INFO("Cycle ended");
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread(){
			static const double timeout = 0.01;
			while (this->drone_node.ok()){
				ros::spinOnce();
				//this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		//public: void OnUpdate(){
		//	this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		//}

		private: physics::ModelPtr model;
		private: ros::NodeHandle drone_node;
		//private: std::unique_ptr<ros::NodeHandle> drone_node;
		private: ros::Subscriber sub;
		private: image_transport::Publisher pub;
		private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;

		// Pointer to the update event connection
		// private: event::ConnectionPtr updateConnection;
	};

GZ_REGISTER_MODEL_PLUGIN(drone)
}
