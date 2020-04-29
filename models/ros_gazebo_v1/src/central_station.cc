#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <functional>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "ros_gazebo_v1/balls.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int mode = 2; //1: HoughCircles, 2: findContours
bool continueSubscription = 0;
//NOTA: Ajustar nẃ de iteraciones de filtros erode y dilate según el tamaño de las bolas

//ros::Publisher balls_pub;
test_image_transport::balls ballsMsg;

//ros::MultiThreadedSpinner spinner;

using namespace std;
using namespace cv;

namespace gazebo {
	class central_station : public ModelPlugin {
		
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
			this->model = _parent;
			if (!ros::isInitialized()){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv,"central_station_node",ros::init_options::NoSigintHandler);
			}

			ROS_INFO("Hello World!");

			//image_transport::SubscribeOptions image_drone = image_transport::SubscribeOptions::create<image_transport::ImageTransport>
			//	("/image_drone",1,boost::bind(&central_station::imageCallback, this, _1),ros::VoidPtr(), &this->rosQueue);
			image_transport::ImageTransport it(this->central_station_node);
			this->sub = it.subscribe("/image_drone",1,boost::bind(&central_station::imageCallback, this, _1));
			this->pub = this->central_station_node.advertise<std_msgs::Float32>("/check",10);
			this->balls_pub = this->central_station_node.advertise<test_image_transport::balls>("balls", 1);
			//spinner.spin();
			this->rosQueueThread = std::thread(std::bind(&central_station::QueueThread, this));
		}
		
		//public: void callbackImage(const std_msgs::Float32ConstPtr &_msg){
		//	float info = 0.0;
		//	ROS_INFO("Receiving image data");
		//	std_msgs::Float32 msg;
		//	info=_msg->data;
		//	msg.data = info;
		//	this->pub.publish(msg);
		//}

		public: void imageCallback(const sensor_msgs::ImageConstPtr& msg){
			try
			{
				Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
				Mat mask;

				imshow("1. Imagen inicial", image);
				mask = circleBalls(image);
				//imshow("circleBalls", mask);
				waitKey(1000);
				cout<<"Listen again? (0/1): ";
				cin>>continueSubscription;
				if(!continueSubscription) ros::shutdown();
		  	}
			catch (cv_bridge::Exception& e)
			{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
			}
		}

		public: Mat circleBalls(Mat image){
			Mat mask;
			cvtColor(image, mask, COLOR_BGR2HSV);
			GaussianBlur(mask, mask, Size(7, 7), 1.5, 1.5);
			inRange(mask, Scalar(55 / 2.0, 0.2 * 256, 0.6 * 256, 0), Scalar(72 / 2.0, 1 * 256, 1 * 256, 0), mask); //Máscara 8
			imshow("2. Máscara", mask);
			erode(mask, mask, Mat(), Point(-1, -1), 1);
			imshow("3. Máscara con filtro erode", mask);
			dilate(mask, mask, Mat(), Point(-1, -1), 4);
			imshow("4. Máscara con filtro erode y dilate", mask);

	
			//Mat se21 = getStructuringElement(MORPH_RECT, Size(21, 21));
			//morphologyEx(mask, mask, MORPH_CLOSE, se21);
			//imshow("5. Máscara con filtro de ruido background", mask);

			if (mode == 1) {	//HoughCircles
				// Convert mask into a grayscale image
				GaussianBlur(mask, mask, Size(15, 15), 0, 0);
				imshow("5-2. Máscara con filtro grayscale", mask);

				// Run the Hough function
				vector<Vec3f> circles;
				HoughCircles(mask, circles, HOUGH_GRADIENT, 2, mask.rows / 10, 100, 40, 0, 0);
				cout << circles.size()<<" balls found.\t"; //Deben ser 2 circulos
				ballsMsg.ballsNumber = circles.size();
				geometry_msgs::Point point;

				cvtColor(mask, mask, COLOR_GRAY2RGB);

				//Mostrar círculos
				for (size_t i = 0; i < circles.size(); i++){
				//Crear mensaje ROS
				point.x=cvRound(circles[i][0]);
				point.y=cvRound(circles[i][1]);
				point.z=0;
				ballsMsg.points.push_back(point);

				//Dibujar círculos en imágenes
				Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				int radius = cvRound(circles[i][2]);
				// circle center
				circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
				circle(mask, center, 3, Scalar(0, 255, 0), -1, 8, 0);
				// circle outline
				circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
				circle(mask, center, radius, Scalar(0, 0, 255), 3, 8, 0);
				}
			}
			else if (mode == 2) { //findContours
				// Run the findContours function
				vector<vector<Point>> contours;
				findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				cout<<contours.size()<<" balls found.\t"; //Número de pelotas encontradas
				ballsMsg.ballsNumber = contours.size();
				geometry_msgs::Point point;

				cvtColor(mask, mask, COLOR_GRAY2RGB);

				//Mostrar círculos
				for (size_t i = 0; i < contours.size(); i++){
					// Find the minimum area enclosing circle
					Point2f center;
					float radius = 0;
					minEnclosingCircle(contours[i], center, radius);

					//Crear mensaje ROS
					point.x=center.x;
					point.y=center.y;
					point.z=0;
					ballsMsg.points.push_back(point);

					// circle center
					circle(image, center, 2, Scalar(0, 255, 0), -1, 8, 0);
					circle(mask, center, 2, Scalar(0, 255, 0), -1, 8, 0);
					// circle outline
					circle(image, center, radius, Scalar(0, 0, 255), 2, 8, 0);
					circle(mask, center, radius, Scalar(0, 0, 255), 2, 8, 0);
				}
			}

			imshow("6. Resultado final", mask);
			imshow("7. Resultado final", image);
			imwrite("tutorial8-"+ to_string(mode)+".jpg", image);

			std::cout<<"Balls positions list:"<<std::endl;
			for(int i=0; i<(int)ballsMsg.ballsNumber; i++){
				std::cout<<"("<<ballsMsg.points[i].x<<", "<<ballsMsg.points[i].y<<")"<<std::endl;
			}
			std::cout<<std::endl;

			//balls_pub.publish(ballsMsg);

			return image;
		}


		//-7 \brief ROS helper function that processes messages
		private: void QueueThread(){
			static const double timeout = 0.01;
			while (this->central_station_node.ok()){
				ros::spinOnce();
				//this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		//public: void OnUpdate(){
		//	this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		//}

		// Pointer to the model
		private: physics::ModelPtr model;
		private: ros::NodeHandle central_station_node;
		private: image_transport::Subscriber sub;
		private: ros::Publisher pub;
		private: ros::Publisher balls_pub;
		//private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;

		// Pointer to the update event connection
		// private: event::ConnectionPtr updateConnection;
	};

GZ_REGISTER_MODEL_PLUGIN(central_station)
}

