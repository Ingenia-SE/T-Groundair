#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <functional>
#include <thread>
#include <iostream>
#include <math.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "ros_gazebo_v1/balls.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

int mode = 2; //1: HoughCircles, 2: findContours
int sent_points = 0;
int sent_target_ugv = 0;
int flag_image = 0;
float home_x_ugv = 0.0;
float home_y_ugv = 0.0;
bool continueSubscription = 0;
//NOTA: Ajustar nẃ de iteraciones de filtros erode y dilate según el tamaño de las bolas

//ros::Publisher balls_pub;
test_image_transport::balls ballsMsg;
geometry_msgs::PoseStamped msg_point;
geometry_msgs::Point msg_ugv;
	 
float point1 []= {10, -10 ,1, 0, 20};
float point2 []= {0, -10, 20, 0 ,20};
float point3 []= {-10, -10, 10, 0, 20};
float point4 []= {0, 0, 0, 0, 0};

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

			ROS_INFO("Hello World central!");
			
			//image_transport::SubscribeOptions image_drone = image_transport::SubscribeOptions::create<image_transport::ImageTransport>
			//	("/image_drone",1,boost::bind(&central_station::imageCallback, this, _1),ros::VoidPtr(), &this->rosQueue);

			ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Int16>("/interface",10,
      				boost::bind(&central_station::interfaceCheck, this, _1),ros::VoidPtr(), &this->rosQueue);
			ros::SubscribeOptions so3 = ros::SubscribeOptions::create<std_msgs::Int16>("/drone_check",10,
      				boost::bind(&central_station::droneCheck, this, _1),ros::VoidPtr(), &this->rosQueue);
			ros::SubscribeOptions so4 = ros::SubscribeOptions::create<std_msgs::Int16>("/ugv_check",10,
      				boost::bind(&central_station::ugvCheck, this, _1),ros::VoidPtr(), &this->rosQueue);
			image_transport::ImageTransport it(this->central_station_node);

			this->sub = it.subscribe("/image_drone",1,boost::bind(&central_station::imageCallback, this, _1));
			this->sub2 = this->central_station_node.subscribe(so2);
			this->sub3 = this->central_station_node.subscribe(so3);
			this->sub4 = this->central_station_node.subscribe(so4);			

			//this->pub = this->central_station_node.advertise<std_msgs::Float32>("/check",10);
			this->pub_interface = this->central_station_node.advertise<std_msgs::Int16>("/central_ok",10);
			this->waypoints_pub = this->central_station_node.advertise<geometry_msgs::PoseStamped>("target_waypoints", 100);
			this->ugv_pub = this->central_station_node.advertise<geometry_msgs::Point>("/ugv_waypoints", 10);
			//this->balls_pub = this->central_station_node.advertise<test_image_transport::balls>("balls", 1);
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
			std_msgs::Float32 message;
			std_msgs::Int16 message_2;
			float info = 0.0;
			int info_2 = 1;
		
			try
			{
				Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
				Mat mask;
				imshow("1. Imagen inicial", image);
				mask = circleBalls(image);
				waitKey(1000);
				message.data = info;
				message_2.data = info_2;
				//this->pub.publish(message);
				//this->pub_interface.publish(message_2);
				
				//imshow("circleBalls", mask);
				//cout<<"Listen again? (0/1): ";
				//cin>>continueSubscription;
				//if(!continueSubscription) ros::shutdown();
		  	}
			catch (cv_bridge::Exception& e)
			{
			info=1.0;
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
			//this->pub.publish(message);
			}
		}

		public: void interfaceCheck(const std_msgs::Int16ConstPtr& msg){
			
			msg_point.pose.position.x = point1[0];
			msg_point.pose.position.y = point1[1];
	 		msg_point.pose.position.z = point1[2];
	 		msg_point.pose.orientation.x = point1[4];
	 		msg_point.pose.orientation.z = point1[3];

			sent_points = 1;
			this->waypoints_pub.publish(msg_point);

		}

		public: void droneCheck(const std_msgs::Int16ConstPtr& msg){
			
			if (msg->data == 1){	
				ROS_INFO("Position received");
			}

			if (msg->data == 2){
				if ((sent_points == 1) || (sent_points == 3)){
					msg_point.pose.position.x = point2[0];
					msg_point.pose.position.y = point2[1];
		 			msg_point.pose.position.z = point2[2];
		 			msg_point.pose.orientation.x = point2[4];
		 			msg_point.pose.orientation.z = point2[3];
					this->waypoints_pub.publish(msg_point);
				}

				if (sent_points == 2){
					msg_point.pose.position.x = point3[0];
					msg_point.pose.position.y = point3[1];
		 			msg_point.pose.position.z = point3[2];
		 			msg_point.pose.orientation.x = point3[4];
		 			msg_point.pose.orientation.z = point3[3];
					this->waypoints_pub.publish(msg_point);
					while (flag_image == 0){};
					msg_ugv.x = (float)ballsMsg.points[0].x;
					msg_ugv.y = (float)ballsMsg.points[0].y;
					flag_image = 0;
				}

				if (sent_points == 4){
					msg_point.pose.position.x = point1[0];
					msg_point.pose.position.y = point1[1];
		 			msg_point.pose.position.z = point1[2];
		 			msg_point.pose.orientation.x = point1[4];
		 			msg_point.pose.orientation.z = point1[3];
					this->waypoints_pub.publish(msg_point);
				}

				if (sent_points == 5){
					msg_point.pose.position.x = point4[0];
					msg_point.pose.position.y = point4[1];
		 			msg_point.pose.position.z = point4[2];
		 			msg_point.pose.orientation.x = point4[4];
		 			msg_point.pose.orientation.z = point4[3];
					this->waypoints_pub.publish(msg_point);
				}

				if (sent_points == 6){
					this->ugv_pub.publish(msg_ugv);
					sent_target_ugv = sent_target_ugv + 1;
					sent_points = -1;
				}

				sent_points = sent_points + 1;
			}

		}

		public: void ugvCheck(const std_msgs::Int16ConstPtr& msg){
			
			if (msg->data == 1){	
				ROS_INFO("Position received - UGV");
			}

			if (msg->data == 2){
				if (sent_target_ugv < (int)ballsMsg.ballsNumber){
					msg_ugv.x = ballsMsg.points[sent_target_ugv].x;
					msg_ugv.y = ballsMsg.points[sent_target_ugv].y;
					this->ugv_pub.publish(msg_ugv);
					sent_target_ugv = sent_target_ugv + 1;
				}

				else {
					msg_ugv.x = home_x_ugv;
					msg_ugv.y = home_y_ugv;
				
					int info = 1;
					std_msgs::Int16 msg_ok;
					msg_ok.data = info;
					this->pub_interface.publish(msg_ok);
					sent_target_ugv = 0;
				}

			}

		}

		public: Mat circleBalls(Mat image){
			Mat mask, azul, rojo;
			double conversion; //para convertir la distancia en la imagen a distancia real
			double distanciareal = 13.5; //distancia real entre los dos puntos de control. es el ancho de la pista de tenis en metros
			cvtColor(image, mask, COLOR_BGR2HSV);
			GaussianBlur(mask, mask, Size(7, 7), 1.5, 1.5);

				//cuadrado azul
				inRange(mask, Scalar(230 / 2.0, 0.7 * 256, 0.6 * 256, 0), Scalar(250 / 2.0, 1 * 256, 1 * 256, 0), azul);
				//imshow("1. azul", azul);
				erode(azul, azul, Mat(), Point(-1, -1), 1);
				dilate(azul, azul, Mat(), Point(-1, -1), 4);
				vector<vector<Point>> puntoazul;
				findContours(azul, puntoazul, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				cvtColor(azul, azul, COLOR_GRAY2RGB);
				Point2f centroazul;
				float radioazul = 0;
				minEnclosingCircle(puntoazul[0], centroazul, radioazul);
				// circle center
				circle(image, centroazul, 2, Scalar(0, 0, 0), -1, 8, 0);
				// circle outline
				circle(image, centroazul, radioazul, Scalar(0, 0, 0), 2, 8, 0);

				//cuadrado rojo
				inRange(mask, Scalar(0 / 2.0, 0.5 * 256, 0.4 * 256, 0), Scalar(15 / 2.0, 1 * 256, 1 * 256, 0), rojo);
				//imshow("1. rojo", rojo);
				erode(rojo, rojo, Mat(), Point(-1, -1), 1);
				dilate(rojo, rojo, Mat(), Point(-1, -1), 4);
				vector<vector<Point>> puntorojo;
				findContours(rojo, puntorojo, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				cvtColor(rojo, rojo, COLOR_GRAY2RGB);
				Point2f centrorojo;
				float radiorojo = 0;
				minEnclosingCircle(puntorojo[0], centrorojo, radiorojo);
				// circle center
				circle(image, centrorojo, 2, Scalar(0, 0, 0), -1, 8, 0);
				// circle outline
				circle(image, centrorojo, radiorojo, Scalar(0, 0, 0), 2, 8, 0);

				//el factor de conversion es la distancia real que le corresponderia a una u.m. en la imagen
				conversion = distanciareal / sqrt( pow( (centroazul.x - centrorojo.x), 2) + pow ( ( centroazul.y - centrorojo.y), 2) );
				//se convierte el centro del cuadrado azul de distancia de la imagen a distancia real
				centroazul.x = conversion * centroazul.x;
				centroazul.y = conversion * centroazul.y;


			//Pelotas
			inRange(mask, Scalar(40 / 2.0, 0.45 * 256, 0.40 * 256, 0), Scalar(115 / 2.0, 0.75 * 256, 0.85 * 256, 0), mask);
			//imshow("2. Máscara", mask);
			erode(mask, mask, Mat(), Point(-1, -1), 1);
			//imshow("3. Máscara con filtro erode", mask);
			dilate(mask, mask, Mat(), Point(-1, -1), 2);
			//imshow("4. Máscara con filtro erode y dilate", mask);

				// Run the findContours function
				vector<vector<Point>> contours;
				findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				cout<<contours.size()<<" balls found.\t"; //Número de pelotas encontradas
				ballsMsg.ballsNumber = contours.size();
				geometry_msgs::Point point;

				cvtColor(mask, mask, COLOR_GRAY2RGB);

				//Mostrar círculos
				for (size_t i = 0; i < contours.size(); i++)
				{
				// Find the minimum area enclosing circle
				Point2f center;
				float radius = 0;
				minEnclosingCircle(contours[i], center, radius);

				
				// circle outline
				circle(image, center, radius, Scalar(0, 255, 0), 1, 8, 0);
				circle(mask, center, radius, Scalar(0, 255, 0), 1, 8, 0);
				// circle center
				circle(image, center, 2, Scalar(0, 0, 0), -1, 8, 0);
				circle(mask, center, 2, Scalar(0, 0, 0), -1, 8, 0);

				//se convierten las posiciones de los centros de las pelotas a coordenadas reales
				center.x = conversion * center.x;
				center.y = conversion * center.y; 

				//Crear mensaje ROS
				point.x=center.x;
				point.y=center.y;
				point.z=0;
				ballsMsg.points.push_back(point);
				}

			//imshow("6. Resultado final", mask);
			imshow("7. Resultado final", image);
			imwrite("final-"+ to_string(1)+".jpg", image);

			std::cout<<"Balls positions list:"<<std::endl;
			for(int i=0; i<(int)ballsMsg.ballsNumber; i++){
				std::cout<<"("<<ballsMsg.points[i].x<<", "<<ballsMsg.points[i].y<<")"<<std::endl;
			}
			std::cout<<std::endl;

			ordenar(home_x_ugv, home_y_ugv);

			std::cout<<"Balls positions ordered list:"<<std::endl;
			for(int i=0; i<(int)ballsMsg.ballsNumber; i++){
				std::cout<<"("<<ballsMsg.points[i].x<<", "<<ballsMsg.points[i].y<<")"<<std::endl;
			}
			std::cout<<std::endl;

			//balls_pub.publish(ballsMsg);
		
			flag_image = 1;

			return image;
		}



		public: void ordenar (float x_ugv, float y_ugv){
     			
			float x_distancia_ugv_bola[ballsMsg.ballsNumber], y_distancia_ugv_bola[ballsMsg.ballsNumber];
     			float modulo_distancia_ugv_bola[ballsMsg.ballsNumber], modulo_distancia_bola_bola[ballsMsg.ballsNumber];
	 		float aux1, aux2, aux3;
			int cuenta = 0;
			int cuenta_2 = 1;

			for (int k=0; k<(int)ballsMsg.ballsNumber; k++){
	    			for(int i=cuenta; i<(int)ballsMsg.ballsNumber; i++){
					x_distancia_ugv_bola[i]=ballsMsg.points[i].x - x_ugv;
					y_distancia_ugv_bola[i]=ballsMsg.points[i].y - y_ugv;
					modulo_distancia_ugv_bola[i]=sqrt(pow(x_distancia_ugv_bola[i],2) + pow(y_distancia_ugv_bola[i],2));
	    			}		

				/* Ordenamos las distancias hasta el ugv y reordenamos la matriz de posicion bolas*/
				for(int i=cuenta; i<(int)ballsMsg.ballsNumber-1; i++){ 
					for(int j=cuenta_2; j<(int)ballsMsg.ballsNumber; j++){
						if (modulo_distancia_ugv_bola[j] < modulo_distancia_ugv_bola[i]) 
						{ 
							/* Cambiamos el orden del vector de distancias*/
							aux1 = modulo_distancia_ugv_bola[j]; 
							modulo_distancia_ugv_bola[j] = modulo_distancia_ugv_bola[i]; 
							modulo_distancia_ugv_bola[i] = aux1; 
							/* Cambiamos la componente x de la matriz de posicion bolas */
							aux2 = ballsMsg.points[j].x; 
							ballsMsg.points[j].x = ballsMsg.points[i].x; 
							ballsMsg.points[i].x = aux2;
							x_ugv = ballsMsg.points[i].x;
							/* Cambiamos la componente y de la matriz de posicion bolas */
							aux3 = ballsMsg.points[j].y; 
							ballsMsg.points[j].y = ballsMsg.points[i].y; 
							ballsMsg.points[i].y = aux3;  
							y_ugv = ballsMsg.points[i].y;
						}
					} 
				} 
				
				cuenta = cuenta + 1;
				cuenta_2 = cuenta_2 + 1;
			}    

		/* Ahora la primera fila de la matriz hace referencia a la bola mas cercana al ugv al inicio, pero
		cuando se recoja esa primera bola cambia la posicion inicial del ugv y puede ser que haya otra bola
		que este mas cercana ahora del ugv, por lo que se tiene que hacer un nuevo orden*/

		/*	for(i=1; i<ballsNumber-1; i++){
				for(j=2; j<ballsNumber; i++){	
					modulo_distancia_bola_bola[i]=sqrt(pow(points[i].x - posicion_bolas[i--][0]) + pow(points[i].y - posicion_bolas[i--][1]));
					modulo_distancia_bola_bola[j]=sqrt(pow(points[j].x - posicion_bolas[i--][0]) + pow(points[j].y - posicion_bolas[i--][1]));
					if (modulo_distancia_bola_bola[j] < modulo_distancia_bola_bola[i]) 
					{  
						//Cambiamos la componente x de la matriz de posicion bolas
						aux2 = points[j].x; 
				 		points[j].x = points[i].x; 
						points[i].x = aux2;
						//Cambiamos la componente y de la matriz de posicion bolas
						aux3 = points[j].y; 
				 		points[j].y = points[i].y; 
						points[i].y = aux3;  
					}
				}
			}
		*/

			/* Lo que se consigue es que la matriz posicion_bolas[][] esté ordenada de tal forma que el ugv
			recogera cada vez la bola que tenga mas cercana en linea recta */ 
		}

		//-7 \brief ROS helper function that processes messages
		private: void QueueThread(){
			static const double timeout = 0.01;
			while (this->central_station_node.ok()){
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
				ros::spinOnce();
			}
		}

		//public: void OnUpdate(){
		//	this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		//}

		// Pointer to the model
		private: physics::ModelPtr model;
		private: ros::NodeHandle central_station_node;
		private: image_transport::Subscriber sub;
		private: ros::Subscriber sub2;
		private: ros::Subscriber sub3;
		private: ros::Subscriber sub4;
		//private: ros::Publisher pub;
		private: ros::Publisher pub_interface; 
		private: ros::Publisher waypoints_pub; 
		private: ros::Publisher ugv_pub; 
		
		//private: ros::Publisher balls_pub;
		private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;

		// Pointer to the update event connection
		// private: event::ConnectionPtr updateConnection;
	};

GZ_REGISTER_MODEL_PLUGIN(central_station)
}

