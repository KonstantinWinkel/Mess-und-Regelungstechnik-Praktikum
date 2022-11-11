#include <ros/ros.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include "gio_path.cpp"
#include <iostream>
#include <sstream>
#include <signal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "velocities.h"
#include "vels.h"
#include "tf/transform_datatypes.h"

std::ofstream controller_file;

std::string mode;

double u, w, vleft, vright, looped, x, y, phi, dphi, T, deltaT, res;
CGioController gio;
ros::Publisher publisher;
int factor = -30;

int deltaTime = 5;
double lastStop = -1;

geometry_msgs::Point amclInit;
geometry_msgs::Point odomInit;
double phiInit;
double odomPhiInit;
bool amclFirst = true;
bool absCoord;

void sendVels(){
	volksbot::vels velocity;
	velocity.left = factor * vleft;
	velocity.right = factor * vright;
	
	publisher.publish(velocity);
}

void controller(const geometry_msgs::Pose p){

	/*
	if(lastStop == -1) lastStop = ros::Time::now().toSec();
	if(ros::Time::now().toSec() - lastStop > deltaTime ){
		vleft = 0;
		vright = 0;
		sendVels();
		double T = ros::Time::now().toSec();
		//while(ros::Time::now().toSec() - T <= 20){
		//hello my name is jeff
		//}
		
		while(true){
		if(std::cin.get() == '\n') {
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cin.clear();
			break;
		}
		}
		
		lastStop = ros::Time::now().toSec();
	}
	*/
	
	deltaT = ros::Time::now().toSec() - T;
	T = ros::Time::now().toSec();
		
	tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	/*
	if(!absCoord){
		x = p.position.x*cos(-phiInit) - p.position.y*sin(-phiInit);
		y = p.position.x*sin(-phiInit) + p.position.y*cos(-phiInit);
	}
	else{
		x = p.position.x;
		y = p.position.y;
	}
	*/
	
	x = p.position.x*cos(-phiInit) - p.position.y*sin(-phiInit);
	y = p.position.x*sin(-phiInit) + p.position.y*cos(-phiInit);
	
	
	double roll, pitch, yaw;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	phi = yaw - phiInit;
	NormalizeAngle(phi);
	
	gio.setPose(x, y, phi);


	if(!gio.getNextState(u, w, vleft, vright, looped)){
		vleft = 0;
		vright = 0;
		sendVels();	
		return;
	}


	
	controller_file << x << "," << y << "," << phi << "," << w  << std::endl;
	sendVels();
}

void OdomCallback(const nav_msgs::Odometry msg){
	geometry_msgs::Pose p = msg.pose.pose;
	controller(p);
}

void AMCLCallback(const geometry_msgs::PoseWithCovarianceStamped msg){
	geometry_msgs::Pose p = msg.pose.pose;
	
	if(!absCoord){
		
		if(amclFirst){
			tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			amclInit = p.position;
			amclFirst = false;
			phiInit = yaw;
			//return;
		}
	
		p.position.x -= amclInit.x;
		p.position.y -= amclInit.y;
		p.position.z -= amclInit.z;
	}
	
	gio.setPose(p.position.x, p.position.y, -phiInit);
	controller(p);

}

void shutdownHandler(int sig){


	controller_file.close();
	ros::shutdown();
}


int main(int argc, char **argv){
	ros::init(argc, argv, "gioCon",ros::init_options::NoSigintHandler);
	ros::NodeHandle n("~");
	signal(SIGINT, shutdownHandler);
	
	//std::string mode;
	std::string filePath;
	std::string abs = "platzhalter";
	absCoord = false;
	n.getParam("mode", mode);
	n.getParam("path", filePath);
	n.getParam("abs", abs);
	publisher = n.advertise<volksbot::vels>("/Vel", 100);
	
	
	absCoord = (abs == "y");
	
	ros::Subscriber Odom_sub; 
	ros::Subscriber Amcl_sub;
		
	if(mode != "odom" && mode != "amcl"){
		ROS_ERROR("INVLAID MODE - MUST BE EITHER odom OR amcl");
		return -2;
	}
	
	if(mode == "odom") Odom_sub = n.subscribe("/odom", 1 , OdomCallback);
	if(mode == "amcl") Amcl_sub = n.subscribe("/amcl_pose", 1 , AMCLCallback);
	
	if(filePath == ""){
		ROS_ERROR("NO FILEPATH GIVEN");
		return -1;
	}
	
	controller_file.open("giocontroller.csv");
	
	cout << "hello World" << endl;
	cout << "Parameter Mode: " << mode << endl;
	cout << "Parameter Path: " << filePath << endl;
	cout << "Parameter Abs: " << absCoord << endl;
	
	gio.getPathFromFile(filePath.c_str());
	
	//u = w = vleft = vright = looped = 0;
	
	gio.getPose(x, y, phi);
	
	T = ros::Time::now().toSec();
	
	res = 1;
	
	ros::spin();
}
