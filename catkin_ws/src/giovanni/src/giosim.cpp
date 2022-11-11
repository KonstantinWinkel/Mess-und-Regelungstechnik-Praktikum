#include <ros/ros.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include "gio_path.cpp"

std::ofstream controller_file;

int main(int argc, char **argv){
	ros::init(argc, argv, "GioSim");
	ros::NodeHandle n("~");
	ros::Rate r(10);
	
	std::string check;
	std::string filePath;
	n.getParam("param", check);
	n.getParam("path", filePath);
	
	if(filePath == ""){
		ROS_ERROR("NO FILEPATH GIVEN");
		return -1;
	}
	
	controller_file.open("giosim.csv");
	
	CGioController gio;
	gio.getPathFromFile(filePath.c_str());
	
	double u, w, vleft, vright, looped;
	u = w = vleft = vright = looped = 0;
	
	double x, y, phi, dphi;
	gio.getPose(x, y, phi);
	
	double deltaT;
	deltaT = 0.1;
	//t = ros::Time::now().toSec();
	
	int res = 1;
	double delx,dely;
	cout << x << "," << y << "," << phi << "," << w << std::endl;
	//for(int i = 0;i<10;i++){
	while(ros::ok()){
		if(!gio.getNextState(u, w, vleft, vright, looped)) break;
	
		gio.getPose(x, y, phi);
	
		dphi = w * deltaT;
	
		double l = deltaT * u;
		if(fabs(dphi) <= 0.02){
			double l = deltaT * u;
			delx = l*cos(dphi);
			dely = l*sin(dphi);    
		}
		else{
			dphi = w * deltaT;
			double r = u/w;
			dely = r*cos(dphi) - r;
			delx = r*sin(dphi);
		}
		x += delx*cos(phi) - dely*sin(phi);
		y += delx*sin(phi) + dely*cos(phi);
		phi+= dphi;
		NormalizeAngle(phi);
		gio.setPose(x, y, phi);
		
		controller_file << x << "," << y << "," << phi << "," << w  << std::endl;

		cout << x << "," << y << "," << phi << "," << w << std::endl;
		ros::spinOnce();

	}
	
	controller_file.close();
	
}
