#include<ros/ros.h>
#include<signal.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<fstream>
#include<iostream> 
#include<sstream>

//Needed variables
std::ostringstream oss;
std::string parameter;

//service method for odometry
void chatterCallbackOdom(const nav_msgs::Odometry msg){
    geometry_msgs::Point p = msg.pose.pose.position;
    oss << p.x << ',' << p.y << ',' << p.z << std::endl;
}

//service method for amcl
void chatterCallbackAmcl(const geometry_msgs::PoseWithCovarianceStamped msg){
    geometry_msgs::Point p = msg.pose.pose.position;
    oss << p.x << ',' << p.y << ',' << p.z << std::endl;
}

//custom Shutdown method to ensure all the data is correctly
//writen to the corosponding file :)
void shutdownHandler(int sig){
    std::cout << parameter << std::endl;
    if(parameter == "odom"){
	std::ofstream odometry_file;
	odometry_file.open("odometryData.csv");
    	odometry_file << oss.str();
    	odometry_file.close();	
    }
    if(parameter == "amcl") {
	std::ofstream amcl_file;
	amcl_file.open("amclData.csv");
    	amcl_file << oss.str();
    	amcl_file.close();
    
    }
    ros::shutdown();
}

int main(int argc, char **argv){

    //initialisations
    ros::init(argc, argv, "LocalizationListener", ros::init_options::NoSigintHandler);
    ros::NodeHandle n("~");
    signal(SIGINT, shutdownHandler);
    
    //set parameter for either amcl or odom
    n.getParam("mode", parameter);
    
    ros::Subscriber Odom_sub;
    ros::Subscriber Amcl_sub;
    //select service depending on given parameter or throw error
    if(parameter == "odom") Odom_sub = n.subscribe("/odom", 1 , chatterCallbackOdom);
    if(parameter == "amcl") Amcl_sub = n.subscribe("/amcl_pose", 1 , chatterCallbackAmcl);
    
    //if no valid parameter go errrrror
    if(parameter != "odom" && parameter != "amcl") ROS_ERROR("INVALID PARAMETER - must be either odom or amcl");
    
    //spin it to win it
    ros::spin();
}
