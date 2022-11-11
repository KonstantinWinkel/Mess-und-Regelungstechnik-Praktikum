#include<ros/ros.h>
#include<signal.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<fstream>
#include<iostream> 
#include<sstream>
#include<tf/transform_datatypes.h>

//Needed variables
std::ostringstream oss;
std::string parameter;
std::string amclmode = "undefined";
bool amclRel = false;
geometry_msgs::Point initPoint;
bool firstCycle = true;
double phiInit;
int i = 0;

//service method for odometry
void chatterCallbackOdom(const nav_msgs::Odometry msg){
    geometry_msgs::Point p = msg.pose.pose.position;
    oss << p.x << ' ' << p.y << std::endl;
    i++;
}

//service method for amcl
void chatterCallbackAmcl(const geometry_msgs::PoseWithCovarianceStamped msg){
    
    if (amclRel) {
        if (firstCycle) {
            initPoint = msg.pose.pose.position;
			tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
			
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			phiInit = yaw;
				
            firstCycle = false;
        }

        geometry_msgs::Point p = msg.pose.pose.position;
        p.x -= initPoint.x;
        p.y -= initPoint.y;
        p.z -= initPoint.z;
		double x, y; 
		
	x = p.x*cos(-phiInit) - p.y*sin(-phiInit);
	y = p.x*sin(-phiInit) + p.y*cos(-phiInit);	
			
        oss << x << ' ' << y << std::endl;
    }
    else {
        geometry_msgs::Point p = msg.pose.pose.position;
        oss << p.x << ' ' << p.y << std::endl;
    }   
    i++;
}

//custom Shutdown method to ensure all the data is correctly
//writen to the corosponding file :)
void shutdownHandler(int sig){
    std::cout << parameter << std::endl;
    std::ofstream odometry_file;
    odometry_file.open("lastPath.dat");
    odometry_file << i << std::endl;
    odometry_file << oss.str();
    odometry_file.close();	
    ros::shutdown();
}

int main(int argc, char **argv){

    //initialisations
    ros::init(argc, argv, "LocalizationListener", ros::init_options::NoSigintHandler);
    ros::NodeHandle n("~");
    signal(SIGINT, shutdownHandler);
    
    //set parameter for either amcl or odom
    n.getParam("mode", parameter);
    n.getParam("amclmode", amclmode);
    
    std::cout << parameter << std::endl;

    
    ros::Subscriber Odom_sub;
    ros::Subscriber Amcl_sub;
    //select service depending on given parameter or throw error
    if(parameter == "odom") Odom_sub = n.subscribe("/odom", 1 , chatterCallbackOdom);
    else if (parameter == "amcl") {
        Amcl_sub = n.subscribe("/amcl_pose", 1, chatterCallbackAmcl);
        if (amclmode == "abs") amclRel = false;
        else if (amclmode == "rel") amclRel = true;
        else ROS_ERROR("INVALD PARAMETER - amclmode must be either abs or rel");
    }
    else ROS_ERROR("INVALID PARAMETER - mode must be either odom or amcl");
    
    //spin it to win it
    ros::spin();
}
