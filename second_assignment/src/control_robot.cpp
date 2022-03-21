#include "ros/ros.h"
#include "geometry_msgs/Twist.h" //header for publishing on cmd_vel
#include "sensor_msgs/LaserScan.h" //header for subscribing to /base_scan
#include <iostream>
#include "second_assignment/UserInterface.h"
#include "std_srvs/Empty.h"

ros::Publisher pub_vel; //publisher for velocities

float custom_acc = 0.0; //custom acceleration given by user inputs

std_srvs::Empty pose_res; //server to reset position

double d_th = 1.5; //treshold distance

double minimum(int first_index, int last_index, double array[]);




/*******************************************************************************************
* MINIMUM(int FIST_INDEX, int LAST_INDEX, double ARRAY[]):                                 *
* This function provides the closest obstacles in the subsection given by the choice of    *
* the angle in which the robot sees. The range is defined by FIRST_INDEX and LAST_INDEX.   *
* The ARRAY[] input is the vector in which are stored all sensor_msgs/LaserScan data.      *
* The return value is the minimum distance from the walls                                  *
********************************************************************************************/

double minimum(int first_index, int last_index, float array[]){

	//as the minimum distance is set a large value that will then be compared
	//with all the values between the first value and the last one in the ranges array
	double min_dist = 35;
	
	for(int i = first_index; i<= last_index; i++){
		if(array[i]<=min_dist){
			min_dist = array[i];	
		}
		
	}
	return min_dist;
}

bool UI_Callback (second_assignment::UserInterface::Request &req, second_assignment::UserInterface::Response &res){

	// if the request sent from the user is 'i' the velocity is incremented
	if(req.command == 'i'){
		custom_acc += 0.5;
		
		//ROS_INFO("\nIncrement of acceleration :@[%f]", vel.linear.x);
	}
	//if the request sent from the user is 'd' the velocity is decreased
	if(req.command == 'd'){
		custom_acc -= 0.5;
		
		
		//ROS_INFO("\nIncrement of acceleration :@[%f]", vel.linear.x);
			
	}
	//if the request sent from the user is 'r' robot goes back to the initial position
	if(req.command == 'r'){
		ros::service::call("/reset_positions", pose_res);
	}
	
	if(req.command != 'i' && req.command != 'd' && req.command != 'r'){
		std::cout<<"NOT SUPPORTED\n"<<std::endl;
		fflush(stdout);
	}
	 
	res.value = custom_acc;
	return true;
}

void Driver(const sensor_msgs::LaserScan::ConstPtr& msg){	
	
	geometry_msgs::Twist vel; //message to publish
	
	float info_distance[721]; //array used for storing ranges[]
	
	for(int i=0; i<=721; i++){
		info_distance[i] = msg->ranges[i];
	}
	
	double front, right, left; //minimum distances on the three directions
	//right measure from 0° to 30°
	right = minimum(0, 120, info_distance);
	//front measure from 75° to 105°
	front = minimum(300, 420, info_distance);
	//left measure from 150° to 180°
	left = minimum(600, 720, info_distance);
	
	//ROS_INFO("subscriber[%f %f %f]", right, front, left);
	
	//if there is a curve to do
	if(front<d_th){
		//if right wall is near wrt left wall turn right
		if(right<left){
			//turn_right();
			vel.linear.x = 0.5;
			vel.angular.z = 1;
		}
		//if the opposite turn left
		else if(right>left){
			//turn_left();
			vel.linear.x = 0.5;
			vel.angular.z = -1;
		}
	
	}
	//if no curve to do
	else{
		vel.linear.x = 1.0 + custom_acc;
  		vel.angular.z = 0.0;
  		if(vel.linear.x<=0){
  			vel.linear.x = 0.0;
  		}
	}
	
	pub_vel.publish(vel);

}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle n;
	
	//service to change velocities
	ros::ServiceServer service =  n.advertiseService("/userinterface", UI_Callback);
	//initialize publisher to cmd_vel with msg Twist
	pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//subscribe to base_scan to retrieve measurements
	ros::Subscriber sub = n.subscribe("/base_scan", 1, Driver);

	
	
	ros::spin();
	return 0;
}


