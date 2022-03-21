#include "ros/ros.h"
#include "second_assignment/UserInterface.h" //header of UserInterface srv
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"

ros::ServiceClient client_UI; 

/***********************************************************************
* GET_COMMAND():                                                       *
* Function that prints the rules for entering the possible user's      *
* commands: <i> to increase velocity, <d< to decrease, <r> to reset    *
* the position.                                                        *
* The return value is the char pressed by the user.                    *
***********************************************************************/


char Get_command(void){
	char comm;
	
	std::cout<<"Welcome player!\n";
	std::cout<<"If you want to increase the speed press <i>\n";
	std::cout<<"If you want to decrease the speed press <d>\n";
	std::cout<<"If you want to reset the position press <r>\n";
	std::cout<<"Please select: ";
	std::cin>>comm;
	
	return comm;
}

//callback function to send the request to change the velocity and reset the position
void CommandCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	second_assignment::UserInterface srv;
	
	//call at the function Get_command() wich returns the char given by the user
	char input = Get_command();
	
	srv.request.command = input;
	
	client_UI.waitForExistence();
	client_UI.call(srv);
}

int main(int argc, char **argv){
 
 ros::init(argc, argv, "UI");
 ros::NodeHandle n;
 
 client_UI = n.serviceClient<second_assignment::UserInterface>("/userinterface");
 
 ros::Subscriber sub_UI = n.subscribe("/base_scan", 1, CommandCallback);
 ros::spin();


 return 0;
}
