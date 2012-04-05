#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <algorithm>
#include <string>

using namespace std;

ros::Time time_init, time_current;
ros::Publisher amplifier_pub;
ros::Subscriber diag_sub;
ros::Subscriber emergency_sub;

/*
diagnostic_updater::DiagnosticStatusWrapper status;
vector<diagnostic_msgs::DiagnosticStatus> statuses;
statuses.push_back(status);
diagnostic_msgs::DiagnosticArray diag_msg;
*/
int battery_status = 0;
bool emergency_switch_status = true;
std_msgs::Bool disable_amplifiers;

void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
	//fuse1=msg->data;
	// Diagnostic message is only relevant if it concerns the batteries
	diagnostic_msgs::DiagnosticStatus diagnostic_status = msg->status[0];
	
	if (!strcmp(diagnostic_status.name.c_str(),"Batteries")){
			
		//ROS_WARN("Status level = %i",diagnostic_status.level);
		battery_status = diagnostic_status.level;
	}
	/*else {
		
		//ROS_WARN("Diagnostic message does not concern batteries");
		
	}*/
	
}

void emergencySwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
	emergency_switch_status=msg->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "amplifier_enabler");
	ros::NodeHandle n;
	
	amplifier_pub = n.advertise<std_msgs::Bool>("/disable_amplifiers", 1);

	diag_sub = n.subscribe("/diagnostics", 1, diagnosticCallback);
	emergency_sub = n.subscribe("/emergency_switch", 1, emergencySwitchCallback);

	time_init = ros::Time::now();	

	ros::Rate loop_rate(250.0);
	
	while(n.ok())
	{
		ros::spinOnce();
		
		// Disable amplifiers when emergency switch is pushed or battery status equals 2 (= Voltage is too low)
		if (battery_status != 2 && emergency_switch_status == false){
			disable_amplifiers.data = false;
			//ROS_WARN("Enable amplifiers");
		}
		else{
			disable_amplifiers.data = true;
			//ROS_WARN("Disable amplifiers");
		}
		amplifier_pub.publish(disable_amplifiers);
		
		loop_rate.sleep();

	}
	return 0;
	
}

