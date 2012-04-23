#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <algorithm>
#include <string>

using namespace std;

ros::Time time_init, time_current, time_ebutton_enable;
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
bool emergency_switch_status = true; /// Status of the incoming emergency switch
bool ebutton_safe = true; /// Status is set to true if the emergency_switch_status is false for a certain duration
bool previous_emergency_switch_status = false;
std_msgs::Bool disable_amplifiers;
ros::Duration emergency_switch_delay(0.5); /// Time between releasing the emergency switch and enabling amplifiers
ros::Duration enabled_time; /// Time the ebutton has been enabled in ROS time
//double enabled_time_sec; /// Time the ebutton has been enabled

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
	/// Set new emergency switch time when ebutton is pressed and upon release
	if ( (emergency_switch_status == false && previous_emergency_switch_status == true) || emergency_switch_status == true){
		
		time_ebutton_enable = ros::Time::now();
		
	}
	/*enabled_time = ros::Time::now() - time_ebutton_enable;
	enabled_time_sec = enabled_time.toSec();
	if (enabled_time_sec > emergency_switch_delay){
		
		ebutton_safe = true;
		
	}*/
	if ( (ros::Time::now() - time_ebutton_enable) > emergency_switch_delay ){
		ebutton_safe = true;
	}
	else {
		ebutton_safe = false;
	}
	
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
		//if (battery_status != 2 && emergency_switch_status == false){
		if (battery_status != 2 && ebutton_safe == true){
			disable_amplifiers.data = false;
			//ROS_WARN("Enable amplifiers");
		}
		else{
			disable_amplifiers.data = true;
			//ROS_WARN("Disable amplifiers");
		}
		amplifier_pub.publish(disable_amplifiers);
		
		previous_emergency_switch_status = emergency_switch_status;
		
		loop_rate.sleep();

	}
	return 0;
	
}

