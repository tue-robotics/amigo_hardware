//Reference publisher used for System Identification
#include <ros/ros.h>
#include <math.h>

#include <amigo_msgs/spindle_setpoint.h>

int main(int argc, char** argv)
{
	double reference = 0.15;
	ros::init(argc, argv, "SystemIdentificationRef");
	ros::NodeHandle n;
	ros::Publisher Spindleref_pub = n.advertise<amigo_msgs::spindle_setpoint>("/spindle_controller/spindle_coordinates", 1);
	ros::Rate rate(1);
	while (ros::ok())
	{
		
		if(reference == 0.15)
		{
			sleep(2);
			reference = 0.3;		
			amigo_msgs::spindle_setpoint spindle_setpoint;
			spindle_setpoint.pos = reference;
			Spindleref_pub.publish(spindle_setpoint);
			ros::spinOnce();
		}
		
		if(reference == 0.3)
		{			
			sleep(2);
			reference = 0.15;
			amigo_msgs::spindle_setpoint spindle_setpoint;
			spindle_setpoint.pos = reference;
			Spindleref_pub.publish(spindle_setpoint);
			ros::spinOnce();
		} 
		  
		rate.sleep();
	}
	return 1;
}
