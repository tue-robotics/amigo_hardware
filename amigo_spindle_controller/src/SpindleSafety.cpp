#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <std_msgs/Bool.h>

#include "SpindleSafety.hpp"

#include <ros/ros.h>

#define	BRAKEOFF true
#define BRAKEON	false

using namespace std;
using namespace RTT;
using namespace AMIGO;

SpindleSafety::SpindleSafety(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "error_pos", errorposPort );
  addPort( "enable_endswitch_safety", enableEndswitchSafetyPort );
  addPort( "endswitch_inport", endswitchPort );
  addPort( "spindle_brake", spindlebrakePort );
  addPort( "status", statusPort );
  addPort( "errortosupervisor", errortosupervisorPort );
  
  addProperty( "error_margin", errormargin );
  
  // Initialising variables
  error_pos.assign(1,0.0);  
}
SpindleSafety::~SpindleSafety(){}

bool SpindleSafety::configureHook()
{
  StatusOperational.level = 0;
  StatusError.level = 4;  	
  return true;
}

bool SpindleSafety::startHook()
{ 
  // Check validity of Ports:
  if ( !errorposPort.connected() )
  {
    log(Error)<<"SpindleSafety::Inputport not connected!"<<endlog();
    return false;
  }
  if ( !spindlebrakePort.connected() || !statusPort.connected() ) {
    log(Warning)<<"SpindleSafety::One or more outputports not connected!"<<endlog();
  }
  if ( ! (errormargin > 0.0) ) {
	  log(Warning)<<"SpindleSafety::error_margin not (correctly) specified!"<<endlog();
  }
  	 
  safety = true;
  enable_endswitch_safety = false;
  
  log(Warning)<<"SpindleSafety::started at " << os::TimeService::Instance()->getNSecs()*1e-9 <<endlog();

  return true;
}

void SpindleSafety::updateHook()
{
	// Measuring safety
	errorposPort.read(error_pos);
	if( abs(error_pos[0]) > errormargin )
	{
		if(safety){ // Safety was true, so log now once
			ROS_ERROR_STREAM( "Error ( " << error_pos[0] << " ) too large! Spindle is probably blocked, change spindle setpoint." );
			log(Error) << "Error ( " << error_pos[0] << " ) too large! Spindle is probably blocked, change spindle setpoint." << endlog();
			errortosupervisorPort.write(true);
		}
		safety = false;
	}
	
	/*TODO
	// The endswitch safety is enabled as soon as it receives the go-ahead from the Spindle Homing
	if (enableEndswitchSafetyPort.read(enable_endswitch_safety) == NewData) log(Info)<<"Endswitch safety enabled"<<endlog();
	
	// If endswitch safety is enabled and the endswitch port reads false, the spindle is out of range which is not safe
	std_msgs::Bool endswitch;
	endswitchPort.read(endswitch);
	if (enable_endswitch_safety && !endswitch.data) 
	{
		if (safety) {
			ROS_ERROR_STREAM( "Spindle out of range, endswitch data is false" );
			log(Error) << "Spindle out of range, endswitch data is false" << endlog();
			//safety = false; // TIMC: Disabled for debugging purposes
		}
	}*/
	
	// Applying brake if necessary
	if(safety == false)
	{
	    spindlebrakePort.write(BRAKEON);
	    statusPort.write(StatusError);
	}
	else
	{
		spindlebrakePort.write(BRAKEOFF);
		statusPort.write(StatusOperational);
	}
}


ORO_CREATE_COMPONENT(SpindleSafety)
