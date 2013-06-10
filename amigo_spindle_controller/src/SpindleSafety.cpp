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
  addEventPort( "error_pos", errorpos_inport );
  addPort( "enable_endswitch_safety", enable_endswitch_safety_inport );
  addPort( "endswitch_inport", endswitch_inport );
  addPort( "spindle_brake", spindle_brake_outport );
  addPort( "safety", safety_outport );
  
  addProperty( "error_margin", errormargin );
  
  // Initialising variables
  error_pos.assign(1,0.0);  
}
SpindleSafety::~SpindleSafety(){}

bool SpindleSafety::configureHook()
{
  return true;
}

bool SpindleSafety::startHook()
{ 
  // Check validity of Ports:
  if ( !errorpos_inport.connected() )
  {
    log(Error)<<"SpindleSafety::Inputport not connected!"<<endlog();
    return false;
  }
  if ( !spindle_brake_outport.connected() || !safety_outport.connected() ) {
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
	errorpos_inport.read(error_pos);
	if( abs(error_pos[0]) > errormargin )
	{
		if(safety){ // Safety was true, so log now once
			ROS_ERROR_STREAM( "Error ( " << error_pos[0] << " ) too large! Spindle is probably blocked, change spindle setpoint." );
			log(Error) << "Error ( " << error_pos[0] << " ) too large! Spindle is probably blocked, change spindle setpoint." << endlog();
		}
		safety = false;
	}
	
	/*TODO
	// The endswitch safety is enabled as soon as it receives the go-ahead from the Spindle Homing
	if (enable_endswitch_safety_inport.read(enable_endswitch_safety) == NewData) log(Info)<<"Endswitch safety enabled"<<endlog();
	
	// If endswitch safety is enabled and the endswitch port reads false, the spindle is out of range which is not safe
	std_msgs::Bool endswitch;
	endswitch_inport.read(endswitch);
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
	    spindle_brake_outport.write(BRAKEON);
	}
	else
	{
		spindle_brake_outport.write(BRAKEOFF);
	}
	
	// Writing boolean safety to port
	safety_outport.write(safety);
}


ORO_CREATE_COMPONENT(SpindleSafety)
