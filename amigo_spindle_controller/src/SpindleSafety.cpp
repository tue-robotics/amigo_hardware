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
  addPort( "errortosupervisor", errortosupervisorPort );
  // properties
  addProperty( "error_margin", errormargin );
}
SpindleSafety::~SpindleSafety(){}

bool SpindleSafety::configureHook()
{
  error_pos.assign(1,0.0);   
  return true;
}

bool SpindleSafety::startHook()
{ 
  safety = true;
  enable_endswitch_safety = false;
  
  // Check validity of Ports:
  if ( !errorposPort.connected() )
  {
    log(Error)<<"SpindleSafety: Inputport not connected!"<<endlog();
    return false;
  }
  if ( !spindlebrakePort.connected() ) {
    log(Error)<<"SpindleSafety: spindlebrakePort not connected!"<<endlog();
    return false;
  }
  if ( ! (errormargin > 0.0) ) {
	  log(Error)<<"SpindleSafety: error_margin not (correctly) specified!"<<endlog();
  }
    
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
			ROS_ERROR_STREAM( "SPINDLE_SAFETY: Error ( " << error_pos[0] << " ) too large! Spindle is probably blocked, change spindle setpoint." );
			log(Error) << "SPINDLE_SAFETY:  Error ( " << error_pos[0] << " ) too large! Spindle is probably blocked, change spindle setpoint." << endlog();
			errortosupervisorPort.write(true);
		}
		safety = false;
	}
	

	if(safety == false)	{
	    spindlebrakePort.write(BRAKEON);
	}
	else {
		spindlebrakePort.write(BRAKEOFF);
	}
}


ORO_CREATE_COMPONENT(SpindleSafety)
