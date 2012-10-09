#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SpindleSafety.hpp"

#define	BRAKEOFF true
#define BRAKEON	false
#define ERRORMARGIN 0.02

using namespace std;
using namespace RTT;
using namespace AMIGO;

SpindleSafety::SpindleSafety(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "error_pos", errorpos_inport );
  addPort( "spindle_brake", spindle_brake_outport );
  addPort( "safety", safety_outport );
  
  // Initialising variables
  error_pos.assign(1,0.0);  
}
SpindleSafety::~SpindleSafety(){}

bool SpindleSafety::configureHook()
{
  once = false;
  return true;
}

bool SpindleSafety::startHook()
{ 
  // Check validity of Ports:
  if ( !errorpos_inport.connected() )
  {
    log(Error)<<"SpindleSafety::Inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !spindle_brake_outport.connected() || !safety_outport.connected() ) {
    log(Warning)<<"SpindleSafety::One or more outputports not connected!"<<endlog();
  }
  	 
  safety = true;
  return true;
}

void SpindleSafety::updateHook()
{
	
	// Measuring safety
	errorpos_inport.read(error_pos);
	if( abs(error_pos[0]) > ERRORMARGIN )
	{
		safety = false;
		
		// Publishing a message after 500 cycles, which is 2 seconds at 250 Hz
		publish_counter += 1.0;
		if( publish_counter == 500.0)
		{
			if(!once){
				log(Warning)<<"Error too large! Spindle is probably blocked, change spindle setpoint."<<endlog();
				once = true;
			}
			publish_counter = 0.0;
		}
	}
	
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
