#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "ReadSpindleSetpoint.hpp"

#define STROKE 0.41
#define SAFETYMARGIN 0.01

using namespace RTT;
using namespace MSG;

ReadSpindleSetpoint::ReadSpindleSetpoint(const string& name) : TaskContext(name, PreOperational),

  homed_position_property("homed_pos", "Vector", 0.35 )

{
  // Creating ports:
  addEventPort( "spindle_setpoint", spindle_setpoint_inport );
  addPort( "afterhoming_pos", afterhoming_outport );
  addPort( "ref_pos", refpos_outport );
  
  // Creating variables
  addProperty( homed_position_property );
}
ReadSpindleSetpoint::~ReadSpindleSetpoint(){}

bool ReadSpindleSetpoint::configureHook()
{
  return true;
}

bool ReadSpindleSetpoint::startHook()
{
  Logger::In in("ReadSpindleSetpoint::startHook()");
  
  // Check validity of Ports:
  if ( !spindle_setpoint_inport.connected() ) {
    log(Error)<<"inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  
  if ( !refpos_outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }

  // During the startup the position which is desired after homing, 
  // is once published to the ROS topic.
  double homed_pos = homed_position_property.get();
  spindle_setpoint.pos = homed_pos;
  afterhoming_outport.write(spindle_setpoint);
  return true;
}

void ReadSpindleSetpoint::updateHook()
{
  // Read the inputports
  spindle_setpoint_inport.read(spindle_setpoint);
  ref_pos = spindle_setpoint.pos;
  
  double minimum_pos = SAFETYMARGIN;
  double maximum_pos = STROKE - SAFETYMARGIN;
 
	// Limiting the references if necessary
    if(ref_pos < minimum_pos)
	{
		ref_pos = minimum_pos;
	}
	if(ref_pos > maximum_pos)
	{
		ref_pos = maximum_pos;
	}
	
  // Write data to ports
  refpos_outport.write( ref_pos );
}

ORO_CREATE_COMPONENT(MSG::ReadSpindleSetpoint)
