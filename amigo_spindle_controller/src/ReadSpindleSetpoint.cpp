#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "ReadSpindleSetpoint.hpp"

using namespace std;
using namespace RTT;
using namespace MSG;

ReadSpindleSetpoint::ReadSpindleSetpoint(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "spindle_setpoint", spindle_setpoint_inport );
  addPort( "afterhoming_pos", afterhoming_outport );
  addPort( "ref_pos", refpos_outport );
  addPort( "ref_vel", refvel_outport );
  addPort( "ref_acc", refacc_outport );
  
  // Creating variables
  homed_position_property = 0.35;
  addProperty( "homed_pos", homed_position_property );
}
ReadSpindleSetpoint::~ReadSpindleSetpoint(){}

bool ReadSpindleSetpoint::configureHook()
{
  return true;
}

bool ReadSpindleSetpoint::startHook()
{
  // Check validity of Ports:
  if ( !spindle_setpoint_inport.connected() ) {
    log(Error)<<"ReadSpindleSetpoint::inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  
  if ( !refpos_outport.connected() ) {
    log(Warning)<<"ReadSpindleSetpoint::Outputport not connected!"<<endlog();
  }

  // During the startup the position which is desired after homing, 
  // is once published to the ROS topic.
  double homed_pos = homed_position_property;
  spindle_setpoint.pos = homed_pos;
  afterhoming_outport.write(spindle_setpoint);
  return true;
}

void ReadSpindleSetpoint::updateHook()
{
  // Read the inputports
  spindle_setpoint_inport.read(spindle_setpoint);
  
  	// Declaring variables
  double ref_pos;
  double ref_vel;
  double ref_acc;
  //int ref_stop;

  ref_pos = spindle_setpoint.pos;
  ref_vel = spindle_setpoint.vel;
  ref_acc = spindle_setpoint.acc;
  //ref_stop = spindle_setpoint.stop;
 	
  // Write data to ports
  refpos_outport.write( ref_pos );
  refvel_outport.write( ref_vel );
  refacc_outport.write( ref_acc );
}

ORO_CREATE_COMPONENT(MSG::ReadSpindleSetpoint)
