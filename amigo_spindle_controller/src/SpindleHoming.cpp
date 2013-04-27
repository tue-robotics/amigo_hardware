#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SpindleHoming.hpp"

#include <ros/ros.h>

#define RESET 1.0
#define NORESET 0.0

using namespace std;
using namespace RTT;
using namespace AMIGO;


SpindleHoming::SpindleHoming(const string& name) : TaskContext(name, PreOperational),
  
  maxvel_property( "max_vel" , "Vector", 0.05),
  maxacc_property( "max_acc" , "Vector", 0.02)
  
  {
  // Creating ports:
  addEventPort( "encoder_in", encoder_inport );
  addPort( "error_pos", errorpos_inport );
  addPort( "ref_pos_in", refpos_inport );
  addPort( "current_pos", currentpos_inport );
  addPort( "safe", safe_inport );
  addPort( "ros_emergency", ros_emergency_inport );
  addPort( "endswitch", endswitch_inport );

  addPort( "ref_pos_out", refpos_outport );
  addPort( "correction_out", correction_outport );
  addPort( "reset_generator", reset_generator_outport );
  addPort( "enable_endswitch_safety", enable_endswitch_safety_outport );
  
  // Creating variables
  addProperty( maxvel_property );
  addProperty( maxacc_property ); 
  addProperty( "stroke", stroke );
  
  // Initialising variables
  ref_pos.assign(1,0.0);
  input.assign(1,0.0);
  correction.assign(1,0.0);
  error_pos.assign(1,0.0);
  generator_reset.assign(4,0.0);
  
}
SpindleHoming::~SpindleHoming(){}

bool SpindleHoming::configureHook()
{
  return true;
}

bool SpindleHoming::startHook()
{ 
  // Check validity of Ports:
  if ( !encoder_inport.connected() || !errorpos_inport.connected() || !refpos_inport.connected() )
  {
    log(Error)<<"SpindleHoming:One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !refpos_outport.connected() || !correction_outport.connected() || !reset_generator_outport.connected() || !endswitch_inport.connected() || !enable_endswitch_safety_outport.connected() ) {
    log(Warning)<<"SpindleHoming:One or more outputports not connected!"<<endlog();
  }
  if (!safe_inport.connected() || !ros_emergency_inport.connected() ){
	  log(Warning)<<"SpindleHoming: Safe inport or emergency button inport not connected!"<<endlog();
  }

  homed = false;
  safe = false;
  sent_enable_endswitch_safety = false;
  maxvel = maxvel_property.get();
  maxacc = maxacc_property.get();
  log(Info)<<"Spindle is not homed. Homing procedure started."<<endlog();  
  return true;
}

void SpindleHoming::updateHook()
{   	
	errorpos_inport.read(error_pos);
	refpos_inport.read(refpos);
	ref_pos[0] = refpos;
	generator_reset[0] = NORESET;
	currentpos_inport.read(current_pos);

	safe_inport.read( safe );
	ros_emergency_inport.read(emergency_button);
	endswitch_inport.read(endswitch);
	
	// Error or emergency button pressed
	if ( !safe || emergency_button.data )
	{
	    ref_pos[0] = current_pos[0];
	}
	
	//Homing finished
	//else if(abs(error_pos[0]) > HOMINGERROR && homed == false)
	else if ( !endswitch.data && homed == false )
	{
		ROS_INFO_STREAM( "Spindle is homed." );
		log(Info) << "Spindle is homed." << endlog();
		homed = true;
		encoder_inport.read(input);
		correction[0] = stroke + input[0];
		
		// a variable used once to reset the reference generator after homing
		homing_correction = correction[0];// + HOMINGERROR;
		generator_reset[0] = RESET;
		generator_reset[1] = current_pos[0] + homing_correction;
		generator_reset[2] = maxvel;
		generator_reset[3] = maxacc;
		
		reset_generator_outport.write(generator_reset);	
	}

	//Homing
	else if(homed == false)
	{
		ref_pos[0] = 0.5;
	}
	
	// Send a bool to SpindleSafety to enable endswitch safety
	if (homed && endswitch.data && !sent_enable_endswitch_safety) 
	{
		log(Info)<<"Sending enable_endswitch_safety"<<endlog();
		sent_enable_endswitch_safety = true;
		enable_endswitch_safety_outport.write(true);
	}
	
	// Write to output ports
	refpos_outport.write(ref_pos);
	correction_outport.write(correction);
	
	// Commented 19-04-2012
	reset_generator_outport.write(generator_reset);	
}

ORO_CREATE_COMPONENT(SpindleHoming)
