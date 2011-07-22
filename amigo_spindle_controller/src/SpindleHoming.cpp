#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "SpindleHoming.hpp"

#define HOMINGERROR 0.005
#define HOMEDPOSITION 0.35
#define STROKE 0.413
#define RESET 1.0
#define NORESET 0.0

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
  addPort( "ref_pos_out", refpos_outport );
  addPort( "correction_out", correction_outport );
  addPort( "reset_generator", reset_generator_outport );
  
  // Creating variables
  addProperty( maxvel_property );
  addProperty( maxacc_property ); 
  
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
  if ( !refpos_outport.connected() || !correction_outport.connected() || !reset_generator_outport.connected() ) {
    log(Warning)<<"SpindleHoming:One or more outputports not connected!"<<endlog();
  }

  homed = false;
  maxvel = maxvel_property.get();
  maxacc = maxacc_property.get();
  log(Info)<<"Spindle is not homed. Homing procedure started."<<endlog();  
  return true;
}

void SpindleHoming::updateHook()
{   	
	errorpos_inport.read(error_pos);
	//double refpos_old = ref_pos[0];
	refpos_inport.read(refpos);
	ref_pos[0] = refpos;
	generator_reset[0] = NORESET;
	
	if(abs(error_pos[0]) > HOMINGERROR && homed == false)
	{
		log(Info)<<"Spindle is homed."<<endlog();
		homed = true;
		encoder_inport.read(input);
		correction[0] = STROKE + input[0];
		
		// a variable used once to reset the reference generator after homing
		homing_correction = correction[0] + HOMINGERROR;
		currentpos_inport.read(current_pos);
		generator_reset[0] = RESET;
		generator_reset[1] = current_pos[0] + homing_correction;
		generator_reset[2] = maxvel;
		generator_reset[3] = maxacc;
	}
	if(homed == false)
	{
		ref_pos[0] = 0.5;
	}
	
	// Write to output ports
	refpos_outport.write(ref_pos);
	correction_outport.write(correction);
	reset_generator_outport.write(generator_reset);	
}

ORO_CREATE_COMPONENT(SpindleHoming)
