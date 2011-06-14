#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "CalculateFFW.hpp"

using namespace RTT;
using namespace AMIGO;

CalculateFFW::CalculateFFW(const string& name) : TaskContext(name, PreOperational),
  
  FFWgrav_property("FFWgrav","Vector",0.07),
  FFWstat_property("FFWstat","Vector",0.05),
  FFWdyn_property("FFWdyn","Vector",0.4),
  FFWacc_property("FFWacc","Vector",0.1)

{
  // Creating ports:
  addEventPort( "ref_vel", refvel_inport );
  addPort( "ref_acc", accref_inport );
  addPort( "FFW_out", FFW_outport );
  
  // Creating variables
  addProperty( FFWgrav_property );
  addProperty( FFWstat_property );
  addProperty( FFWdyn_property );
  addProperty( FFWacc_property );
  
  
  // Initialising variables
  FFW.assign(1,0.0);
  ref_vel.assign(1,0.0);
  ref_acc.assign(1,0.0);
}
CalculateFFW::~CalculateFFW(){}

bool CalculateFFW::configureHook()
{
  FFWgrav = FFWgrav_property.get();
  FFWstat = FFWstat_property.get();
  FFWdyn = FFWdyn_property.get();
  FFWacc = FFWacc_property.get();
  return true;
}

bool CalculateFFW::startHook()
{
  // Check validity of Ports:
  if ( !refvel_inport.connected() || !accref_inport.connected() )
  {
    log(Error)<<"CalculateFFW::One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !FFW_outport.connected() ) {
    log(Warning)<<"CalculateFFW::Outputport not connected!"<<endlog();
  }
  return true;
}

void CalculateFFW::updateHook()
{
  // Read the inputports
  refvel_inport.read(ref_vel);
  accref_inport.read(ref_acc);
  
  // Calculating FFW
  FFW[0] = FFWgrav + FFWstat*sign(ref_vel[0]) + FFWdyn*ref_vel[0] + FFWacc*ref_acc[0];
  
  // Write data to output ports
  FFW_outport.write(FFW);
}


// A function used to determine the sign of the velocity
int CalculateFFW::sign(double sign_vel)
{
  if (sign_vel > 0.0)
  return 1;
  else if (sign_vel < 0.0)
  return -1;
  else
  return 0; 
}

ORO_CREATE_COMPONENT(CalculateFFW)
