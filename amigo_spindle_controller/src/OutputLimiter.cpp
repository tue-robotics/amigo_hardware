#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>
#include <stdint.h>

#include "OutputLimiter.hpp"

#define VOLTLIMIT 1.0

using namespace RTT;
using namespace AMIGO;

OutputLimiter::OutputLimiter(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "input", inport );
  addPort( "safety", safety_inport);
  addPort( "output", outport );
  
  // Initialising variables
  input.assign(1,0.0);
  
}
OutputLimiter::~OutputLimiter(){}

bool OutputLimiter::configureHook()
{
  return true;
}

bool OutputLimiter::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() || !safety_inport.connected() )
  {
    log(Error)<<"OutputLimiter::One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !outport.connected() ) {
    log(Warning)<<"OutputLimiter::Outputport not connected!"<<endlog();
  }
  return true;
}

void OutputLimiter::updateHook()
{
	// Reading input ports
	inport.read(input);
	bool safety;
	safety_inport.read(safety);
	
	// Limiting output or setting output to zero if safety is not guaranteed
	double output;
	if( safety == false )
		output = 0.0 ;
	else
		output = - max(-VOLTLIMIT, min(VOLTLIMIT,input[0]) );
	
	// Writing to output port
	outport.write(output);
}

ORO_CREATE_COMPONENT(OutputLimiter)
