#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <std_msgs/Float64.h>

#include "PublishPosition.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

PublishPosition::PublishPosition(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "in", spindle_pos_inport );
  addPort( "out", spindle_pos_outport );
  
  // Initialising variables
  current_position.assign(1,0.0);
}
PublishPosition::~PublishPosition(){}

bool PublishPosition::configureHook()
{
  return true;
}

bool PublishPosition::startHook()
{
  // Check validity of Ports:
  if ( !spindle_pos_inport.connected() )
  {
    log(Error)<<"PublishPosition::One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !spindle_pos_outport.connected() ) {
    log(Warning)<<"PublishPosition::Outputport not connected!"<<endlog();
  }
  return true;
}

void PublishPosition::updateHook()
{
	// Reading input port
	spindle_pos_inport.read(current_position);
	
	// Writing to output port
	spindle_position.data = current_position[0];
	spindle_pos_outport.write(spindle_position);	
}

ORO_CREATE_COMPONENT(PublishPosition)
