#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "ReferenceLimiter.hpp"

#define STROKE 0.413
#define SAFETYMARGIN 0.005
#define TRESHOLDHEIGHT 0.35
#define BASERADIUS 0.678

using namespace RTT;
using namespace AMIGO;

ReferenceLimiter::ReferenceLimiter(const string& name) : TaskContext(name, PreOperational)


{
  // Creating ports:
  addPort( "right_tip", right_tip_inport );
  addPort( "left_tip", left_tip_inport ); 
  addPort( "spindle_position", spindle_position_inport );
  addEventPort( "ref_pos_in", refpos_inport );
  addPort( "ref_pos_out", refpos_outport );
  addPort( "report_right_tip", reporter_port );
  
  // Initialising variables
  current_position.assign(1,0.0);
  publish_countera = 0.0;
  publish_counterb = 0.0;
}
ReferenceLimiter::~ReferenceLimiter(){}

bool ReferenceLimiter::configureHook()
{
  return true;
}

bool ReferenceLimiter::startHook()
{
  // Check validity of Ports:
  if ( !right_tip_inport.connected() || !left_tip_inport.connected() || !spindle_position_inport.connected() || !refpos_inport.connected() ) {
    log(Error)<<"ReferenceLimiter::One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  
  if ( !refpos_outport.connected() ) {
    log(Warning)<<"ReferenceLimiter::Outputport not connected!"<<endlog();
  }
  return true;
}

void ReferenceLimiter::updateHook()
{
  // Read the inputports
  refpos_inport.read(ref_pos);
  spindle_position_inport.read(current_position);
  right_tip_inport.read(right_tip);
  left_tip_inport.read(left_tip);
 
  double minimum_pos = SAFETYMARGIN;
  double maximum_pos = STROKE - SAFETYMARGIN;
 
	// Limiting the references to the minimum and maximum if necessary
    if(ref_pos < minimum_pos)
	{
		ref_pos = minimum_pos;
	}
	if(ref_pos > maximum_pos)
	{
		ref_pos = maximum_pos;
	}
	
	// Calculating circular coordinates of tip
	double radius_right = sqrt( right_tip.x*right_tip.x + right_tip.y*right_tip.y );
	double radius_left = sqrt( left_tip.x*left_tip.x + left_tip.y*left_tip.y );
	
	// Checking if the arms are in the collision zone. 
	// Limiting references if this is the case. 
	if( NewData != right_tip_inport.read( right_tip ) )
	{	
		// Publishing a message after 1500 cycles, which is 6 seconds at 250 Hz
		publish_countera += 1.0;
		if( publish_countera == 1500.0 )
		{
		log(Warning)<<"No information about right arm received. Use spindle with caution, no arm safety!!"<<endlog();
		publish_countera = 0.0;
		}
	}
	if( NewData != left_tip_inport.read( left_tip ) )
	{
		// Publishing a message after 1500 cycles, which is 6 seconds at 250 Hz		
		publish_counterb += 1.0;
		if( publish_counterb == 1500.0 )
		{
		log(Warning)<<"No information about left arm received. Use spindle with caution, no arm safety!!"<<endlog();
		publish_counterb = 0.0;
		}
	}
	
  // Write data to ports
  refpos_outport.write( ref_pos );
  reporter_port.write(right_tip);
}

ORO_CREATE_COMPONENT(ReferenceLimiter)
