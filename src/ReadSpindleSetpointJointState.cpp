#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ReadSpindleSetpointJointState.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGOSPINDLE;

ReadSpindleSetpointJointState::ReadSpindleSetpointJointState(const string& name) : TaskContext(name, PreOperational)
{
	// Creating ports:
	addEventPort( "spindle_setpoint", spindle_setpoint_inport );
	addPort( "out", ref_outport );

	// Creating variables
	minpos = -1.0;
	maxpos = -1.0;
	maxvel = -1.0;
	addProperty( "min_pos", minpos );
	addProperty( "max_pos", maxpos );
	addProperty( "max_vel", maxvel );
}

ReadSpindleSetpointJointState::~ReadSpindleSetpointJointState()
{
}

bool ReadSpindleSetpointJointState::configureHook()
{
	// Set size of reference vector
	ref.resize(1); //Single joint
	ref[0].assign(3,0.0); //pos, vel, acc

	return true;
}

bool ReadSpindleSetpointJointState::startHook()
{
	// Check validity of Ports:
	if ( !spindle_setpoint_inport.connected() ) {
		log(Error)<<"ReadSpindleSetpoint::inputport not connected!"<<endlog();
		return false;
	}

	if ( !ref_outport.connected() ) {
		log(Warning)<<"ReadSpindleSetpoint::Outputport not connected!"<<endlog();
	}

	if ( minpos < 0.0 || maxpos < 0.0 || maxvel <= 0.0 ) {
		log(Error)<<"ReadSpindleSetpoint::Minima or maxima not correctly indicated!"<<endlog();
		return false;
	}
	
	return true;
}

void ReadSpindleSetpointJointState::updateHook()
{
	// Read the inputports
	//amigo_msgs::spindle_setpoint spindle_setpoint;
	sensor_msgs::JointState input;
	if ( spindle_setpoint_inport.read( input ) == NewData ) {
		log(Debug)<<"ReadSpindle: Received new input!"<<endlog();
	  ref[0][0] = fmin( maxpos, fmax( minpos, input.position[0] ) );
	  ref[0][1] = maxvel;//fmin( maxvel, input.velocity[0] );
	  ref[0][2] = 0.0;//input.effort[0];
		
	  // Write data to ports
	  ref_outport.write( ref );
	}
}

ORO_CREATE_COMPONENT(AMIGOSPINDLE::ReadSpindleSetpointJointState)
