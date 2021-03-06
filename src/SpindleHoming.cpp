#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "SpindleHoming.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace AMIGOSPINDLE;


SpindleHoming::SpindleHoming(const string& name) : TaskContext(name, PreOperational)
{  
	// inports
	addEventPort( "endswitch", endswitch_inport );
	addPort( "position",pos_inport );
	// outports
	addPort( "ref_out", ref_outport );
	addPort( "homing_finished", homingfinished_outport );
	addPort( "resetRef",resetRefPort).doc("Sends reset joint coordinates to ROS topic");

	// Properties
	addProperty( "home_vel", home_vel );
	addProperty( "home_acc", home_acc );
	addProperty( "stroke", stroke );
	addProperty( "endpos", endpos );
	addProperty( "homed", homed );  
}

SpindleHoming::~SpindleHoming()
{
}

bool SpindleHoming::configureHook()
{
	ref.resize(1);
	position.assign(1,0.0);
	reference = 0.0;
	referencestep = 0.01/1000;
	out_msg.position.assign(1,0.0);
	return true;
}

bool SpindleHoming::startHook()
{ 
	// if homing is not required, then this component is stopped: To Do is this function still required in the new structure?
	if ( homed == true ){
		log(Warning)<<"Spindle: stopping component without homing. Homing was set to false in .ops script"<<endlog();
		this->stop(); 
	}	
		
	// Lookup the Supervisor component.
	TaskContext* Supervisor = this->getPeer("Supervisor");
	if ( !Supervisor ) {
		log(Error) << "Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
	}
	// Lookup the Encoder component.
	TaskContext* SpindleReadEncoder = this->getPeer("SPINDLE_ReadEncoders");
	if ( !SpindleReadEncoder ) {
		log(Error) << "Could not find SPINDLE_ReadEncoders component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
	}
	// Lookup the Setpoint component.
	TaskContext* Spindle_ReadReferences = this->getPeer("SPINDLE_ReadReferences");
	if ( !Spindle_ReadReferences ) {
		log(Error) << "Could not find SPINDLE_ReadReferences component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
	}
	
	// Lookup operations of peers
	StartBodyPart = Supervisor->getOperation("StartBodyPart");
	if ( !StartBodyPart.ready() ) {
		log(Error) << "Could not find Supervisor.StartBodyPart Operation!"<<endlog();
		return false;
	}
	StopBodyPart = Supervisor->getOperation("StopBodyPart");
	if ( !StopBodyPart.ready() ) {
		log(Error) << "Could not find Supervisor.StopBodyPart Operation!"<<endlog();
		return false;
	}	
	ResetEncoder = SpindleReadEncoder->getOperation("reset");
	if ( !ResetEncoder.ready() ) {
		log(Error) << "Could not find SPINDLE_ReadEncoders.reset Operation!"<<endlog();
		return false;
	}
	
	homed_ = homed;
	goToEndpos = false;
	homingfinished = false;
	cntr = 0;
	ref[0].assign(3,0.0);

	pos_inport.read(position);
	reference = position[0];
		
	return true;
}

void SpindleHoming::updateHook()
{
	std_msgs::Bool endswitch;
	endswitch_inport.read(endswitch);
	
	if ( homed_ == false )
	{	
		double oldreference = reference;
		reference = oldreference + referencestep;
		ref[0][0] = reference;
		ref[0][1] = home_vel;
		ref[0][2] = home_acc;
		ref_outport.write(ref);
	}

	if ( !endswitch.data && homed_ == false )
	{
		homed_ = true;
		StopBodyPart("spindle");
		ResetEncoder(0,stroke);
		StartBodyPart("spindle");
		
		goToEndpos = true;
	}
	if (goToEndpos) {
		goToEndpos = false;
		// Go to desired position
		ref[0][0] = endpos;
		ref[0][1] = 0.0; // Use default
		ref[0][2] = 0.0; // Use default
		ref_outport.write(ref);
		homingfinished = true;
		cntr = 0;
	}
	if (homingfinished) {
		cntr++;
		if (cntr > 1000) {
			log(Warning)<<"Spindle: Homing Finished!"<<endlog();
			// send to supervisor that homing is finished. Supervisor will shutdown homing component
			homingfinished_outport.write(true);
			
			out_msg.position[0] = endpos;
			// send reset data
			resetRefPort.write(out_msg);
			
			cntr = 0;
		}
	}
}

ORO_CREATE_COMPONENT(AMIGOSPINDLE::SpindleHoming)
