#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "SpindleHoming.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace AMIGO;


SpindleHoming::SpindleHoming(const string& name) : TaskContext(name, PreOperational)
  {
  addEventPort( "endswitch", endswitch_inport );

  addPort( "ref_out", ref_outport );

  
  // Creating variables
  home_vel = 0.01;
  home_acc = 0.01;
  addProperty( "home_vel", home_vel );
  addProperty( "home_acc", home_acc );
  addProperty( "stroke", stroke );
  addProperty( "endpos", endpos );
  addProperty( "homed", homed );
  
  
}
SpindleHoming::~SpindleHoming(){}

bool SpindleHoming::configureHook()
{
	// Lookup the Supervisor component.
	TaskContext* Supervisor = this->getPeer("Supervisor");
	if ( !Supervisor ) {
		log(Error) << "Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
	}
	// Lookup the Encoder component.
	TaskContext* SpindleReadEncoder = this->getPeer("Spindle_ReadEncoders");
	if ( !SpindleReadEncoder ) {
		log(Error) << "Could not find Spindle_ReadEncoders component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
	}	
	// Lookup the Setpoint component.
	TaskContext* Spindle_ReadReferences = this->getPeer("Spindle_ReadReferences");
	if ( !Spindle_ReadReferences ) {
		log(Error) << "Could not find Spindle_ReadReferences component! Did you add it as Peer in the ops file?"<<endlog();
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
		log(Error) << "Could not find Spindle_ReadEncoders.reset Operation!"<<endlog();
		return false;
	}	
	
	// Set size of reference vector
	ref.resize(1); //Single joint
	ref[0].resize(3); //pos, vel, acc
	ref[0][1] = home_vel; //Redundant?
	ref[0][2] = home_acc; //Redundant?
	
	return true;
}

bool SpindleHoming::startHook()
{ 
		if ( !homed ) {
			TaskContext* Spindle_ReadReferences = this->getPeer("Spindle_ReadReferences");
			if ( ! Spindle_ReadReferences->isRunning() ) {
				log(Error) << "Spindle component is not running yet, please start this component first" << endlog();
			}
			else {
				Spindle_ReadReferences->stop(); //Disabling reading of references. Will be enabled automagically at the end by the supervisor.
			}
		}
		
	starttime = os::TimeService::Instance()->getNSecs()*1e-9;
	log(Warning)<<"SpindleHoming::started at " << os::TimeService::Instance()->getNSecs()*1e-9 <<endlog();

	return true;
}

void SpindleHoming::updateHook()
{   	
	//log(Warning)<<"Spindle: Reading endswitch"<<endlog();
	std_msgs::Bool endswitch;
	endswitch_inport.read(endswitch);
	
	//log(Warning)<<"Spindle: Sending reference"<<endlog();
	if ( homed == false )
	{
		ref[0][0] = 1.0; // You always find the endstop within the meter
		ref[0][1] = home_vel;
		ref[0][2] = home_acc;
		ref_outport.write(ref);
	}
	//log(Warning)<<"Spindle: Checking whether homed = true"<<endlog();
	if ( !endswitch.data && homed == false )
	{
		ROS_INFO_STREAM( "Spindle is homed." );
		homed = true;


		// Actually call the services
		//log(Warning)<<"SpindleHoming::start calling services at " << os::TimeService::Instance()->getNSecs()*1e-9 - starttime <<endlog();
		StopBodyPart("spindle");
		ResetEncoder(0,stroke);
		StartBodyPart("spindle");
		//log(Warning)<<"SpindleHoming::finshed calling services at " << os::TimeService::Instance()->getNSecs()*1e-9 - starttime <<endlog();
		
		// Got to desired position
		ref[0][0] = endpos;
		ref[0][1] = 0.0; // Use default
		ref[0][2] = 0.0; // Use default
		ref_outport.write(ref);
		this->stop(); 
	}
	if ( homed == true ){
		//log(Warning)<<"Spindle: stop component"<<endlog();
		//Should not happen only if homed == true is defined in ops file
		this->stop(); 
	}
}



ORO_CREATE_COMPONENT(SpindleHoming)
