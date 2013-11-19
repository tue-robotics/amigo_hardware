#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "SpindleHomingAdmittance.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace AMIGO;


SpindleHomingAdmittance::SpindleHomingAdmittance(const string& name) : TaskContext(name, PreOperational)
  {
  addEventPort( "endswitch", endswitch_inport );

  addPort( "ref_out", ref_outport );
  addPort( "homing_finished", homingfinished_outport );

  
  // Creating variables
  home_vel = 0.01;
  home_acc = 0.01;
  addProperty( "home_vel", home_vel );
  addProperty( "home_acc", home_acc );
  addProperty( "stroke", stroke );
  addProperty( "endpos", endpos );
  addProperty( "homed", homed );
  
}
SpindleHomingAdmittance::~SpindleHomingAdmittance(){}

bool SpindleHomingAdmittance::configureHook()
{
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
	
	// Set size of reference vector
	ref.resize(1); //Single joint
	
	reference = 0.0;
	referencestep = 0.01/1000;
	cntr = 0;
	
	return true;
}

bool SpindleHomingAdmittance::startHook()
{ 
		if ( !homed ) {
			TaskContext* Spindle_ReadReferences = this->getPeer("SPINDLE_ReadReferences");
			if ( ! Spindle_ReadReferences->isRunning() ) {
				log(Error) << "Spindle component is not running yet, please start this component first" << endlog();
			}
			else {
				Spindle_ReadReferences->stop(); //Disabling reading of references. Will be enabled automagically at the end by the supervisor.
			}
		}
		
	starttime = os::TimeService::Instance()->getNSecs()*1e-9;
	log(Warning)<<"SpindleHomingAdmittance::started at " << os::TimeService::Instance()->getNSecs()*1e-9 <<endlog();

	return true;
}

void SpindleHomingAdmittance::updateHook()
{   	
	//log(Warning)<<"Spindle: Reading endswitch"<<endlog();
	std_msgs::Bool endswitch;
	endswitch_inport.read(endswitch);
	
	//log(Warning)<<"Spindle: Sending reference"<<endlog();
	if ( homed == false )
	{	
		double oldreference = reference;
		reference = oldreference + referencestep;
		ref[0] = reference;
		//log(Warning) << "Reference sent by spindle homing:"<< reference << endlog();
		ref_outport.write(ref);
	}
	//log(Warning)<<"Spindle: Checking whether homed = true"<<endlog();
	if ( !endswitch.data && homed == false )
	{
		ROS_INFO_STREAM( "Spindle is homed." );
		homed = true;

		// Actually call the services
		//log(Warning)<<"SpindleHomingAdmittance::start calling services at " << os::TimeService::Instance()->getNSecs()*1e-9 - starttime <<endlog();
		StopBodyPart("spindle");
		ResetEncoder(0,stroke);
		StartBodyPart("spindle");
		//log(Warning)<<"SpindleHomingAdmittance::finshed calling services at " << os::TimeService::Instance()->getNSecs()*1e-9 - starttime <<endlog();
		
		// Got to desired position
		ref[0] = endpos;
		
		ref_outport.write(ref);
		this->stop(); 
	}
	
	if (homed == true && cntr < 5000) {
		cntr++;
	}
	
	if ( homed == true && (cntr >= 5000 )){
		
		homingfinished_outport.write(true);
		//log(Warning)<<"Spindle: stop component"<<endlog();
		//Should not happen only if homed == true is defined in ops file
		this->stop(); 
	}
}



ORO_CREATE_COMPONENT(SpindleHomingAdmittance)
