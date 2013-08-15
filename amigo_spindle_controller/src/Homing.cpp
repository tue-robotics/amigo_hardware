#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "Homing.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace AMIGO;


Homing::Homing(const string& name) : TaskContext(name, PreOperational)
  {
  addEventPort( "endswitch", endSwitch_inport );
  addPort( "abs_pos_in", absPos_inport );
  addPort( "force_in", force_inport );
  addPort( "servo_error_in", servoError_inport );
  addPort( "ref_out", ref_outport );
  addPort( "relPos_in", relPos_inport );

  // Creating variables
  addProperty( "homing_body", homing_body );     // name of the body that is stopped/started during homing procedure
  addProperty( "homing_type", homing_type );     // 0 = abs sen homing 1 = servo error homing  2 = Force sen homing  3 = endSwitch homing
  //addProperty( "homing_order", homing_order );   // TO DO : add different homing order
  addProperty( "homing_refPos", homing_refPos ); // Pos Reference for the homing joint
  addProperty( "homing_refVel", homing_refVel ); // Vel Reference for the homing joint
  addProperty( "homing_stroke", homing_stroke ); // Stroke from zero point to homing point (encoders are resetted using this value)
  addProperty( "homing_midpos", homing_midpos ); // position that the body should have during homing. To avoid collisions with other bodies/ itself
  addProperty( "homing_endpos", homing_endpos ); // position that the body should go to after homing is finished. (could be same as midpos)
  addProperty( "homing_absPos", homing_absPos ); // homing criterion for abs sen homing
  addProperty( "homing_force", homing_force );   // homing criterion for Force sen homing
  addProperty( "homing_error", homing_error );   // homing criterion for servo error homing
}

Homing::~Homing(){}

bool Homing::configureHook()
{	homed = false;
    N = homing_refPos.size();
    ref.resize(N); //N joint(s)
    for( unsigned int j = 0; j < N; j++ )
    {
        ref[j].resize(3); //pos, vel, acc
    }
    
    // Lookup the Supervisor component.
	TaskContext* Supervisor = this->getPeer("Supervisor");
	if ( !Supervisor ) {
        log(Error) << "Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog(); // TODO check with Tim if removed checks are really neccesary and if these can be made generic
		return false;
	}			
		// Lookup the Encoder component.
	TaskContext* SpindleReadEncoder = this->getPeer("SpindleReadEncoder");		//To Do make generic
	if ( !SpindleReadEncoder ) {
		log(Error) << "Could not find SpindleReadEncoder component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
	}	
	// Lookup the Setpoint component.
	TaskContext* SpindleReadSetpoint = this->getPeer("SpindleReadSetpoint");		//To Do make generic
	if ( !SpindleReadSetpoint ) {
		log(Error) << "Could not find SpindleReadSetpoint component! Did you add it as Peer in the ops file?"<<endlog();
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
		log(Error) << "Could not find SpindleReadEncoder.reset Operation!"<<endlog();
		return false;
	}	
	
	
    return true;  
}

bool Homing::startHook()
{ 
    JntNr = 1;
    HomingConstraintMet = false;
    GoToMidPos = false;

    if ( !homed ) {
        TaskContext* SpindleReadSetpoint = this->getPeer("SpindleReadSetpoint");
        if ( ! SpindleReadSetpoint->isRunning() ) {
            log(Error) << "Spindle component is not running yet, please start this component first" << endlog();
        }
        else {
            SpindleReadSetpoint->stop(); //Disabling reading of references. Will be enabled automagically at the end by the supervisor.
        }
    }
    else {
        this->stop(); //Nothing to do
    }

  log(Warning)<<"SpindleHoming::started at " << os::TimeService::Instance()->getNSecs()*1e-9 <<endlog();

  return true;
}

void Homing::updateHook()
{
	if (HomingConstraintMet == false)
    {
        ref[JntNr-1][0] = homing_refPos[JntNr-1];
        ref[JntNr-1][1] = homing_refVel[JntNr-1];
        ref[JntNr-1][2] = 0.0;  
        ref_outport.write(ref);
        
		int homing_type_t = homing_type[JntNr-1];
        switch (homing_type_t) {
            case 0 : 
            {
				log(Warning)<< "SPINDLEHOMING CASE 0"  <<endlog();
                if (abs(absPos[JntNr-1]-homing_absPos[JntNr-1]) <= 1) 
                {
                    log(Warning)<< "Homed a joint using absPos homing"  <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 1 : 
            {
				log(Warning)<< "SPINDLEHOMING CASE 1"  <<endlog();
                servoError_inport.read(servoErrors);
                if (fabs(servoErrors[JntNr-1]) >= homing_error[JntNr-1])
                {
                    log(Warning)<< "Homed a joint using endstop homing"  <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 2 : 
            {
				log(Warning)<< "SPINDLEHOMING CASE 2"  <<endlog();
                force_inport.read(forces);
                if (fabs(forces[JntNr-1]) >= homing_force[JntNr-1]) 
                {
                    log(Warning)<< "Homed a joint using forceSensor homing"  <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 3 : 
            {
				endSwitch_inport.read(endSwitch);
                if (!endSwitch.data) 
                {
					log(Warning)<< "Endswitch reached 3"  <<endlog();
					HomingConstraintMet = true; 
                }
            }
        }
    }

    if (HomingConstraintMet == true)
    {
        ROS_INFO_STREAM( "Joint X is homed" );
        

        
        relPos_inport.read(relPos);
		log(Warning)<< "relPos[0] before:" << relPos[0]  <<endlog();        
        // Actually call the services
        StopBodyPart(homing_body);
        ResetEncoder((JntNr-1),homing_stroke[JntNr-1]);
        StartBodyPart(homing_body);
		relPos_inport.read(relPos);
		log(Warning)<< "relPos[0] after:" << relPos[0]  <<endlog();
				
        // send body joint to midpos, joints that are not homed yet are kept at zero
        
        ref[JntNr-1][0] = homing_midpos[JntNr-1];
        ref[JntNr-1][1] = 0.0;
        ref[JntNr-1][2] = 0.0;
        ref_outport.write(ref);
        
        HomingConstraintMet = false;
        GoToMidPos = true;  
                          
    }

    if (GoToMidPos)  {              // this loop is used to send the joint to a position where it does not hinder other joints that needs to be homded of the same body part
		//log(Warning)<< "GoingToMidPos:" << homing_body  <<endlog(); 
        relPos_inport.read(relPos);        
        //log(Warning)<< "relPos[0]" << relPos[0]  <<endlog();
        //log(Warning)<< "homing_midpos[JntNr-1]" << homing_midpos[JntNr-1] <<endlog();
        //log(Warning)<< "relPos[JntNr-1] - homing_midpos[JntNr-1]" << (relPos[JntNr-1]-homing_midpos[JntNr-1]) <<endlog();
        //log(Warning)<< "fabs(relPos[JntNr-1]-homing_midpos[JntNr-1])" << fabs(relPos[JntNr-1]-homing_midpos[JntNr-1]) <<endlog();
        //log(Warning)<< "( fabs(relPos[JntNr-1]-homing_midpos[JntNr-1]) <= 0.1)" << ( fabs(relPos[JntNr-1]-homing_midpos[JntNr-1]) <= 0.1) <<endlog();

        if ( fabs(relPos[JntNr-1]-homing_midpos[JntNr-1]) <= 0.1) {
			GoToMidPos = false;     
			JntNr++;
			log(Warning)<< "Set => GoToMidPos = false: JntNr = " << JntNr <<endlog();
        }
    }

    if ( JntNr == (N + 1) && (!GoToMidPos) ) // if Last Jnt is homed and mid pos is reached for the last joint go to end pos
    {
		log(Warning)<< "Going to EndPos" <<endlog();
		
        ref[0][0] = homing_endpos[0]; // To do fix for all joints
        ref[0][1] = 0.0;
        ref[0][2] = 0.0;
        ref_outport.write(ref);
        
		relPos_inport.read(relPos);
        if ( fabs(relPos[0]-homing_endpos[0]) <= 0.1) {
			homed = true;
			log(Warning)<< "Finished homing of body:" << homing_body  <<endlog();

			// Stop this component.
			this->stop(); // stop Homing component when Homing is finished
        }     
    }
}

ORO_CREATE_COMPONENT(Homing)
