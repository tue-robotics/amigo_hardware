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
  addProperty( "homing_body", homing_body );     	// name of the body that is stopped/started during homing procedure
  addProperty( "homing_type", homing_type );     	// 0 = abs sen homing 1 = servo error homing  2 = Force sen homing  3 = endSwitch homing
  addProperty( "homing_order", homing_order );   	// Order in which the joints are homed
  addProperty( "homing_refPos", homing_refPos ); 	// Pos Reference for the homing joint
  addProperty( "homing_refVel", homing_refVel ); 	// Vel Reference for the homing joint
  addProperty( "homing_stroke", homing_stroke ); 	// Stroke from zero point to homing point (encoders are resetted using this value)
  addProperty( "homing_midpos", homing_midpos ); 	// position that the body should have during homing. To avoid collisions with other bodies/ itself
  addProperty( "homing_endpos", homing_endpos ); 	// position that the body should go to after homing is finished. (could be same as midpos)
  addProperty( "homing_absPos", homing_absPos ); 	// homing criterion for abs sen homing
  addProperty( "homing_force", homing_force );   	// homing criterion for Force sen homing
  addProperty( "homing_error", homing_error );   	// homing criterion for servo error homing
  addProperty( "require_homing", require_homing ); 	// require homing
}

Homing::~Homing(){}

bool Homing::configureHook()
{	homed = !require_homing;
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
  return true;
}

void Homing::updateHook()
{
	if ( HomingConstraintMet == false && homed == false && GoToMidPos == false )
    {
		int homing_order_tt = homing_order[JntNr-1];
		homing_order_t = homing_order_tt;

        ref[homing_order_t-1][0] = homing_refPos[homing_order_t-1];
        ref[homing_order_t-1][1] = homing_refVel[homing_order_t-1];
        ref[homing_order_t-1][2] = 0.0;  
       
        ref_outport.write(ref);
  
   		int homing_type_t = homing_type[homing_order_t-1];
        switch (homing_type_t) {
            case 0 : 
            {
				//log(Warning)<< "SPINDLEHOMING CASE 0"  <<endlog(); // remove this warning if case is tested
                if (abs(absPos[homing_order_t-1]-homing_absPos[homing_order_t-1]) <= 1.0) 
                {
                    log(Warning)<< homing_body << ": Absolute sensor reached goal of joint :" << homing_order_t <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 1 : 
            {
				//log(Warning)<< "SPINDLEHOMING CASE 1"  <<endlog(); // remove this warning if case is tested
                servoError_inport.read(servoErrors);
                if (fabs(servoErrors[homing_order_t-1]) >= homing_error[homing_order_t-1])
                {
                    log(Warning)<< homing_body << ": Servo error exceeded threshold, endstop reached of joint:" << homing_order_t <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 2 : 
            {
				//log(Warning)<< "SPINDLEHOMING CASE 2"  <<endlog(); // remove this warning if case is tested
                force_inport.read(forces);
                if (fabs(forces[homing_order_t-1]) >= homing_force[homing_order_t-1]) 
                {
                    log(Warning)<< homing_body << ": Force Sensor measured threashold value" << homing_order_t <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 3 : 
            {
				endSwitch_inport.read(endSwitch);
                if (!endSwitch.data) 
                {
					log(Warning)<< homing_body << ": Endswitch reached of joint:" << homing_order_t <<endlog();
					HomingConstraintMet = true; 
                }
            }
        }
    }

    if ( HomingConstraintMet == true && homed == false )
    {
        // Actually call the services
        StopBodyPart(homing_body);
        ResetEncoder((homing_order_t-1),homing_stroke[homing_order_t-1]);
        StartBodyPart(homing_body);
				
        // send body joint to midpos, joints that are not homed yet are kept at zero        
        ref[homing_order_t-1][0] = homing_midpos[homing_order_t-1];
        ref[homing_order_t-1][1] = 0.0;
        ref[homing_order_t-1][2] = 0.0;
        ref_outport.write(ref);
        
        HomingConstraintMet = false;
        GoToMidPos = true;                            
    }

    if ( GoToMidPos && homed == false )  {              // this loop is used to send the joint to a position where it does not hinder other joints that needs to be homded of the same body part
		relPos_inport.read(relPos);        
        if ( fabs(relPos[homing_order_t-1]-homing_midpos[homing_order_t-1]) <= 0.01) {
			GoToMidPos = false;     
			//log(Warning)<< homing_body << ": Reached MidPos of joint:" << homing_order_t <<endlog();
			homing_order_t++;
			homed = true;
        }
    }

    if ( homing_order_t == (N + 1) && (!GoToMidPos) ) // if Last Jnt is homed and mid pos is reached for the last joint go to end pos
    {
		for (uint j = 0; j < N; j++){
			ref[j][0] = homing_endpos[j]; // To do check for all joints, 
			ref[j][1] = 0.0;
			ref[j][2] = 0.0;
		}
		
        ref_outport.write(ref);
        
		relPos_inport.read(relPos);
        if ( fabs(relPos[0]-homing_endpos[0]) <= 0.01) { 
			log(Warning)<< "Finished homing of body:" << homing_body <<endlog();

			// Stop this component.
			this->stop(); // stop Homing component when Homing is finished
        }     
    }
}

ORO_CREATE_COMPONENT(Homing)
