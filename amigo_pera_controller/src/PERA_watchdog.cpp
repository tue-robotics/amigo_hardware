/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0 
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>
#include <std_msgs/Bool.h>
#include <amigo_msgs/arm_joints.h>
#include <amigo_msgs/pera_status.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include "PERA_watchdog.hpp"

#define PI 3.1415926535897932384626433

using namespace RTT;
using namespace PERA;

WATCHDOG::WATCHDOG(const string& name) : 
	TaskContext(name, PreOperational)
{
	addPort("requestedJointAnglesPort",reqJntAngPort).doc("Receives joint coordinates from a ROS topic");
	addPort("enablePort",enablePort).doc("Sends enablesignal to PERA_IO");
	addPort("errorPort",jointErrorsPort).doc("Receives joint control errors");
	addPort("measRelJointAnglesPort",mRelJntAngPort).doc("Receives measured relative joint angles");
	addPort("measAbsJointAnglesPort",mAbsJntAngPort).doc("Receives measured absolute joint angles");
	addPort("eButtonPort",eButtonPort).doc("Receives emergency button signal from Soem");
	addPort("resetInterpolatorPort",resetIntPort).doc("Sends resetvalues to the ReferenceInterpolator");
	addPort("resetRefPort",resetRefPort).doc("Sends reset joint coordinates to ROS topic");
	addPort("homJntAnglesPort",homJntAngPort).doc("Sends the requested angles for homing to the ReferenceInterpolator");
	addPort("reNullPort",reNullPort).doc("Sends signal to PERA_IO to renull at actual position");
	addPort("enableReadRefPort",enableReadRefPort).doc("Sends enable signal to ReadReference allow reading of joint angles from ROS topic");
	addPort("gripperClosePort",gripperClosePort).doc("Requests gripper open/close at GripperController");
	addPort("gripperStatusPort",gripperStatusPort).doc("Receives gripper status from GripperController");
	addPort("gripperResetPort",gripperResetPort).doc("Requests for GripperController reset");
	addPort("peraStatusPort",peraStatusPort).doc("Sends amigo_msgs::pera_status message containing PERA status information");

	addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
	addProperty( "enableOutput", ENABLE_PROPERTY ).doc("Specifies if PERA_IO should be enabled");
	addProperty( "jointUpperBounds", UPPERBOUNDS ).doc("Inverse kinematics upper bound");
	addProperty( "jointLowerBounds", LOWERBOUNDS ).doc("Inverse kinematics lower bound");
	addProperty( "resetAngles", RESETANGLES ).doc("Joint angles to be published when emergency button is released");
	addProperty( "homedPos", HOMEDPOS ).doc("Homing positions for the joints");
	addProperty( "absOrRel", ABS_OR_REL ).doc("Defines whether joint is homed using absolute sensors or using mechanical endstop");
	addProperty( "absSenDir", ABS_SEN_DIR ).doc("Defines if absolute sensor has its positive direction the same or opposite to relative sensors");
	addProperty( "stepSize", STEPSIZE ).doc("Speed the joint moves to its mechanical endstop during homing");
	addProperty( "requireHoming", REQUIRE_HOMING ).doc("Specifies if the arm will home yes or no");
	addProperty( "startJoint", STRT_JNT ).doc("Joint number to start homing with");
	
}

WATCHDOG::~WATCHDOG(){}

bool WATCHDOG::configureHook()
{
	
	Logger::In in("WATCHDOG"); 
	
	jointAngles.resize(7);
	jointErrors.resize(8);
	homJntAngles.resize(8);
	
	// Set loopcounters to zero initially
	cntr=0;
	cntr2=0;
	
	// Errors is false by default
	errors=false;
	
	// Gripper initially not homed by default
	gripperHomed = true;
	
	// Reference not yet resetted
	resetReference=false;
	
	// Assign ENABLE_PROPERTY to value enable
	enable=ENABLE_PROPERTY;
	
	// Write enable value to PERA_IO
	enablePort.write(enable);
	
	// goodToGo true means homing can proceed to next joint
	goodToGo = true;
	prevJntNr = 4;
	
	// Set homed to false by default
	if(REQUIRE_HOMING){
		homed = false;
	}
	else if(!REQUIRE_HOMING){
		homed = true;
	}
	
	// Set the initial jnt for homingprocedure
	jntNr=8;
	
	// Pressed is true. No SOEM heartbeat means no amplifier enabling.
	pressed = true;
	
	return true;
}

bool WATCHDOG::startHook()
{
	
	// Wait for SOEM heartbeat.
	while(!(eButtonPort.read(eButtonPressed) == NewData)){
		sleep(1);
		log(Info)<<"WATCHDOG: Waiting for enable-signal from the emergency button"<<endlog();
		cntr++;
		if( cntr == 10 && cntr<3 ){
			log(Warning)<<"WATCHDOG: no signal from emergency button (SOEM). Is SOEM running?"<<endlog();
			cntr = 0;
			cntr2++;
		}
		else if( cntr==3 ){
			log(Error)<<"WATCHDOG: no SOEM heartbeat. Shutting down LPERA."<<endlog();
			cntr=0;
			cntr2=0;
			return false;
		}
	}
	
	if(!REQUIRE_HOMING){
		bool enableReadRef = true;
		enableReadRefPort.write(enableReadRef);
	}
		
	log(Info)<<"WATCHDOG: configured and running."<<endlog();

    return true;
}

/*!
 * First it is monitored if the emergency button is pressed. If this is 
 * the case (eButtonPressed = false) then the amplifiers are disabled. 
 * Also for every instance the interpolators are ordered to reset at the
 * measured joint angles to prevent the jointerror from increasing. If 
 * the emergency button is released (eButtonPressed = true) the 
 * jointerrors are monitored if they are within the specified limits and 
 * the joint angles requested by the inverse kinematics are checked if 
 * they are within the feasible range of the joints. If either of these 
 * go outside their bounds the amplifiers are disabled. 
 */
void WATCHDOG::updateHook()
{
	if(ENABLE_PROPERTY){
		// Read emergency-button state
		eButtonPort.read(eButtonPressed);
			
		if(!eButtonPressed.data){
			if(pressed!=false){
				log(Info)<<"WATCHDOG: Emergency Button Released"<<endlog();
			}
			pressed = false;
		}
		else if(eButtonPressed.data){
			if(pressed!=true){
				log(Info)<<"WATCHDOG: Emergency Button Pressed"<<endlog();
			}
			pressed = true;
		}	
		
		if(!pressed){

			/* Set enable to true. In case of an error, it will be set to 
			 * false. This construction is used to make sure that enable
			 * can NEVER become true once an error has occured but CAN become
			 * true after unplugging the eButton.
			 */
			if(!errors){
				enable = true;
			}
			
			// Set the value to false, otherwise if eButtonPressed.data is
			// true the reference will not be reset.
			resetReference = false;
		
			reqJntAngPort.read(jointAngles);
			jointErrorsPort.read(jointErrors);
			
			// Check if joint angle requested by inv. kin. are within limits.
			for(unsigned int i = 0;i<7;i++){
				
				if(!(jointAngles[i]>=LOWERBOUNDS[i] && jointAngles[i]<=UPPERBOUNDS[i])){ 
									
					enable=false;
					
					if(jointAngles[i]>UPPERBOUNDS[i] && errors == false){
						log(Error)<<"WATCHDOG: Joint q"<<i+1<<" exceeded upper limit ("<<UPPERBOUNDS[i]<<"). PERA output disabled."<<endlog();
						errors = true;
					}
					else if(jointAngles[i]<LOWERBOUNDS[i] && errors == false){
						log(Error)<<"WATCHDOG: Joint q"<<i+1<<" exceeded lower limit ("<<LOWERBOUNDS[i]<<"). PERA output disabled."<<endlog();
						errors = true;
					}
					
				}
				
			}
			
			// Check if joint errors are within specified limits
			for(unsigned int i = 0;i<8;i++){

				// If the error is too large and corresponding joint is NOT being homed -> stop PERA IO
				if( (fabs(jointErrors[i])>MAX_ERRORS[i]) && (jntNr!=i+1) ){
					
					enable = false;
					
					if( errors == false ){
						log(Error)<<"WATCHDOG: Error of joint q"<<i+1<<" exceeded limit ("<<MAX_ERRORS[i]<<"). PERA output disabled."<<endlog();
						errors = true;
					}		
					
				}
				
			}
			
			/* Check if homing is completed. Else, request for homing angles and
			 * disable the reading of the reference joint angles from the
			 * ROS inverse kinematics.
			 */
			//Comment lines below up to next marker in case of NO homing
			if(enable && !homed && !errors){

				doubles measRelJntAngles(8,0.0);
				ints measAbsJntAngles(7,0.0);
				doubles homJntAngTemp(7,0.0);

				// Measure the abs en rel angles.
				mRelJntAngPort.read(measRelJntAngles);
				mAbsJntAngPort.read(measAbsJntAngles);
				
				if(cntr==0){
					
					for(unsigned int i = 0;i<7;i++){
						homJntAngles[i]=measRelJntAngles[i];
					}
					
					// Disable the reading of the reference joint angles.
					bool enableReadRef = false;
					enableReadRefPort.write(enableReadRef);
					
					cntr++;
				}

				// Compute the joint angles for the homing procedure.
				homJntAngTemp = homing(jointErrors,measAbsJntAngles,homJntAngles,measRelJntAngles);
				
				for(unsigned int i = 0;i<8;i++){
					homJntAngles[i]=homJntAngTemp[i];
				}

				// Forward computed homing angles to the ReferenceInterpolator
				homJntAngPort.write(homJntAngles);
						
			}
			
			enablePort.write(enable);
			
		}
		else if(pressed){
			doubles resetdata(32,0.0);
			doubles measRelJntAngles(8,0.0);
			amigo_msgs::arm_joints jointResetData;
			
			// Set enable to false and write it to the PERA_IO component.
			enable = false;
			enablePort.write(enable);
			
			// Read angles from PERA angles from IO
			mRelJntAngPort.read(measRelJntAngles);
			
			// Fill up resetdata
			for(unsigned int i = 0;i<8;i++){
				resetdata[i*4]=1.0;
				resetdata[i*4+1]=measRelJntAngles[i];
				resetdata[i*4+2]=0.0;
				resetdata[i*4+3]=0.0;
			}
			
			// Reset the ROS inv. kin. topic
			if( !resetReference ){
				for(unsigned int i = 0;i<7;i++){
					jointResetData.pos[i].data=RESETANGLES[i];
				}
				resetRefPort.write(jointResetData);
				resetReference = true;
			}
			
			// Write the new angles to the interpolator for reset such that interpolator will follow the arm position causing no jump in the error
			resetIntPort.write(resetdata);
			
		}	

	}
	else if(!ENABLE_PROPERTY){
		
		enable = false;
		enablePort.write(enable);
		
	}
	
	amigo_msgs::pera_status status;
	status=updateStatus(jointErrors);
	peraStatusPort.write(status);
	
}

doubles WATCHDOG::homing(doubles jointErrors, ints absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles){
	
	if(!gripperHomed){
		
		std_msgs::Bool gripperClose;
		gripperClose.data = true;
		gripperClosePort.write(gripperClose);
		//log(Warning)<<"WATCHDOG: order gripper to close"<<endlog();
		
		std_msgs::Bool gripperStatus;
		gripperStatusPort.read(gripperStatus);
		
		if(gripperStatus.data){
			log(Warning)<<"WATCHDOG: gripper homed"<<endlog();
			gripperHomed = true;
			jntNr=STRT_JNT;
		}
		
	}
	
	if(jntNr!=0 && goodToGo && gripperHomed){
	
		// If true the homing will be done using abs sensor
		if(ABS_OR_REL[jntNr-1]==0){
			
			// Homing position not reached yet
			if(fabs(HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=1.0){
				
				// Are positive directions absolute sensors and joints the same?
				if(ABS_SEN_DIR[jntNr-1]==1.0){
					
					if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]+=0.0007; //go forward fast
						log(Info)<<"WATCHDOG: 1.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]-=0.0007; //go back fast
						log(Info)<<"WATCHDOG: 1.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]+=0.00017; //go forward slowly
						log(Info)<<"WATCHDOG: 2.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]-=0.00017; //go back slowly
						log(Info)<<"WATCHDOG: 2.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					
					
				}
				// Are positive directions absolute sensors and joints opposite?
				else if(ABS_SEN_DIR[jntNr-1]==-1.0){
								
					if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]-=0.0007; //go forward fast 
						log(Info)<<"WATCHDOG: -1.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]+=0.0007; //go back fast
						log(Info)<<"WATCHDOG: -1.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]-=0.00017; //go forward slowly
						log(Info)<<"WATCHDOG: -2.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]+=0.00017; //go back slowly
						log(Info)<<"WATCHDOG: -2.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					
				}
				
			}
			// Homing position reached
			else if(fabs(HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<1.0){
				goodToGo = false;
			}
			
		}
		// If true the homing will be done using rel encoders
		else if(ABS_OR_REL[jntNr-1]==1){
			
			// If the mechanical endstop is not reached
			if( fabs(jointErrors[jntNr-1]) < (MAX_ERRORS[jntNr-1]-0.0017) ){
				tempHomJntAngles[jntNr-1]-=STEPSIZE;
			}

			// If the mechanical endstop is reached (error to large)
			else if( fabs(jointErrors[jntNr-1]) >= (MAX_ERRORS[jntNr-1]-0.0017) ){
				
				// From the mechanical endstop move back to homing position
				tempHomJntAngles[jntNr-1]=measRelJntAngles[jntNr-1]+HOMEDPOS[jntNr-1];
				
				// Reset the interpolator for jnt jntNr to the position it is at
				doubles resetdata(32,0.0);
				resetdata[(jntNr-1)*4]=1.0;
				resetdata[(jntNr-1)*4+1]=measRelJntAngles[jntNr-1];
				resetIntPort.write(resetdata);
				
				// Proceed to next joint
				goodToGo = false;
				
			}	
				
		}
		
	}
	
	else if(!goodToGo && gripperHomed){
		
		// If joint is homed using abs sens no waiting time is required
		if( (ABS_OR_REL[jntNr-1]==0 && jntNr!=1) || (ABS_OR_REL[jntNr-1]==1 && jntNr==4) ){
			jntNr--;
			log(Info)<<"WATCHDOG: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo = true;
		}
		// If joint is moved to endstop using rel enc waiting time is required to move to homing position
		else if(ABS_OR_REL[jntNr-1]==1 && cntr2<1500 && jntNr!=1 && jntNr!=4){
			
			cntr2++;
			
		}
		// If waiting is complete move on to next joint
		else if(ABS_OR_REL[jntNr-1]==1 && cntr2==1500 && jntNr!=1 && jntNr!=4){
			
			cntr2=0;
			jntNr--;
			log(Info)<<"WATCHDOG: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo=true;
			
		}
		// If homed joint is last one, reset interpolator and PERA_USB_IO
		else if(jntNr==1){
			
			if(cntr2>=0 && cntr2<5){
				cntr2++;
				enable = false;
				enablePort.write(enable);
			}
			
			else if(cntr2==5){
				
				// Reset the referenceInterpolator and PERA_IO.
				doubles resetdata(32,0.0);
				
				for(unsigned int i = 0;i<8;i++){
					tempHomJntAngles[i]=0.0;
				}
				
				// Null the PERA_IO.
				bool reNull = true;
				reNullPort.write(reNull);
				log(Info)<<"WATCHDOG: Renulled PERA_IO \n"<<endlog();
				
				// Enable the reading of the reference joint angles.
				bool enableReadRef = true;
				enableReadRefPort.write(enableReadRef);
				
				// Fill up resetdata
				for(unsigned int i = 0;i<8;i++){
					resetdata[i*4]=1.0;
					resetdata[i*4+1]=0.0;
					resetdata[i*4+2]=0.0;
					resetdata[i*4+3]=0.0;
				}
				
				// Reset the reference interpolator to zero
				resetIntPort.write(resetdata);
				
				// Reset the gripper controller position to zero
				gripperResetPort.write(true);
				
				cntr2++;
				
			}
			else if(cntr2>=6 && cntr2<50){
				cntr2++;
			}
			else if(cntr2==50){
				
				// Enable PERA IO
				enable = true;
				enablePort.write(enable);
				
				// Enable homing for next joint
				goodToGo = true;
				
				// Reset counter for next round
				cntr2 = 0;
				
				// Move towards next joint
				jntNr--;
				
				// Set homing to true
				homed = true;

				log(Info)<<"WATCHDOG: Finished homing \n"<<endlog();
				
			}			
			
		}
				
	}
	
	return tempHomJntAngles;
	
}

amigo_msgs::pera_status WATCHDOG::updateStatus(doubles jointErrors){
	
	amigo_msgs::pera_status tempStatus;
	tempStatus.error.data=errors;
	tempStatus.homed.data=homed;
	tempStatus.ebutton_pressed.data=eButtonPressed.data;
	
	std_msgs::Bool gripperStatus;
	gripperStatusPort.read(gripperStatus);
	if(gripperStatus.data){
		tempStatus.gripper.data="Closed";
	}
	else{
		tempStatus.gripper.data="Open";
	}
	
	for(unsigned int i = 0;i<8;i++){
			tempStatus.jnt_errors[i].data=jointErrors[i];
	}
	
	tempStatus.enable.data=enable;
	
	return tempStatus;
	
}

ORO_CREATE_COMPONENT(PERA::WATCHDOG)
