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
#include <vector>
#include <math.h>
#include "PERA_watchdog.hpp"

#define PI 3.1415926535897932384626433

using namespace RTT;
using namespace PERA;

WATCHDOG::WATCHDOG(const string& name) : 
	TaskContext(name, PreOperational)
{
	addPort("requestedJointAnglesPort",reqJntAngPort).doc("");
	addPort("enablePort",enablePort).doc("");
	addPort("errorPort",jointErrorsPort).doc("");
	addPort("measRelJointAnglesPort",mRelJntAngPort).doc("");
	addPort("measAbsJointAnglesPort",mAbsJntAngPort).doc("");
	addPort("eButtonPort",eButtonPort).doc("");
	addPort("resetInterpolatorPort",resetIntPort).doc("");
	addPort("resetRefPort",resetRefPort);
	addPort("homJntAnglesPort",homJntAngPort).doc("");
	addPort("reNullPort",reNullPort).doc("");
	addPort("enableReadRefPort",enableReadRefPort).doc("");

	
	addProperty( "maxJointErrors", maxErrors).doc("");
	addProperty( "enableOutput", enableProperty ).doc("");
	addProperty( "jointUpperBounds", upperBounds ).doc("");
	addProperty( "jointLowerBounds", lowerBounds ).doc("");
	addProperty( "resetAngles", resetAngles ).doc("");
	addProperty( "homedPos", homedPos ).doc("");
	addProperty( "absOrRel", absOrRel ).doc("");
	addProperty( "absSenDir", absSenDir ).doc("");
	addProperty( "stepSize", stepSize ).doc("");
	addProperty( "requireHoming", requireHoming ).doc("");
	
}

WATCHDOG::~WATCHDOG(){}

bool WATCHDOG::configureHook()
{
	
	Logger::In in("WATCHDOG"); 
	
	jointAngles.resize(7);
	jointErrors.resize(8);
	homJntAngles.resize(8);
	cntr=0;
	cntr2=0;
	errors=false;
	resetReference=false;
	enablePort.write(enable);
	goodToGo = true;
	prevJntNr = 4;
	
	// Set homed to false by default
	if(requireHoming){
		homed = false;
	}
	else if(!requireHoming){
		homed = true;
	}
	
	// Set the initial jnt for homingprocedure
	jntNr=5;
	
	/* Pressed is true by default to ensure that the SOEM is running
	 * upon enabling PERA_USB_IO. Otherwise PERA_USB_IO could be enabled
	 * while SOEM is not running which will cause undesired behavior of
	 * the PERA when the eButton is unplugged after pressing it.
	 */
	pressed = true;
	
	return true;
}

bool WATCHDOG::startHook()
{
	
	while(!(eButtonPort.read(eButtonPressed) == NewData)){
		sleep(1);
		log(Info)<<"WATCHDOG: Waiting for enable-signal from the emergency button"<<endlog();
		cntr++;
		if(cntr == 10){
			log(Error)<<"WATCHDOG: no signal from emergency button (SOEM). Is SOEM running?"<<endlog();
			cntr = 0;
		}
	}
		
	log(Info)<<"WATCHDOG: now running and monitoring: \n - Feasibility requested joint angles \n - Control error"<<endlog();

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
	if(enableProperty){
	
		// Read emergency-button state
		eButtonPort.read(eButtonPressed);
			
		if(!eButtonPressed.data){
			pressed = false;
		}
		else if(eButtonPressed.data){
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
				
				if(!(jointAngles[i]>=lowerBounds[i] && jointAngles[i]<=upperBounds[i])){ 
									
					enable=false;
					
					if(jointAngles[i]>upperBounds[i] && errors == false){
						log(Error)<<"WATCHDOG: Joint q"<<i+1<<" exceeded upper limit ("<<upperBounds[i]<<"). PERA output disabled."<<endlog();
						errors = true;
					}
					else if(jointAngles[i]<lowerBounds[i] && errors == false){
						log(Error)<<"WATCHDOG: Joint q"<<i+1<<" exceeded lower limit ("<<lowerBounds[i]<<"). PERA output disabled."<<endlog();
						errors = true;
					}
					
				}
				
			}
			
			// Check if joint errors are within specified limits
			for(unsigned int i = 0;i<8;i++){

				// If the error is too large and corresponding joint is NOT being homed -> stop PERA IO
				if( (fabs(jointErrors[i])>maxErrors[i]) && (jntNr!=i+1) && homed ){
					
					enable = false;
					
					if( errors == false ){
						log(Error)<<"WATCHDOG: Error of joint q"<<i+1<<" exceeded limit ("<<maxErrors[i]<<"). PERA output disabled."<<endlog();
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
				doubles homJntAngTemp(8,0.0);

				// Measure the abs en rel angles.
				mRelJntAngPort.read(measRelJntAngles);
				mAbsJntAngPort.read(measAbsJntAngles);
				
				if(cntr==0){
					for(unsigned int i = 0;i<8;i++){
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
					//log(Warning)<<"Request homingangle"<<homJntAngles[i]<<" for joint q"<<i+1<<endlog();
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
					jointResetData.pos[i].data=resetAngles[i];
				}
				resetRefPort.write(jointResetData);
				resetReference = true;
			}
			
			// Write the new angles to the interpolator for reset such that interpolator will follow the arm position causing no jump in the error
			resetIntPort.write(resetdata);
			
		}	

	}
	else if(!enableProperty){
		
		enable = false;
		enablePort.write(enable);
		
	}
	
}

doubles WATCHDOG::homing(doubles jointErrors, ints absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles){
	
	if(jntNr!=0 && goodToGo){
	
		// If true the homing will be done using abs sensor
		if(absOrRel[jntNr-1]==0){
			
			// Homing position not reached yet
			if(fabs(homedPos[jntNr-1]-absJntAngles[jntNr-1])>=1.0){
				
				// Are positive directions absolute sensors and joints the same?
				if(absSenDir[jntNr-1]==1.0){
					
					if((homedPos[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]+=0.0007; //go forward fast
						log(Info)<<"WATCHDOG: 1.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((homedPos[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]-=0.0007; //go back fast
						log(Info)<<"WATCHDOG: 1.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((homedPos[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (homedPos[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]+=0.00017; //go forward slowly
						log(Info)<<"WATCHDOG: 2.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((homedPos[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (homedPos[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]-=0.00017; //go back slowly
						log(Info)<<"WATCHDOG: 2.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					
					
				}
				// Are positive directions absolute sensors and joints opposite?
				else if(absSenDir[jntNr-1]==-1.0){
								
					if((homedPos[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]-=0.0007; //go forward fast 
						log(Info)<<"WATCHDOG: -1.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((homedPos[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]+=0.0007; //go back fast
						log(Info)<<"WATCHDOG: -1.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((homedPos[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (homedPos[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]-=0.00017; //go forward slowly
						log(Info)<<"WATCHDOG: -2.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((homedPos[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (homedPos[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]+=0.00017; //go back slowly
						log(Info)<<"WATCHDOG: -2.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					
				}
				
			}
			// Homing position reached
			else if(fabs(homedPos[jntNr-1]-absJntAngles[jntNr-1])<1.0){
				goodToGo = false;
			}
			
		}
		// If true the homing will be done using rel encoders
		else if(absOrRel[jntNr-1]==1){
			
			// If the mechanical endstop is not reached
			if( fabs(jointErrors[jntNr-1]) < (maxErrors[jntNr-1]-0.0017) ){
				tempHomJntAngles[jntNr-1]-=stepSize;
			}

			// If the mechanical endstop is reached (error to large)
			else if( fabs(jointErrors[jntNr-1]) >= (maxErrors[jntNr-1]-0.0017) ){
				
				// From the mechanical endstop move back to homing position
				tempHomJntAngles[jntNr-1]=measRelJntAngles[jntNr-1]+homedPos[jntNr-1];
				
				// Reset the interpolator for jnt jntNr to the position it is at
				doubles resetdata(32,0.0);
				resetdata[(jntNr-1)*4]=1.0;
				resetdata[(jntNr-1)*4+1]=measRelJntAngles[jntNr-1];
				resetdata[(jntNr-1)*4+1]=measRelJntAngles[jntNr-1];
				resetIntPort.write(resetdata);
				
				// Proceed to next joint
				goodToGo = false;
				
			}	
				
		}
		
	}
	
	else if(goodToGo == false){
		
		// If joint is homed using abs sens no waiting time is required
		if(absOrRel[jntNr-1]==0 && jntNr!=1){
			jntNr--;
			log(Warning)<<"WATCHDOG: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo = true;
		}
		if(absOrRel[jntNr-1]==1 && jntNr==4){
			jntNr--;
			log(Warning)<<"WATCHDOG: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo = true;
		}
		// If joint is moved to endstop using rel enc waiting time is required to move to homing position
		else if(absOrRel[jntNr-1]==1 && cntr2<1500 && jntNr!=1 && jntNr!=4){
			
			cntr2++;
			
		}
		// If waiting is complete move on to next joint
		else if(absOrRel[jntNr-1]==1 && cntr2==1500 && jntNr!=1 && jntNr!=4){
			
			cntr2=0;
			jntNr--;
			log(Warning)<<"WATCHDOG: Proceeded to joint "<<jntNr<<"\n"<<endlog();
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
				log(Warning)<<"WATCHDOG: Renulled PERA_IO \n"<<endlog();
				
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

				log(Warning)<<"WATCHDOG: Finished homing \n"<<endlog();
				
			}			
			
		}
				
	}
	
	return tempHomJntAngles;
	
}

ORO_CREATE_COMPONENT(PERA::WATCHDOG)
