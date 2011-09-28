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
#include <diagnostic_msgs/DiagnosticArray.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include "PERA_Supervisor.hpp"

#define PI 3.1415926535897932384626433

using namespace RTT;
using namespace PERA;
using namespace std;

Supervisor::Supervisor(const string& name) : 
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
	addPort("controllerOutputPort",controllerOutputPort).doc("Receives motorspace output of the controller");
	addPort("peraStatusPort",peraStatusPort).doc("For publishing the PERA status to the AMIGO dashboard");

	addProperty( "hardware_id", HARDWARE_ID).doc("Hardware ID");
	addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
	addProperty( "enableOutput", ENABLE_PROPERTY ).doc("Specifies if PERA_IO should be enabled");
	addProperty( "jointUpperBounds", UPPERBOUNDS ).doc("Joint upper mechanical bound wrt zero-pose");
	addProperty( "jointLowerBounds", LOWERBOUNDS ).doc("Joint lower mechanical bound wrt zero-pose");
	addProperty( "motorSaturations", MOTORSAT ).doc("Motor saturation values");
	addProperty( "maxConSatTime", MAXCONSATTIME ).doc("Maximum time the controller is allowed to be saturated");
	addProperty( "resetAngles", RESETANGLES ).doc("Joint angles to be published when emergency button is released");
	addProperty( "homedPos", HOMEDPOS ).doc("Homing positions for the joints");
	addProperty( "absOrRel", ABS_OR_REL ).doc("Defines whether joint is homed using absolute sensors or using mechanical endstop");
	addProperty( "absSenDir", ABS_SEN_DIR ).doc("Defines if absolute sensor has its positive direction the same or opposite to relative sensors");
	addProperty( "stepSize", STEPSIZE ).doc("Speed the joint moves to its mechanical endstop during homing");
	addProperty( "requireHoming", REQUIRE_HOMING ).doc("Specifies if the arm will home yes or no");
	addProperty( "startJoint", STRT_JNT ).doc("Joint number to start homing with");

}

Supervisor::~Supervisor(){}

bool Supervisor::configureHook()
{

	Logger::In in("SUPERVISOR"); 

	jointAngles.resize(7);
	jointErrors.resize(8);
	homJntAngles.resize(8);
	previousAngles.resize(8);
	timeReachedSaturation.resize(8);
	
	firstSatInstance = {0, 0, 0, 0, 0, 0, 0, 0};

	// Set loopcounters to zero initially
	cntr=0;
	cntr2=0;

	// Errors is false by default
	errors=false;

	// Gripper initially not homed by default
	gripperHomed = false;

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
	jntNr=5;

	// Pressed is true. No SOEM heartbeat means no amplifier enabling.
	pressed = true;

	return true;
}

bool Supervisor::startHook()
{

	// Wait for SOEM heartbeat.
	while(!(eButtonPort.read(eButtonPressed) == NewData)){
		sleep(1);
		log(Info)<<"SUPERVISOR: Waiting for enable-signal from the emergency button"<<endlog();
		cntr++;
		if( cntr == 10 && cntr<3 ){
			log(Warning)<<"SUPERVISOR: no signal from emergency button (SOEM). Is SOEM running?"<<endlog();
			cntr = 0;
			cntr2++;
		}
		else if( cntr==3 ){
			log(Error)<<"SUPERVISOR: no SOEM heartbeat. Shutting down LPERA."<<endlog();
			cntr=0;
			cntr2=0;
			return false;
		}
	}

	if(!REQUIRE_HOMING){
		bool enableReadRef = true;
		enableReadRefPort.write(enableReadRef);
	}

	log(Info)<<"SUPERVISOR: configured and running."<<endlog();

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
void Supervisor::updateHook()
{
	if(ENABLE_PROPERTY){
		// Read emergency-button state
		eButtonPort.read(eButtonPressed);

		if(!eButtonPressed.data){
			if(pressed!=false){
				log(Info)<<"SUPERVISOR: Emergency Button Released"<<endlog();
			}
			pressed = false;
		}
		else if(eButtonPressed.data){
			if(pressed!=true){
				log(Info)<<"SUPERVISOR: Emergency Button Pressed"<<endlog();
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
			
			
			/* When a controller reaches its saturation value it is
			 * monitored how long this continues. After MAXCONSATTIME
			 * the PERA_IO is disabled.
			 */
			doubles controllerOutputs;
			controllerOutputs.resize(8);
			controllerOutputPort.read(controllerOutputs);
			
			long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9; 
			
			for(unsigned int i = 0;i<7;i++){
				if(firstSatInstance[i]==0 && fabs(controllerOutputs[i])>=MOTORSAT[i]){
					timeReachedSaturation[i]=timeNow;
					firstSatInstance[i]=1;
				}
				else if(fabs(controllerOutputs[i])<MOTORSAT[i]){
					timeReachedSaturation[i]=timeNow;
					firstSatInstance[i]=0;
				}
				if(fabs(timeNow-timeReachedSaturation[i])>=MAXCONSATTIME){
					if(errors==false){ // This check makes sure it is printed only once.
						log(Error)<<"SUPERVISOR: Motor output "<<i+1<<" satured too long (absolute "<<MAXCONSATTIME<<" sec above "<<fabs(MOTORSAT[i])<<"). PERA output disabled."<<endlog();
						enable = false;
						errors = true;
					}
				}
			}

			/* Set the value to false, otherwise if eButtonPressed.data
			 * becomes false it will still keep resetting the reference
			 * interpolator
			 */
			resetReference = false;

			reqJntAngPort.read(jointAngles);
			jointErrorsPort.read(jointErrors);
			
			int ii=0;
			
			/* Check if joint angles requested by the path planning node
			 * in ROS are within the feasible joint limits.
			 */
			for(unsigned int i = 0;i<7;i++){
				
				// If outside bounds, disable usage and write previous angles to the reference interpolator
				if(!(jointAngles[i]>=LOWERBOUNDS[i] && jointAngles[i]<=UPPERBOUNDS[i])){ 

					if(jointAngles[i]>UPPERBOUNDS[i] && errors == false){
						
						bool enableReadRef = false;
						enableReadRefPort.write(enableReadRef);
						homJntAngPort.write(previousAngles);
					}
					else if(jointAngles[i]<LOWERBOUNDS[i] && errors == false){
						bool enableReadRef = false;
						enableReadRefPort.write(enableReadRef);
						homJntAngPort.write(previousAngles);
					}

				}
				else{
					ii++;
				}
				if(ii==7 && errors == false){
				
					if(homed){
						bool enableReadRef = true;
						enableReadRefPort.write(enableReadRef);
					}
					previousAngles=jointAngles;
				
				}

			}

			// Check if joint errors are within specified limits
			for(unsigned int i = 0;i<8;i++){

				// If the error is too large and corresponding joint is NOT being homed -> stop PERA_IO
				if( (fabs(jointErrors[i])>MAX_ERRORS[i]) && (jntNr!=i+1) ){

					enable = false;

					if( errors == false ){ // This check makes sure it is printed only once.
						log(Error)<<"SUPERVISOR: Error of joint q"<<i+1<<" exceeded limit ("<<MAX_ERRORS[i]<<"). PERA output disabled."<<endlog();
						errors = true;
					}		

				}

			}

			/* Check if homing is completed. Else, request for homing angles and
			 * disable the reading of the reference joint angles from the
			 * ROS inverse kinematics.
			 */
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

			// Reset the ROS path planning incoming reference topic
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
	
	std_msgs::Bool statusToDashboard;
	
	if(enable==true && homed==true && errors==false){
		statusToDashboard.data = true;
		peraStatusPort.write(statusToDashboard);
	}
	else{
		statusToDashboard.data = false;
		peraStatusPort.write(statusToDashboard);
	}

}

/* Using the absolute sensors and the mechanical endstops the PERA is
 * homed. Given the current angles this function returns the next set
 * of reference joint angles for the homing procedure.
 */
doubles Supervisor::homing(doubles jointErrors, ints absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles){

	if(!gripperHomed){

		std_msgs::Bool gripperClose;
		gripperClose.data = true;
		gripperClosePort.write(gripperClose);

		std_msgs::Bool gripperStatus;
		gripperStatusPort.read(gripperStatus);

		if(gripperStatus.data){
			log(Warning)<<"SUPERVISOR: gripper homed"<<endlog();
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
						log(Info)<<"SUPERVISOR: 1.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]-=0.0007; //go back fast
						log(Info)<<"SUPERVISOR: 1.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]+=0.00017; //go forward slowly
						log(Info)<<"SUPERVISOR: 2.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]-=0.00017; //go back slowly
						log(Info)<<"SUPERVISOR: 2.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}


				}
				// Are positive directions absolute sensors and joints opposite?
				else if(ABS_SEN_DIR[jntNr-1]==-1.0){

					if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]-=0.0007; //go forward fast 
						log(Info)<<"SUPERVISOR: -1.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]+=0.0007; //go back fast
						log(Info)<<"SUPERVISOR: -1.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]-=0.00017; //go forward slowly
						log(Info)<<"SUPERVISOR: -2.0 for joint q"<<jntNr<<" decreasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]+=0.00017; //go back slowly
						log(Info)<<"SUPERVISOR: -2.0 for joint q"<<jntNr<<" increasing despos towards "<<homJntAngles[jntNr-1]<<endlog();
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
			log(Info)<<"SUPERVISOR: Proceeded to joint "<<jntNr<<"\n"<<endlog();
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
			log(Info)<<"SUPERVISOR: Proceeded to joint "<<jntNr<<"\n"<<endlog();
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
				log(Info)<<"SUPERVISOR: Renulled PERA_IO \n"<<endlog();

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

				log(Info)<<"SUPERVISOR: Finished homing \n"<<endlog();

			}			

		}

	}

	return tempHomJntAngles;

}

ORO_CREATE_COMPONENT(PERA::Supervisor)
