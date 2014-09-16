/*! 
 * \author Max Baeten
 * \date May, 2011
 * \version 1.0 
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <amigo_msgs/pera_status.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include "PeraHoming.hpp"

#define PI 3.1415926535897932384626433

using namespace std;
using namespace RTT;
using namespace AMIGOPERA;

PeraHoming::PeraHoming(const string& name) : TaskContext(name, PreOperational)
{
	// inports
	addPort("errorPort",jointErrorsPort).doc("Receives joint control errors");
	addPort("measRelJointAnglesPort",mRelJntAngPort).doc("Receives measured relative joint angles");
	addPort("measAbsJointAnglesPort",mAbsJntAngPort).doc("Receives measured absolute joint angles");
	addPort("gripper_measurement",gripperMeasurementPort);
	// outports
	addPort("reNullPort",reNullPort).doc("Sends signal to ReadEncoders to renull at actual position");
	addPort("gripper_command", gripperCommandPort);
	addPort("homJntAnglesPort",homJntAngPort).doc("Sends the requested angles for homing to the ReferenceInterpolator");
  	addPort("homing_finished", homingfinished_outPort ).doc("Sends homing finished bool to supervisor");
  	addPort("homing_joint", homingjoint_outPort).doc("Sends homing joint to safety such that safety component can temporarily switch off this check");
  	addPort("resetInterpolatorPort",resetIntPort).doc("Sends resetvalues to the ReferenceInterpolator");
  	addPort("resetInterpolatorPort2",resetIntPort2).doc("Sends resetvalues to the ReferenceInterpolator");
  	addPort("endpos_out",endpose_outPort).doc("To send a end pose when homing is finished");

	addProperty( "homedPos", HOMEDPOS ).doc("Homing positions for the joints"); 
	addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
	addProperty( "absOrRel", ABS_OR_REL ).doc("Defines whether joint is homed using absolute sensors or using mechanical endstop");
	addProperty( "absSenDir", ABS_SEN_DIR ).doc("Defines if absolute sensor has its positive direction the same or opposite to relative sensors");
	addProperty( "stepSize", STEPSIZE ).doc("Speed the joint moves to its mechanical endstop during homing");
	addProperty( "requireHoming", REQUIRE_HOMING ).doc("Specifies if the arm will home yes or no");
	addProperty( "startJoint", STRT_JNT ).doc("Joint number to start homing with");
	addProperty( "requireGripperHoming", REQUIRE_GRIPPER_HOMING ).doc("Defines whether the gripper is homed upon startup");
    addProperty( "endPose", END_POSE ).doc("end Pose the PERA should go after homing");
    addProperty( "jointNames", out_msg.name ).doc("Joint state names");
}

PeraHoming::~PeraHoming(){}

bool PeraHoming::configureHook()
{
	FastStep = 0.15;
	SlowStep = 0.01;
	Ts = 1000; 
	out_msg.position.resize(7);
	return true;
}

bool PeraHoming::startHook()
{	// checking inports
	if (!jointErrorsPort.connected()) {
		log(Error)<<"jointErrorsPort not connected!"<<endlog();
		return false;
	}
	if (!mRelJntAngPort.connected()) {
		log(Error)<<"mRelJntAngPort not connected!"<<endlog();
		return false;
	}
	if (!mAbsJntAngPort.connected()) {
		log(Error)<<"mAbsJntAngPort not connected!"<<endlog();
		return false;
	}
	if (!gripperMeasurementPort.connected()) {
		log(Error)<<"gripperMeasurementPort not connected!"<<endlog();
	}
	
	// checking outports
	if (!reNullPort.connected()) {
		log(Error)<<"reNullPort not connected!"<<endlog();
		return false;
	}
	if (!gripperCommandPort.connected()) {
		log(Error)<<"gripperCommandPort not connected!"<<endlog();
	}
	if (!homJntAngPort.connected()) {
		log(Error)<<"homJntAngPort not connected!"<<endlog();
		return false;
	}
	if (!homingfinished_outPort.connected()) {
		log(Error)<<"homingfinished_outPort not connected!"<<endlog();
		return false;
	}
	if (!homingjoint_outPort.connected()) {
		log(Error)<<"homingjoint_outPort not connected, safety does not know which joint is homing!"<<endlog();
	}
	
	if ((!endpose_outPort.connected()) || (END_POSE.size() != 7 )) {
		log(Error)<<"endpose_out not connected or END_POSE has a wrong size, size should be 7!"<<endlog();
	}
	
	jointAngles.assign(7,0.0);
	homJntAngles.assign(8,0.0);
	previousAngles.assign(8,0.0);
	jointErrors.assign(8,0.0);	
	
	cntr=0;   // used to check if it is the first time the loop is running 
	cntr2=0;  // used for timing in the homing procedure (to wait after a joint reached its endstop to let it go back to homedPOS)
	cntr3=0;  // used for warning in homing procedure to make sure it is printed only once per X seconds
	cntr4=0;  // used for sleep of 1s to make sure soem is awake TODO find out why this is needed

	goodToGo = true;
	homed_ = !REQUIRE_HOMING;
	gripperhomed_ = !REQUIRE_GRIPPER_HOMING;
	Q6homed = false;
	jntNr=STRT_JNT;
	
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
 
void PeraHoming::updateHook()
{
	doubles measRelJntAngles(8,0.0);
	doubles measAbsJntAngles(9,0.0);
	doubles homJntAngTemp(7,0.0);
			
	// Measure the abs en rel angles.
	mRelJntAngPort.read(measRelJntAngles);
	mAbsJntAngPort.read(measAbsJntAngles);			
	jointErrorsPort.read(jointErrors);

	if(cntr==0){
		for(unsigned int i = 0;i<7;i++){
			homJntAngles[i]=measRelJntAngles[i];
			log(Info)<<"measRelJntAngles :"<<measRelJntAngles[jntNr-1]<<endlog(); 
		}
		cntr++;
	}

	// Compute the joint angles for the homing procedure.
	homJntAngTemp = homing(jointErrors,measAbsJntAngles,homJntAngles,measRelJntAngles);

	for(unsigned int i = 0;i<8;i++){
		homJntAngles[i]=homJntAngTemp[i];
	}
	
	if ( homed_ != true ) {
		// Forward computed homing angles to the ReferenceInterpolator
		homJntAngPort.write(homJntAngles);
	}
	
	if ( homed_ == true ) {
		endpose_outPort.write(END_POSE);
		homingfinished_outPort.write(true);
	}
	
	// Send jntNr to safety component such that safety component can be disabled for these joints
	double homingjoint = (double) jntNr; 
	homingjoint_outPort.write(homingjoint);
}

void PeraHoming::stopHook()
{
	endpose_outPort.write(END_POSE);
	log(Warning)<<"PeraHoming: Sent to reset pos \n"<<endlog();
}

doubles PeraHoming::homing(doubles jointErrors, doubles absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles){
	
	if(!gripperhomed_) {
		amigo_msgs::AmigoGripperCommand gripperCommand;
		gripperCommand.direction = amigo_msgs::AmigoGripperCommand::CLOSE;
		gripperCommand.max_torque = 1000;

		gripperCommandPort.write(gripperCommand);

		amigo_msgs::AmigoGripperMeasurement gripperMeasurement;
		gripperMeasurementPort.read(gripperMeasurement);
		
		if (gripperMeasurement.max_torque_reached)
		log(Warning) << "PeraHoming: Gripper Max torque reached" <<endlog();
		
		if (gripperMeasurement.end_position_reached)
		log(Warning) << "PeraHoming: Gripper end postion reached" <<endlog();

		if(gripperMeasurement.end_position_reached || gripperMeasurement.max_torque_reached){
			log(Info)<<"PeraHoming: gripper homed"<<endlog();
			gripperhomed_ = true;
		}
	}
	
	// This code makes sure q6 is homed first
	if (jntNr == 7 && Q6homed == false ) {
		jntNr = 6;
	}
	
	if (jntNr == 5 && Q6homed == false) {
		jntNr = 7;
		Q6homed = true;		
	}
	if (jntNr == 6 && Q6homed == true) {
		jntNr = 5;		
	}

	if(jntNr!=0 && goodToGo && gripperhomed_) {
		// If true the homing will be done using abs sensor
		if(ABS_OR_REL[jntNr-1]==0){
			// Homing position not reached yet
			if(fabs(HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=1.0){
				// Are positive directions absolute sensors and joints the same?
				if(ABS_SEN_DIR[jntNr-1]==1.0){
					if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]+=(FastStep/Ts); //go forward fast
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving forward fast to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]-=(FastStep/Ts); //go back fast
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving backward fast to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]+=(SlowStep/Ts); //go forward slowly
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving forward slowly to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]-=(SlowStep/Ts); //go back slowly
						if (cntr3 > (Ts/10)) {						
						log(Info)<<"Homing q"<<jntNr<<". Moving backward slowly to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}


				}
				// Are positive directions absolute sensors and joints opposite?
				else if(ABS_SEN_DIR[jntNr-1]==-1.0){

					if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>15.0){ //if desired point is far ahead
						tempHomJntAngles[jntNr-1]-=(FastStep/Ts); //go forward fast 
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving forward fast to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]+=(FastStep/Ts); //go back fast
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving backward fast to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]-=(SlowStep/Ts); //go forward slowly
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving forward slowly to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]+=(SlowStep/Ts); //go back slowly
						if (cntr3 > (Ts/10)) {
						log(Info)<<"Homing q"<<jntNr<<". Moving backward slowly to:"<< HOMEDPOS[jntNr-1] << ". Sensor outputs:" << absJntAngles[jntNr-1] << "."<<endlog();
						cntr3 = 1;
						}
						cntr3++;
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
			// If the mechanical endstop is not reached yet
			
			if( fabs(jointErrors[jntNr-1]) < (MAX_ERRORS[jntNr-1]*0.5) ){
				tempHomJntAngles[jntNr-1]-=(STEPSIZE/Ts);
			}

			// If the mechanical endstop is reached (error to large)
			else if( fabs(jointErrors[jntNr-1]) >= (MAX_ERRORS[jntNr-1]*0.3) ){
				// From the mechanical endstop move back to homing position
				tempHomJntAngles[jntNr-1]=measRelJntAngles[jntNr-1]+HOMEDPOS[jntNr-1];
				homJntAngPort.write(tempHomJntAngles);
				
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

	else if(!goodToGo && gripperhomed_){
		// If joint is homed using abs sens short waiting time is required
		if( ((ABS_OR_REL[jntNr-1]==0 && jntNr!=1) || (ABS_OR_REL[jntNr-1]==1 && jntNr==4) ) && (cntr2<(1*Ts))){
			cntr2++;
		}

		// If waiting is complete move on to next joint
		if( ((ABS_OR_REL[jntNr-1]==0 && jntNr!=1) || (ABS_OR_REL[jntNr-1]==1 && jntNr==4) ) && (cntr2==(1*Ts))){
			cntr2=0;
			jntNr--;
			log(Info)<<"PeraHoming: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo = true;
		}
		// If joint is moved to endstop using rel enc waiting time is required to move to homing position
		else if(ABS_OR_REL[jntNr-1]==1 && cntr2<(5*Ts) && jntNr!=1 && jntNr!=4){
			cntr2++;
		}
		// If waiting is complete move on to next joint
		else if(ABS_OR_REL[jntNr-1]==1 && cntr2==(5*Ts) && jntNr!=1 && jntNr!=4){
			cntr2=0;
			jntNr--;
			log(Info)<<"PeraHoming: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo=true;

		}
		// If homed joint is last one, reset interpolator and PERA_USB_IO
		else if(jntNr==1){
		
			if(cntr2>=0 && cntr2<Ts){
				// wait a second
				cntr2++;
			}
			else if ( cntr2==Ts ) {
				
				// Reset the referenceInterpolator and PERA_IO.
				doubles resetdata(32,0.0);

				for(unsigned int i = 0;i<8;i++){
					tempHomJntAngles[i]=0.0;
				}
				
				// Fill up resetdata
				for(unsigned int i = 0;i<8;i++){
					resetdata[i*4]=1.0;
					resetdata[i*4+1]=0.0;
					resetdata[i*4+2]=0.0;
					resetdata[i*4+3]=0.0;
				}
				
				// Reset the reference interpolator to zero
				resetIntPort.write(resetdata);
				resetIntPort2.write(tempHomJntAngles);

				// Null the PERA_IO.
				bool reNull = true;
				reNullPort.write(reNull);
				log(Info)<<"PeraHoming: Renulled PERA_IO \n"<<endlog();
				
				cntr2++;

			}
			else if(cntr2 >= (Ts+1)){
				
				// Enable homing for next joint
				goodToGo = true;

				// Reset counter for next round
				cntr2 = 0;

				// set jointNr to zero, otherwise q1 will not be checked for exceeding errors
				jntNr--;

				// Set homing to true
				homed_ = true;
				
				log(Warning)<<"PeraHoming: Finished homing \n"<<endlog();
				
			}
		}
	}

	return tempHomJntAngles;

}


ORO_CREATE_COMPONENT(AMIGOPERA::PeraHoming)
