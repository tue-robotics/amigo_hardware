/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0 
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
//#include <amigo_msgs/arm_joints.h>
#include <amigo_msgs/pera_status.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include "PERA_Supervisor.hpp"

#define PI 3.1415926535897932384626433

using namespace std;
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
	addPort("reNullPort",reNullPort).doc("Sends signal to ReadEncoders to renull at actual position");
	addPort("enableReadRefPort",enableReadRefPort).doc("Sends enable signal to ReadReference allow reading of joint angles from ROS topic");
	addPort("gripper_command", gripperCommandPort);
	addPort("gripper_measurement",gripperMeasurementPort);
	addPort("gripperResetPort",gripperResetPort).doc("Requests for GripperController reset");
	addPort("controllerOutputPort",controllerOutputPort).doc("Receives motorspace output of the controller");
	addPort("peraStatusPort",peraStatusPort).doc("For publishing the PERA status to the AMIGO dashboard");
	addPort("jointVelocity",measVelPort).doc("Receives the reference interpolator joint velocities");

	addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
	addProperty( "enableOutput", ENABLE_PROPERTY ).doc("Specifies if PERA_IO should be enabled");
	addProperty( "jointUpperBounds", UPPERBOUNDS ).doc("Joint upper mechanical bound wrt zero-pose");
	addProperty( "jointLowerBounds", LOWERBOUNDS ).doc("Joint lower mechanical bound wrt zero-pose");
	addProperty( "motorSaturations", MOTORSAT ).doc("Motor saturation values");
	addProperty( "maxConSatTime", MAXCONSATTIME ).doc("Maximum time the controller is allowed to be saturated");
	addProperty( "homedPos", HOMEDPOS ).doc("Homing positions for the joints");
	addProperty( "absOrRel", ABS_OR_REL ).doc("Defines whether joint is homed using absolute sensors or using mechanical endstop");
	addProperty( "absSenDir", ABS_SEN_DIR ).doc("Defines if absolute sensor has its positive direction the same or opposite to relative sensors");
	addProperty( "stepSize", STEPSIZE ).doc("Speed the joint moves to its mechanical endstop during homing");
	addProperty( "requireHoming", REQUIRE_HOMING ).doc("Specifies if the arm will home yes or no");
	addProperty( "startJoint", STRT_JNT ).doc("Joint number to start homing with");
	addProperty( "requireGripperHoming", REQUIRE_GRIPPER_HOMING ).doc("Defines whether the gripper is homed upon startup");
    addProperty( "jointNames", out_msg.name ).doc("Joint state names");
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
    out_msg.position.resize(7);

	// Set initial counters
	cntr=0;   // used to check if it is the first time the loop is running 
	cntr2=0;  // used for timing in the homing procedure (to wait after a joint reached its endstop to let it go back to homedPOS)
	cntr3=0;  // used for warning in homing procedure to make sure it is printed only once per X seconds
	cntr4=0;  // used for sleep of 1s to make sure soem is awake TODO find out why this is needed
	soemAwake=false;
	nulling = false;
	
	firstSatInstance[0] = 0;
	firstSatInstance[1] = 0;
	firstSatInstance[2] = 0;
	firstSatInstance[3] = 0;
	firstSatInstance[4] = 0;
	firstSatInstance[5] = 0;
	firstSatInstance[6] = 0;
	firstSatInstance[7] = 0;

	// Errors is false by default
	errors=false;

	// Gripper initially not homed by default
	gripperHomed = !REQUIRE_GRIPPER_HOMING;

	// Assign ENABLE_PROPERTY to value enable
	enable=ENABLE_PROPERTY;

	// Write enable value to PERA_IO
	enablePort.write(enable);

	// goodToGo true means homing can proceed to next joint
	goodToGo = true;

	// Set homed to false by default
	if(REQUIRE_HOMING){
		homed = false;
	}
	else if(!REQUIRE_HOMING){
		homed = true;
	}

	// Set variables for homingprocedure
	jntNr=STRT_JNT;	
	FastStep = 0.175;
	SlowStep = 0.0425;
	Ts = 1000; 
	
	// Pressed is true. Assume emergency button pressed untill informed otherwise.
	pressed = true;
	
	return true;
}

bool Supervisor::startHook()
{
	if(!REQUIRE_HOMING){
		bool enableReadRef = true;
		enableReadRefPort.write(enableReadRef);
	}

	log(Info)<<"SUPERVISOR: started."<<endlog();

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
	doubles jointarray1(9,0.0); //mAbs has size 9 since 8 sensors are read plus one obsolete sensorport
	doubles jointarray2(8,0.0); //mRel has size 8 since only 8 encoders are recieved from the MotorToJointAngles component
	if ( !mAbsJntAngPort.read(jointarray1) == NewData || !mRelJntAngPort.read(jointarray2) == NewData )
	{
		log(Warning) << "RPERA_Supervisor: Excecuting updatehook not usefull if no new data is received." << endlog();
		return;
	}
		
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
			controllerOutputs.resize(9);
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

			reqJntAngPort.read(jointAngles);
			jointErrorsPort.read(jointErrors);

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
						//log(Warning)<<"SUPERVISOR: Joint reference "<<i+1<<" of "<<jointAngles[i]<<" exceeds maximum of "<<UPPERBOUNDS[i]<<endlog();
					}
					else if(jointAngles[i]<LOWERBOUNDS[i] && errors == false){
						bool enableReadRef = false;
						enableReadRefPort.write(enableReadRef);
						homJntAngPort.write(previousAngles);
						//log(Warning)<<"SUPERVISOR: Joint reference "<<i+1<<" of "<<jointAngles[i]<<" exceeds minimum of "<<LOWERBOUNDS[i]<<endlog();
					}

				}
				// If inside the bounds enable the reading of the reference and update the previous position
				else if(jointAngles[i]>=LOWERBOUNDS[i] && jointAngles[i]<=UPPERBOUNDS[i]){

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
				if( (fabs(jointErrors[i])>MAX_ERRORS[i]) && (jntNr!=i+1) && (nulling == false)  ){

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
			 
			 if (cntr4 < 1000) {
				cntr4++;				
				}
			 else if (cntr4 == 1000) {
				soemAwake = true;
				cntr4++;
				}

			if(enable && !homed && !errors && soemAwake){

				doubles measRelJntAngles(8,0.0);
                doubles measAbsJntAngles(9,0.0);
				doubles homJntAngTemp(7,0.0);
						
				// Measure the abs en rel angles.
				mRelJntAngPort.read(measRelJntAngles);
				mAbsJntAngPort.read(measAbsJntAngles);

				if(cntr==0){

					for(unsigned int i = 0;i<7;i++){
						homJntAngles[i]=measRelJntAngles[i];
						log(Info)<<"measRelJntAngles :"<<measRelJntAngles[jntNr-1]<<endlog(); 
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
			//log(Warning)<<"SUPERVISOR ( UH !Pressed): enable is send to driver. Enable = [" << enable << "]" <<endlog();	
			enablePort.write(enable);

		}
		else if(pressed){
			
			doubles resetdata(32,0.0);
			doubles measRelJntAngles(8,0.0);
//			amigo_msgs::arm_joints jointResetData;

			// Set enable to false and write it to the PERA_IO component.
			enable = false;
			//log(Warning)<<"SUPERVISOR ( UH Pressed): enable is send to driver. Enable = [" << enable << "]" <<endlog();
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

			// Change sign and add offset (wrt inverse kinematics)
			for ( uint i = 0; i < 7; i++ )
			{
                out_msg.position[i] = measRelJntAngles[i];
//				jointResetData.pos[i].data = measRelJntAngles[i];
			}

			//log(Error)<<"SUPERVISOR: i wrote q2 = "<<jointResetData.pos[1].data<<"."<<endlog();
            resetRefPort.write(out_msg);

			// Write the new angles to the interpolator for reset such that interpolator will follow the arm position causing no jump in the error
			resetIntPort.write(resetdata);

		}	

	}
	else if(!ENABLE_PROPERTY){

		enable = false;
		log(Warning)<<"SUPERVISOR (ENABLE_PROPERTY): enable is send to driver. Enable = [" << enable << "]" <<endlog();
		enablePort.write(enable);
	}
	
	std_msgs::UInt8 statusToDashboard;
	
	if(homed==true && errors==false){
		statusToDashboard.data = 0;
		peraStatusPort.write(statusToDashboard);
	}
	else if(homed==false && errors==false){
		statusToDashboard.data = 1;
		peraStatusPort.write(statusToDashboard);
	}
	else if(errors==true){
		statusToDashboard.data = 2;
		peraStatusPort.write(statusToDashboard);
	}

}

/* Using the absolute sensors and the mechanical endstops the PERA is
 * homed. Given the current angles this function returns the next set
 * of reference joint angles for the homing procedure.
 */
doubles Supervisor::homing(doubles jointErrors, doubles absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles){
	

	if(!gripperHomed) {

		amigo_msgs::AmigoGripperCommand gripperCommand;
		gripperCommand.direction = amigo_msgs::AmigoGripperCommand::CLOSE;
		gripperCommand.max_torque = 1000;

		gripperCommandPort.write(gripperCommand);

		amigo_msgs::AmigoGripperMeasurement gripperMeasurement;
		gripperMeasurementPort.read(gripperMeasurement);
		
		if (gripperMeasurement.max_torque_reached)
		log(Warning) << "Gripper Max torque reached" <<endlog();
		
		if (gripperMeasurement.end_position_reached)
		log(Warning) << "Gripper end postion reached" <<endlog();

		if(gripperMeasurement.end_position_reached || gripperMeasurement.max_torque_reached){
			log(Warning)<<"SUPERVISOR: gripper homed"<<endlog();
			gripperHomed = true;
			jntNr = STRT_JNT;
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
						tempHomJntAngles[jntNr-1]+=(FastStep/Ts); //go forward fast
						if (cntr3 > (Ts/10)) {
						log(Warning)<<"SUPERVISOR: 1.0 for joint q"<<jntNr<<" increasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]-=(FastStep/Ts); //go back fast
						if (cntr3 > (Ts/10)) {
						log(Warning)<<"SUPERVISOR: 1.0 for joint q"<<jntNr<<" decreasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]+=(SlowStep/Ts); //go forward slowly
						if (cntr3 > (Ts/10)) {
						log(Warning)<<"SUPERVISOR: 2.0 for joint q"<<jntNr<<" increasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]-=(SlowStep/Ts); //go back slowly
						if (cntr3 > (Ts/10)) {						
						log(Warning)<<"SUPERVISOR: 2.0 for joint q"<<jntNr<<" decreasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
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
						log(Warning)<<"SUPERVISOR: -1.0 for joint q"<<jntNr<<" decreasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
						cntr3 = 1;
						}
						cntr3++;					
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<-15.0){ //if desired point is far behind
						tempHomJntAngles[jntNr-1]+=(FastStep/Ts); //go back fast
						if (cntr3 > (Ts/10)) {						
						log(Warning)<<"SUPERVISOR: -1.0 for joint q"<<jntNr<<" increasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
						cntr3 = 1;
						}
						cntr3++;				
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<=15.0){ //if desired point is close ahead
						tempHomJntAngles[jntNr-1]-=(SlowStep/Ts); //go forward slowly
						if (cntr3 > (Ts/10)) {						
						log(Warning)<<"SUPERVISOR: -2.0 for joint q"<<jntNr<<" decreasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]"  <<endlog();
						cntr3 = 1;
						}
						cntr3++;
					}
					else if((HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])<0.0 && (HOMEDPOS[jntNr-1]-absJntAngles[jntNr-1])>=-15.0){ //if desired point is close behind
						tempHomJntAngles[jntNr-1]+=(SlowStep/Ts); //go back slowly
						if (cntr3 > (Ts/10)) {						
						log(Warning)<<"SUPERVISOR: -2.0 for joint q"<<jntNr<<" increasing despos towards:"<<homJntAngles[jntNr-1] << ", [" << absJntAngles[1] << "," << absJntAngles[0] << "]" <<endlog();
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
			//if (jntNr == 6) log(Warning) << "Joint error: " << fabs(jointErrors[jntNr-1]) << "  Threshold: " << (MAX_ERRORS[jntNr-1]*0.5) << " Joint angle: " << measRelJntAngles[jntNr-1] << endlog();
			
			
			if( fabs(jointErrors[jntNr-1]) < (MAX_ERRORS[jntNr-1]*0.5) ){
				tempHomJntAngles[jntNr-1]-=(STEPSIZE/Ts);
				//log(Warning) << "Stepsize is done: [ " << fabs(jointErrors[jntNr-1]) << " >= " << (MAX_ERRORS[jntNr-1]-0.0017) << "," << jntNr << "]" <<endlog();
			}

			// If the mechanical endstop is reached (error to large)
			else if( fabs(jointErrors[jntNr-1]) >= (MAX_ERRORS[jntNr-1]*0.5) ){
				log(Warning) << "The mechanical endstop is reached for joint : [ " << jntNr << "]" <<endlog();

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
		// If joint is homed using abs sens short waiting time is required
		if( ((ABS_OR_REL[jntNr-1]==0 && jntNr!=1) || (ABS_OR_REL[jntNr-1]==1 && jntNr==4) ) && (cntr2<(1*Ts))){
			cntr2++;
		}

		// If waiting is complete move on to next joint
		if( ((ABS_OR_REL[jntNr-1]==0 && jntNr!=1) || (ABS_OR_REL[jntNr-1]==1 && jntNr==4) ) && (cntr2==(1*Ts))){
			cntr2=0;
			jntNr--;
			log(Warning)<<"SUPERVISOR: Proceeded to joint "<<jntNr<<"\n"<<endlog();
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
			log(Warning)<<"SUPERVISOR: Proceeded to joint "<<jntNr<<"\n"<<endlog();
			goodToGo=true;

		}
		// If homed joint is last one, reset interpolator and PERA_USB_IO
		else if(jntNr==1){
		
			if(cntr2>=0 && cntr2<Ts){
				cntr2++;
			}

			if(cntr2>=Ts && cntr2<(Ts+10)){
												
				enable = false;
				enablePort.write(enable);
				nulling = true;
				
				cntr2++;
			}

			else if(cntr2==(Ts+10)){			
						
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

				enable = false;
				enablePort.write(enable);


				// Enable the reading of the reference joint angles.
				bool enableReadRef = false;
				enableReadRefPort.write(enableReadRef);

				// Reset the reference interpolator to zero
				resetIntPort.write(resetdata);
				
				// Null the PERA_IO.
				bool reNull = true;
				reNullPort.write(reNull);
				log(Info)<<"SUPERVISOR: Renulled PERA_IO \n"<<endlog();
				
				enable = false;
				enablePort.write(enable);

				cntr2++;

			}
			else if(cntr2>=(Ts+11) && cntr2<(Ts+Ts)){
				if (cntr2 == (Ts+11))
					enable = false;
					enablePort.write(enable);
				
				cntr2++;
			}
			
			else if(cntr2>=(Ts+Ts) && cntr2<(Ts+Ts+10)){
				
				// Enable PERA IO
				enable = true;
				enablePort.write(enable);
				nulling = false;
				
				cntr2++;
			}
			
			else if(cntr2 >= (Ts+Ts+10)){
				
				// Enable homing for next joint
				goodToGo = true;

				// Reset counter for next round
				cntr2 = 0;

				// set jointNr to zero, otherwise q1 will not be checked for exceeding errors
				jntNr--;

				// Set homing to true
				homed = true;
				
				log(Warning)<<"SUPERVISOR: Finished homing \n"<<endlog();
			}			

		}

	}

	return tempHomJntAngles;

}


ORO_CREATE_COMPONENT(PERA::Supervisor)
