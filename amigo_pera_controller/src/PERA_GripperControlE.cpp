/**************************************************************************
 *                                                                        *
 *   S. Marinkov                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PERA_GripperControlE.hpp"

//#define gripperGain 0.040 // Original
//#define gripperGain 0.40
#define MAX_TORQUE 100.0

using namespace std;
using namespace RTT;
using namespace PERA;

	GripperControlE::GripperControlE(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addPort("gripper_command", gripperCommandPort);
			addPort("torque_in", torqueInPort);
			addPort("position_in", positionInPort);
			addPort("reNullPort",reNullPort);
			addEventPort("resetGripperPort",resetGripperPort);
			
			/// Outports
			addPort("gripper_ref",gripperRefPort);
			addPort("gripper_measurement",gripperMeasurementPort);
			
			/// Thresholds for the gripper force
			addProperty( "threshold_closed", threshold_closed);	
			addProperty( "gripper_gain", gripperGain);		
			addProperty( "max_pos", maxPos);
			
			gripperHomed = false;
					
	  }

	GripperControlE::~GripperControlE(){}

	bool GripperControlE::configureHook(){
		torques.resize(8);
		gripperPos.resize(1,0.0);
		completed = true;
		return true;
	}

	bool GripperControlE::startHook(){
		return true;
	}

	void GripperControlE::updateHook(){
		
		bool resetGripper;
		
		if (resetGripperPort.read(resetGripper) == NewData){
			if(resetGripper){
				gripperPos[0]=0.0;
			}
		}
		
		if (gripperCommandPort.read(gripperCommand) == NewData){
			completed = false;
		}
			
		// Check whether supervisor specifies nulling of the relative encoders
		bool reNull;
		if(NewData == reNullPort.read(reNull)){
			if(reNull == true){
				log(Warning)<<"Grippercontrol received reNull signal"<<endlog();
				// Increase threshold after the gripper has homed
				threshold_closed = threshold_closed*1.25;
				// Renull the gripperPos after homing
				gripperPos[0] = 0;
				gripperRefPort.write(gripperPos);
				// gripperHomed = true if all joints are homed
				gripperHomed = true;
			}
		}
				

		if (!completed){
			torqueInPort.read(torques);
			positionInPort.read(measPos);
			
			amigo_msgs::AmigoGripperMeasurement gripperMeasurement;
			gripperMeasurement.direction = gripperCommand.direction;
			gripperMeasurement.torque = torques[GRIPPER_JOINT_INDEX];
			gripperMeasurement.position = measPos[GRIPPER_JOINT_INDEX]/maxPos;
			gripperMeasurement.end_position_reached = false;
			gripperMeasurement.max_torque_reached = false;

			if(gripperCommand.direction == amigo_msgs::AmigoGripperCommand::OPEN){
				if (gripperPos[0] >= maxPos){
					//log(Info)<<"Gripper is OPEN"<<endlog();
					gripperMeasurement.end_position_reached = true;
					completed = true;
				} 
				else{
					gripperPos[0] += gripperGain*PI/180;
				}
			} 
			else{
				//log(Warning)<<"gripper torques = "<<torques[GRIPPER_JOINT_INDEX]<<endlog();
				if ( (torques[GRIPPER_JOINT_INDEX] >= threshold_closed && torques[GRIPPER_JOINT_INDEX] < MAX_TORQUE) || ( gripperHomed && (gripperPos[0] < 0.0) ) ){
					log(Warning)<<"Gripper is CLOSED"<<endlog();
					gripperMeasurement.end_position_reached = true;
					completed = true;
				} 
				else if(torques[GRIPPER_JOINT_INDEX] < threshold_closed && torques[GRIPPER_JOINT_INDEX] < MAX_TORQUE){
					//log(Warning)<<"GRIPPERCON: closing with torque = "<<torques[GRIPPER_JOINT_INDEX]<<endlog();
					gripperPos[0] -= gripperGain*PI/180;
				}
				else {
					log(Error)<<"Gripper torque "<<torques[GRIPPER_JOINT_INDEX]<<" exceeds maximum torque of "<<MAX_TORQUE<<" abort close_gripper"<<endlog();
					completed = true;
					gripperMeasurement.max_torque_reached = true;
				}
			}
			gripperRefPort.write(gripperPos);
			gripperMeasurementPort.write(gripperMeasurement);
		}
	}

void GripperControlE::stopHook(){}

ORO_CREATE_COMPONENT(PERA::GripperControlE)

