/**************************************************************************
 *                                                                        *
 *   S. Marinkov                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "PERA_GripperControl.hpp"

#define gripperGain 0.040
#define MAX_TORQUE 100.0

using namespace RTT;
using namespace PERA;

	GripperControl::GripperControl(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addPort("gripper_close", gripperClosePort);
			addPort("torque_in", torqueInPort);
			addEventPort("resetGripperPort",resetGripperPort);
			
			/// Outports
			addPort("gripper_ref",gripperRefPort);
			addPort("gripper_status",gripperStatusPort);
			
			/// Thresholds for the gripper force
			addProperty( "threshold_open", threshold_open);
			addProperty( "threshold_closed", threshold_closed);			
					
	  }

	GripperControl::~GripperControl(){}

	bool GripperControl::configureHook(){
		torques.resize(8);
		gripperPos.resize(1.0,0.0);
		completed = true;
		return true;
	}

	bool GripperControl::startHook(){
		return true;
	}

	void GripperControl::updateHook(){
		
		bool resetGripper;
		
		if (resetGripperPort.read(resetGripper) == NewData){
			if(resetGripper){
				gripperPos[0]=0.0;
			}
		}
		
		if (gripperClosePort.read(gripperClose) == NewData){
			completed = false;
		}
		
		if (!completed){
			torqueInPort.read(torques);
			if(!gripperClose.data){
				if (gripperPos[0] >= 3.5){
					log(Info)<<"Gripper is OPEN"<<endlog();
					std_msgs::Bool gripperStatus;
					gripperStatus.data = false;
					gripperStatusPort.write(gripperStatus);
					completed = true;
				} 
				else{
					gripperPos[0] += gripperGain*PI/180;
				}
			} 
			else{
				//log(Warning)<<"gripper torques = "<<torques[GRIPPER_JOINT_TORQUE_INDEX]<<endlog();
				if (torques[GRIPPER_JOINT_TORQUE_INDEX] >= threshold_closed && torques[GRIPPER_JOINT_TORQUE_INDEX] < MAX_TORQUE){
					log(Warning)<<"Gripper is CLOSED"<<endlog();
					std_msgs::Bool gripperStatus;
					gripperStatus.data = true;
					gripperStatusPort.write(gripperStatus);
					completed = true;
				} 
				else if(torques[GRIPPER_JOINT_TORQUE_INDEX] < threshold_closed && torques[GRIPPER_JOINT_TORQUE_INDEX] < MAX_TORQUE){
					//log(Warning)<<"GRIPPERCON: closing with torque = "<<torques[GRIPPER_JOINT_TORQUE_INDEX]<<endlog();
					gripperPos[0] -= gripperGain*PI/180;
				}
			}
			gripperRefPort.write(gripperPos);
		}
	}

void GripperControl::stopHook(){}

ORO_CREATE_COMPONENT(PERA::GripperControl)

