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

#include "PERA_gripper_control.hpp"

using namespace RTT;
using namespace PERA;

	Gripper_control::Gripper_control(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addPort("gripper_close", gripperInPort);
			addPort("torque_in", torqueInPort);
			
			/// Outports
			addPort("gripper_ref",gripperPosOutPort);
			addPort("gripper_status",gripperStatusPort);
			
			/// Thresholds for the gripper force
			addProperty( "threshold_open", threshold_open);
			addProperty( "threshold_closed", threshold_closed);			
			
			/// Compensationfactor left/right gripper encoder
			addProperty( "encoder_compensation", compFactor);			
	  }

	Gripper_control::~Gripper_control(){}

	bool Gripper_control::configureHook(){
		torques.resize(8);
		gripperPos = 0.0;
		prevGripperPos = 0.0;
		completed = true;
		return true;
	}

	bool Gripper_control::startHook(){
		return true;
	}

	void Gripper_control::updateHook(){
		torqueInPort.read(torques);
		if (gripperInPort.read(gripperData) == NewData) {
			completed = false;
			if(!gripperData.data){
				log(Debug)<<"Received a command to OPEN a gripper.."<<endlog();
				gripperStatus.data = string("openning");
			}else{
				log(Debug)<<"Received a command to CLOSE a gripper.."<<endlog();
				gripperStatus.data = string("closing");
			}
			gripperStatusPort.write(gripperStatus);
		} 
		else {
			if (!completed) {
				if(!gripperData.data){
					if (torques[GRIPPER_JOINT_TORQUE_INDEX] < threshold_open) {
						log(Debug)<<"Gripper is OPEN"<<endlog();
						gripperStatus.data = string("open");
						gripperStatusPort.write(gripperStatus);
						completed = true;
					} else {
						gripperPos += (0.009375*PI/180)*compFactor;
					}
				}
				else {
					if (torques[GRIPPER_JOINT_TORQUE_INDEX] > threshold_closed) {
						log(Debug)<<"Gripper is CLOSED"<<endlog();
						gripperStatus.data = string("closed");
						gripperStatusPort.write(gripperStatus);
						completed = true;
					} else {
						gripperPos -= (0.009375*PI/180)*compFactor;
					}
				}
			}
		}
		if(gripperPos != prevGripperPos){
			gripperPosOutPort.write(gripperPos);
			prevGripperPos = gripperPos;
		}
	}

	void Gripper_control::stopHook(){
	}

ORO_CREATE_COMPONENT(PERA::Gripper_control)

