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
			addPort("gripper_close", gripperClosePort);
			addPort("torque_in", torqueInPort);
			
			/// Outports
			addPort("gripper_ref",gripperRefPort);
			addPort("gripper_status",gripperStatusPort);
			
			/// Thresholds for the gripper force
			addProperty( "threshold_open", threshold_open);
			addProperty( "threshold_closed", threshold_closed);			
					
	  }

	Gripper_control::~Gripper_control(){}

	bool Gripper_control::configureHook(){
		torques.resize(8);
		gripperPos = 0.0;
		completed = true;
		return true;
	}

	bool Gripper_control::startHook(){
		return true;
	}

	void Gripper_control::updateHook(){
		if (gripperClosePort.read(gripperClose) == NewData) {
			completed = false;
		}
		if (!completed) {
			torqueInPort.read(torques);
			if(!gripperClose.data){
				if (torques[GRIPPER_JOINT_TORQUE_INDEX] < threshold_open) {
					log(Info)<<"Gripper is OPEN"<<endlog();
					std_msgs::Bool gripperStatus;
					gripperStatus.data = false;
					gripperStatusPort.write(gripperStatus);
					completed = true;
				} else {
					gripperPos += 0.009375*PI/180;
				}
			} else {
				if (torques[GRIPPER_JOINT_TORQUE_INDEX] > threshold_closed) {
					log(Warning)<<"Gripper is CLOSED"<<endlog();
					std_msgs::Bool gripperStatus;
					gripperStatus.data = true;
					gripperStatusPort.write(gripperStatus);
					completed = true;
				} else {
					log(Warning)<<"gripper torques = "<<torques[GRIPPER_JOINT_TORQUE_INDEX]<<endlog();
					gripperPos -= 0.009375*PI/180;
				}
			}
			gripperRefPort.write(gripperPos);
		}
	}

void Gripper_control::stopHook(){}

ORO_CREATE_COMPONENT(PERA::Gripper_control)

