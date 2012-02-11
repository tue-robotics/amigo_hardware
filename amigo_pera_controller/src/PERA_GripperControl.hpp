/**************************************************************************
 *                                                                        *
 *   S. Marinkov                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <amigo_msgs/AmigoGripperCommand.h>
#include <amigo_msgs/AmigoGripperMeasurement.h>

#define PI 3.141592654
#define GRIPPER_JOINT_TORQUE_INDEX		5

using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	/*! \class GripperControl
	 *  \brief Defines Orocos component for controlling the gripper
	 * 
	 * The GripperControl closes the gripper using a force threshold. It
	 * opens the gripper to a predefined position.
	 */
	
	class GripperControl : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //! Inputport for receiving gripper commands
	  //InputPort<std_msgs::Bool> gripperClosePort;
	  InputPort<amigo_msgs::AmigoGripperCommand> gripperCommandPort;
	  //! Inputport for receving torques from PERA_USB_IO
	  InputPort<doubles> torqueInPort;
	  //! Inputport for resetting the gripper position
	  InputPort<bool> resetGripperPort;
	  //! Inputport for nulling the gripper reference
	  InputPort<bool> reNullPort;
	  //! Outputport for writing the desired gripper position
	  OutputPort<doubles> gripperRefPort;
	  //! Outputport for outputting the gripper status to ROS
	  //OutputPort<std_msgs::Bool> gripperStatusPort;
	  OutputPort<amigo_msgs::AmigoGripperMeasurement> gripperMeasurementPort;
	  
	  //! Vector for the read torques
	  doubles torques;
	  //! Vector containing gripper desired value
	  doubles gripperPos;
	  //! Maximum gripper position
	  double maxPos;
	  //! Constant defining the speed the gripper opens/closes with
	  double gripperGain;
	  //! Bool for storing gripper status
	  bool completed;
	  //! ROS msg bool for outputting gripper status to ROS
      //std_msgs::Bool gripperClose;
      amigo_msgs::AmigoGripperCommand gripperCommand;
	  //! Threshold force value for gripper closed
	  double threshold_closed;
	  //! Bool for gripper homed
	  bool gripperHomed;
	  
	public:

	  GripperControl(const std::string& name);
	  
	  ~GripperControl();
	  
	  //! Configuration sequence, executed before startHook()
	  bool configureHook();
	  //! Starting sequence, executed once upon startup of the component
	  bool startHook();
	  //! Update sequence, performed at specified rate
	  void updateHook();
	  //! Stopping sequence, performed upon component stopping
	  void stopHook();
	  	  
	};
	
};

