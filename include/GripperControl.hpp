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
#define GRIPPER_JOINT_INDEX_JOINTSPACE	7

using namespace std;
using namespace RTT;

namespace AMIGOPERA
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
	  // Inports
	  InputPort<amigo_msgs::AmigoGripperCommand> gripperCommandPort;
	  InputPort<doubles> torqueInPort;
	  InputPort<doubles> positionInPort;
	  InputPort<bool> resetGripperPort;
	  InputPort<bool> reNullPort;
	  
	  // Outports
	  OutputPort<doubles> gripperRefPort;
	  OutputPort<amigo_msgs::AmigoGripperMeasurement> gripperMeasurementPort;
	  
	  // Properties
	  uint sensorPos;
	  double maxPos;
	  double gripperGain;
	  double threshold_closed;
	  
	  // variables
  	  bool completed;
	  bool gripperHomed;
	  doubles torques;
	  doubles measPos;
	  doubles gripperPos;
      amigo_msgs::AmigoGripperCommand gripperCommand;
	  
	public:

	  GripperControl(const std::string& name);	  
	  ~GripperControl();
	  
	  bool configureHook();
	  bool startHook();
	  void updateHook();
	  void stopHook();
	  	  
	};
	
};

