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

#define PI 3.141592654
#define GRIPPER_JOINT_TORQUE_INDEX		5

using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	class Gripper_control : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //Ports
	  InputPort<std_msgs::Bool> gripperClosePort;
	  InputPort<doubles> torqueInPort;
	  OutputPort<double> gripperRefPort;
	  OutputPort<std_msgs::Bool> gripperStatusPort;
	  
	  doubles torques;
	  double gripperPos;
	  bool completed;
      std_msgs::Bool gripperClose;
	  
	  double threshold_open;
	  double threshold_closed;
	  
	public:

	  Gripper_control(const std::string& name);
	  
	  ~Gripper_control();
	  
	  bool configureHook();

	  bool startHook();
	  
	  void updateHook();

	  void stopHook();
	  	  
	};
	
};

