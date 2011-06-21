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
#include <std_msgs/String.h>

#define PI 3.141592654
#define GRIPPER_JOINT_TORQUE_INDEX		7

using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	class Gripper_control : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //Ports
	  InputPort<bool> gripperClosePort;
	  InputPort<doubles> torqueInPort;
	  OutputPort<double> gripperRefPort;
	  OutputPort<bool> gripperStatusPort;
	  
	  doubles torques;
	  double gripperPos;
	  bool completed;
      bool gripperClose;
	  
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

