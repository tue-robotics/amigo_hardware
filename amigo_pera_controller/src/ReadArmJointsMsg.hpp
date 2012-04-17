/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0 
 */

#ifndef READARMJOINTSMSG_HPP
#define READARMJOINTSMSG_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/arm_joints.h>

using namespace std;
using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{

  typedef vector<double> doubles;

  /*! \class ReadArmJointsMsg
   *  \brief Defines Orocos component for reading and splitting up the 
   *         ROS topic joint_references
   * 
   * The ReadArmJointsMsg component reads the joint coordinates from the 
   * ROS topic /joint_references. As this topic publishes joint angles 
   * wrt the Denavit-Hartenberg zero-position an offset is added in 
   * order to account for the difference with the controller 
   * zero-position. The same holds for the rotational direction. Signs 
   * can be introduced to account for these differences. The rosmsg 
   * received is split up in pos, vel and acc and each one of them is 
   * send over a separerate port. For the gripper position it listens to
   * the ROS topic /set_gripper.
   */
  
  class ReadArmJointsMsg
  : public RTT::TaskContext
    {
    private:

    //! Inputport for the joint angles requested by the inverse kinematics
    InputPort<amigo_msgs::arm_joints> inport;
    //! Inputport for checking watchdog status
    InputPort<bool> enablePort;
    //! Outputport for all the joint angles
    OutputPort<doubles> posport;
     //! Outputport for all the joint velocities
    OutputPort<doubles> velport;
     //! Outputport for all the joint accelerations
    OutputPort<doubles> accport;
    
    //! Offsets to account for difference between controller and DH convention zero-position
    doubles OFFSET_VALUES;
    //! Signs to account for the difference in controller and DH conventio rotational directions
    doubles SIGNAL_SIGNS;
    //! Vector for storing arm position
    doubles pos;
	//! Vector for storing arm velocity
	doubles vel;
	//! Vector for storing arm acceleration
	doubles acc;
    //! Bool to preventing updateHook to perform cycle @ t=0.0
    bool goodtogo;
    //! Bool for receiving enable signal from watchdog
    bool enable;

    public:

    //! Class constructor
    ReadArmJointsMsg(const string& name);
    //! Class destructor
    ~ReadArmJointsMsg();

    //! Configuration sequence, executed before startHook()
	bool configureHook();
	//! Starting sequence, executed once upon startup of the component
	bool startHook();
	//! Update sequence, performed at specified rate
	void updateHook();
	
    };
}
#endif
