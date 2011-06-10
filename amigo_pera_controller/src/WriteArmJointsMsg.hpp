/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0 
 */

#ifndef WRITEARMJOINTSMSG_HPP
#define WRITEARMJOINTSMSG_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/arm_joints.h>

using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{

  typedef vector<double> doubles;
  
  /*! \class WriteArmJointsMsg
   *  \brief Defines Orocos component for writing the measured joint 
   * 	     angles to the ROS topic joint_coordinates 
   * 
   * The component WriteArmJointsMsg receives joint angles from the 
   * operating IO, puts them in the message type amigo_msgs::arm_joints
   * and publishes them on the ROS topic joint_coordinates. 
   */

  class WriteArmJointsMsg
  : public RTT::TaskContext
    {
    private:

    //! Inputport for the joint angles to be published
    InputPort<doubles> inport;
    //! Outputport for the ROS message amigo_msgs::arm_joints
    OutputPort<amigo_msgs::arm_joints> outport;
    
    //! Signs to account for the difference in controller and DH conventio rotational directions
    doubles signalSigns;
    //! Offsets to account for difference between controller and DH convention zero-position
    doubles offsetValues;

    public:

    //! Class constructor
    WriteArmJointsMsg(const string& name);
    //! Class destructor
    ~WriteArmJointsMsg();

    //! Configuration sequence, executed before startHook()
	bool configureHook();
	//! Starting sequence, executed once upon startup of the component
	bool startHook();
	//! Update sequence, performed at specified rate
	void updateHook();
    };
}
#endif
