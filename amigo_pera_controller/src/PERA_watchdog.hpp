/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0
 */

#ifndef WATCHDOG_HPP
#define WATCHDOG_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  typedef std::vector<int> ints;
  
  /*! \class WATCHDOG
   *  \brief Defines Orocos component for monitoring the PERA controller
   * 
   * The Watchdog component monitors the PERA controller. At this point
   * the following data is monitored:
   * 
   * 	* Controller error -> if too large the amplifiers are disabled
   * 
   * 	* Requested angles -> if outside joint range amplifiers disabled
   * 
   * 	* Emergency button state -> if pressed then the interpolators 
   * 	  are continuously reset at the measured angle.
   */
  
  class WATCHDOG
  : public RTT::TaskContext
    {
    private:

		//! Inputport for the joint angles requested by ROS inverse kinematics
		InputPort<doubles> reqJntAngPort;
		//! Inputport for the controller errors
		InputPort<doubles> jointErrorsPort;
		//! Inputport for the measured joint rel angles
		InputPort<doubles> mRelJntAngPort;
		//! Inputport for the measured joint abs angles
		InputPort<ints> mAbsJntAngPort;
		//! Inputport for the emergency button state
		InputPort<std_msgs::Bool> eButtonPort;
		//! Outputport for enabling/disabling the PERA_USB_IO
		OutputPort<bool> enablePort;
		//! Outputport for forwarding homing angles to the ReferenceInterpolator
		OutputPort<doubles> homJntAngPort;
		//! Outputport for resetting the ReferenceInterpolator
		OutputPort<doubles> resetIntPort;
		//! Outputport for resetting the ROS inverse kinematics topic
		OutputPort<amigo_msgs::arm_joints> resetRefPort;
		//! Outputport for disabling/enabling the ReadReferenceAngles to read topic
		OutputPort<bool> enableReadRefPort;
		//! Outputport for ordering the PERA_IO to renull
		OutputPort<bool> reNullPort;
		
		//! Emergency button pressed / released (true/false) (message type bool)
		std_msgs::Bool eButtonPressed;
		//! Counter for loopcounting
		int cntr;
		int cntr2;
		//! Emergency button pressed / released (true/false) (memory bool)
		bool pressed;
		//! Does the deployerfile require homing
		bool requireHoming;
		//! Property to enable / disable output of amplifiers
		bool enableProperty;
		//! Variable for storing value of PERA enableport
		bool enable;
		//! Bool indicating if homing is completed yes / no (true/false)
		bool homed;
		//! Bool making sure that jntNr is lowered after x.x seconds
		bool goodToGo;
		//! An error did occur yes / no (true/false)
		bool errors;
		//! Indicates if the rostopic /joint_coordinates has been reset
		bool resetReference;
		//! Variable for storing the previous joint nr being homed
		double prevJntNr;
		//! Variable for storing the nr of the joint being homed during homing
		double jntNr;
		//! Variable for storing the previous measured jointangle during homing
		double prevAngle;
		//! The stepsize if moving towards mechanical endstop
		double stepSize;	
		//! The margin used to determine if the mechanical endstop is reached
		double margin;	
		//! Defines the joint angles for after the reset
		doubles resetAngles;
		//! Vector for storing desired positions during homing
		doubles homJntAngles;
		//! Joint lower bounds
		doubles lowerBounds;
		//! Joint upper bounds
		doubles upperBounds;
		//! Requested joint angles
		doubles jointAngles;
		//! Specified maximum joint errors
		doubles maxErrors;
		//! Measured joint errors
		doubles jointErrors;	
		//! The homed position (combination of rel and abs)
		doubles homedPos;	
		//! Specifies if homing is done using abs (0.0) or rel (1.0) data
		doubles absOrRel;	
		//! Specifies if the abs sensors measure opposite (-1.0) to the joint angles or in the same direction (1.0)
		doubles absSenDir;	
	
    public:

		//! Class constructor
		WATCHDOG(const string& name);
		//! Class destructor
		~WATCHDOG();

		//! Configuration sequence, executed before startHook()
		bool configureHook();
		//! Starting sequence, executed once upon startup of the component
		bool startHook();
		//! Update sequence, performed at specified rate
		void updateHook();
		//! Function outputting jointangles for homing procedure
		doubles homing(doubles jointErrors, ints absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles);

    };
}
#endif
