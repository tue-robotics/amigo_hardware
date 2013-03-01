/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0
 */

#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/AmigoGripperCommand.h>
#include <amigo_msgs/AmigoGripperMeasurement.h>

using namespace std;
using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{
  
  typedef vector<double> doubles;
  typedef std::vector<int> ints;
  
  /*! \class Supervisor
   *  \brief Defines Orocos component for monitoring the PERA controller
   * 
   * The Supervisor component monitors the PERA controller.
   * The following data is monitored:
   * 
   * 	* Controller error -> if too large the amplifiers are disabled.
   * 
   * 	* Requested angles -> if outside joint range amplifiers disabled.
   * 
   * 	* Emergency button state -> if pressed then the interpolators 
   * 	  are continuously reset at the measured angle.
   * 
   * 	* Controller saturation -> if the controller motorspace output
   * 	  is not higher than MOTORSAT for MAXCONSATTIME seconds.
   * 
   * 	* Active braking -> if the arm is moving towards the mechanical
   * 	  endstop to fast the supervisor will stop it with maximum
   * 	  deceleration.
   */
  
  class Supervisor
  : public RTT::TaskContext
    {
    private:

		//! Inputport for the joint angles requested by ROS inverse kinematics
		InputPort<doubles> reqJntAngPort;
		//! Inputport for the controller errors
		InputPort<doubles> jointErrorsPort;
		//! Inputport for the measured joint rel angles
		InputPort<doubles> mRelJntAngPort;
        //! Inputport for the measured joint abs angles                                              // changed from ints to doubles
        InputPort<doubles> mAbsJntAngPort;
		//! Inputport for the emergency button state
		InputPort<std_msgs::Bool> eButtonPort;
		//! Inputport for receiving gripper status 
		//InputPort<std_msgs::Bool> gripperStatusPort;
		InputPort<amigo_msgs::AmigoGripperMeasurement> gripperMeasurementPort;
		//! Inputport for receiving controller motorspace output 
		InputPort<doubles> controllerOutputPort;
		//! Inputport for receiving the reference interpolator velocity 
		InputPort<doubles> measVelPort;
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
		//! Outputport for ordering gripper to close
		//OutputPort<std_msgs::Bool> gripperClosePort;
		OutputPort<amigo_msgs::AmigoGripperCommand> gripperCommandPort;
		//! Outputport for ordering gripper to reset its positions
		OutputPort<bool> gripperResetPort;
		//! Outputport for publishing the PERA status to the AMIGO dashboard
		OutputPort<std_msgs::UInt8> peraStatusPort;
		//! Emergency button pressed / released (true/false) (message type bool)
		std_msgs::Bool eButtonPressed;
		//! Counters for loopcounting
		int cntr;
		int cntr2;
		//! Integer for storing the joint number of the joint that is being stopped
		uint breaking;
		//! Emergency button pressed / released (true/false) (memory bool)
		bool pressed;
		//! Does the deployerfile require homing
		bool REQUIRE_HOMING;
		//! Property to enable / disable output of amplifiers
		bool ENABLE_PROPERTY;
		//! Variable for storing value of PERA enableport
		bool enable;
		//! Bool indicating if homing is completed yes / no (true/false)
		bool homed;
		//! Bool making sure that jntNr is lowered after x.x seconds
		bool goodToGo;
		//! An error did occur yes / no (true/false)
		bool errors;
		//! Indicates if the rostopic /joint_references has been reset
		bool resetReference;
		//! Bool for indicating whether gripper is homed (true) or not (false)
		bool gripperHomed; 
		//! Property to define whether homing of the gripper is desired
		bool REQUIRE_GRIPPER_HOMING;
		//! Variable for storing the previous joint nr being homed
		double prevJntNr;
		//! Variable for storing the nr of the joint being homed during homing
		double jntNr;
		//! Variable for storing the previous measured jointangle during homing
		double prevAngle;
		//! Requested joint angles
		doubles jointAngles;
		//! Storage of previous sample joint angles
		doubles previousAngles;
		//! Measured joint errors
		doubles jointErrors;
		//! Vector for storing desired positions during homing
		doubles homJntAngles;
		//! Vector for storing timeinstance controller saturation was reached
		doubles timeReachedSaturation;
		//! Vector for storing positions controller should break towards
		doubles breakingPos;
		//! The startingjoint for the homing procedure
		double STRT_JNT;	
		//! The stepsize if moving towards mechanical endstop
		double STEPSIZE;	
		//! Defines the joint angles for after the reset
		doubles OFFSETANGLES;
		//! Defines the signs for publishing the reset reference angles
		doubles SIGNS;
		//! Joint lower bounds
		doubles LOWERBOUNDS;
		//! Joint upper bounds
		doubles UPPERBOUNDS;	
		//! Controller motorspace saturation values
		doubles MOTORSAT;	
		//! Maximum time the controller is allowed to be saturated
		double MAXCONSATTIME;	
		//! Tuning epsilon for dynamical breaking (margin on minimum breaking distance)
		double DYNBREAKEPS;	
		//! Specified maximum joint errors
		doubles MAX_ERRORS;
		//! The homed position (combination of rel and abs)
		doubles HOMEDPOS;	
		//! Specifies if homing is done using abs (0.0) or rel (1.0) data
		doubles ABS_OR_REL;	
		//! Specifies if the abs sensors measure opposite (-1.0) to the joint angles or in the same direction (1.0)
		doubles ABS_SEN_DIR;
		//! Specifies the joint maximum accelerations
		doubles MAXACCS;
		//! Memory array storing if controller saturation was already reached
		int firstSatInstance [8];
	
    public:

		//! Class constructor
		Supervisor(const string& name);
		//! Class destructor
		~Supervisor();

		//! Configuration sequence, executed before startHook()
		bool configureHook();
		//! Starting sequence, executed once upon startup of the component
		bool startHook();
		//! Update sequence, performed at specified rate
		void updateHook();
		//! Function outputting jointangles for homing procedure
        doubles homing(doubles jointErrors, doubles absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles);           // changed from ints to doubles
		//! Function for determining the sign of the input vector
		doubles signum(doubles a);

    };
}
#endif
