/*! 
 * \author Max Baeten
 * \date May, 2011
 * \version 1.0
 */

#ifndef PERAHOMING_HPP
#define PERAHOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/AmigoGripperCommand.h>
#include <amigo_msgs/AmigoGripperMeasurement.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

using namespace std;
using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{
  
  typedef vector<double> doubles;
  typedef std::vector<int> ints;
  
  class PERAHoming
  : public RTT::TaskContext
    {
    private:
		//inports
		InputPort<doubles> jointErrorsPort;
		InputPort<doubles> mRelJntAngPort;
        InputPort<doubles> mAbsJntAngPort;
        InputPort<amigo_msgs::AmigoGripperMeasurement> gripperMeasurementPort;
        //outports
        OutputPort<bool> reNullPort;
        OutputPort<amigo_msgs::AmigoGripperCommand> gripperCommandPort;
		OutputPort<doubles> homJntAngPort;
		OutputPort<bool> homingfinished_outPort; 
		OutputPort<double> homingjoint_outPort; 
		OutputPort<doubles> resetIntPort;// Kept for backwards compatability
        OutputPort<doubles> resetIntPort2; // Used to reset trajectories
		
		//properties
		bool REQUIRE_HOMING;
		bool REQUIRE_GRIPPER_HOMING;
		bool Q6HOMED;
		double STRT_JNT;	
		double STEPSIZE;	
		doubles HOMEDPOS;	
		doubles ABS_OR_REL;
		doubles ABS_SEN_DIR;
		doubles MAX_ERRORS;
		
		//constants
		double FastStep;
		double SlowStep;
		double Ts;
		
		//counters
		int cntr;
		int cntr2;
		int cntr3;
		int cntr4;
		
		//variables
		bool homed_;
		bool gripperhomed_;
		bool goodToGo;
		double prevJntNr;
		double jntNr;
		double prevAngle;
		doubles jointErrors;
		doubles jointAngles;
		doubles previousAngles;
		doubles homJntAngles;

        sensor_msgs::JointState out_msg;
	
    public:

		//! Class constructor
		PERAHoming(const string& name);
		//! Class destructor
		~PERAHoming();

		bool configureHook();
		bool startHook();
		void updateHook();
        doubles homing(doubles jointErrors, doubles absJntAngles, doubles tempHomJntAngles, doubles measRelJntAngles);

    };
}
#endif
