/**
 * Temporary component to replace ReadSpindleSetpoint using JointState Messages
 * Will become obsolete when new structure will be used: then the JointStateToDoubles
 * component in rtt_ROS_interfacing will be used
 */

#ifndef READSPINDLESETPOINT_HPP
#define READSPINDLESETPOINT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/spindle_setpoint.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace SPINDLE
{
  typedef vector<double> doubles;

  class ReadSpindleSetpointJointState
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<sensor_msgs::JointState> spindle_setpoint_inport;
    OutputPort<amigo_msgs::spindle_setpoint> afterhoming_outport;
    OutputPort< vector<doubles> > ref_outport;

	// Properties
    vector<doubles> ref;
    double minpos;
    double maxpos;
    double maxvel;

    public:

    ReadSpindleSetpointJointState(const string& name);
    ~ReadSpindleSetpointJointState();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
