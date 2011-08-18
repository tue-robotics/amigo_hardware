#ifndef READSPINDLESETPOINT_HPP
#define READSPINDLESETPOINT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/spindle_setpoint.h>


using namespace RTT;

namespace MSG // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class ReadSpindleSetpoint
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<amigo_msgs::spindle_setpoint> spindle_setpoint_inport;
    OutputPort<amigo_msgs::spindle_setpoint> afterhoming_outport;
    OutputPort<double> refpos_outport;
    
    // Declaring property
    double homed_position_property;
    
    // Declaring message types
    amigo_msgs::spindle_setpoint spindle_setpoint;
    
	// Declaring variables
	double ref_pos;

    public:

    ReadSpindleSetpoint(const string& name);
    ~ReadSpindleSetpoint();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
