#ifndef READSPINDLESETPOINT_HPP
#define READSPINDLESETPOINT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/spindle_setpoint.h>


using namespace std;
using namespace RTT;

namespace SPINDLE // Just because it looks nice
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
    OutputPort< vector<doubles> > ref_outport;

    // Declaring message types
    
    vector<doubles> ref;
    double minpos;
    double maxpos;
    double maxvel;

    public:

    ReadSpindleSetpoint(const string& name);
    ~ReadSpindleSetpoint();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
