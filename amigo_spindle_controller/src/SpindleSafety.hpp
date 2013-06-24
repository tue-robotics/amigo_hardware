#ifndef SPINDLESAFETY_HPP
#define SPINDLESAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{  
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class SpindleSafety
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> errorpos_inport;
    InputPort<bool> enable_endswitch_safety_inport;
    InputPort<std_msgs::Bool> endswitch_inport;
    OutputPort<bool> spindle_brake_outport;
    OutputPort<bool> safety_outport;
    
    // Declaring variables
    bool safety;
    bool enable_endswitch_safety;
    doubles error_pos;
    double errormargin;
    

    public:

    SpindleSafety(const string& name);
    ~SpindleSafety();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
