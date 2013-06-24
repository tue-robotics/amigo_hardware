#ifndef SPINDLESAFETY_HPP
#define SPINDLESAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>


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
    InputPort<std_msgs::Bool> endswitch_inport;
    OutputPort<bool> spindle_brake_outport;
    OutputPort<bool> safety_outport;
	OutputPort<std_msgs::UInt8> spindlestatus_outport;
    
    // Declaring variables
    bool safety;
    bool endswitchSafetyActivated;
    doubles error_pos;
    double errormargin;
    

    public:

    SpindleSafety(const string& name);
    ~SpindleSafety();

    bool configureHook();
    bool startHook();
    void updateHook();
    void setEndswitchSafety( bool active );

    };
}
#endif
