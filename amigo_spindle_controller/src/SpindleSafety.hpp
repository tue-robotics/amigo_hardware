#ifndef SPINDLESAFETY_HPP
#define SPINDLESAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>



using namespace std;
using namespace RTT;

namespace AMIGO
{  
  typedef vector<double> doubles;

  class SpindleSafety
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> errorposPort;
    InputPort<bool> enableEndswitchSafetyPort;
    InputPort<std_msgs::Bool> endswitchPort;
    OutputPort<bool> spindlebrakePort;
    OutputPort<diagnostic_msgs::DiagnosticStatus> statusPort;
    
    // Declaring variables
    bool safety;
    bool enable_endswitch_safety;
    doubles error_pos;
    double errormargin;    
    diagnostic_msgs::DiagnosticStatus StatusError;
    diagnostic_msgs::DiagnosticStatus StatusOperational;
    
    public:

    SpindleSafety(const string& name);
    ~SpindleSafety();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
