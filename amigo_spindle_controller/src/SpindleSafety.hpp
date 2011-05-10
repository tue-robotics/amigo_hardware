#ifndef SPINDLESAFETY_HPP
#define SPINDLESAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


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
    OutputPort<bool> spindle_brake_outport;
    OutputPort<bool> safety_outport;
    
    // Declaring variables
    bool safety;
    doubles error_pos;
    

    public:

    SpindleSafety(const string& name);
    ~SpindleSafety();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif