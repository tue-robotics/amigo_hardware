#ifndef REFERENCELIMITER_HPP
#define REFERENCELIMITER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class ReferenceLimiter
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> spindle_position_inport;
    InputPort<double> refpos_inport;
    InputPort<double> refvel_inport;
    InputPort<double> refacc_inport;
    OutputPort<double> refpos_outport;
    OutputPort<double> refvel_outport;
    OutputPort<double> refacc_outport;
    
    
	// Declaring variables
	doubles current_position;

    public:

    ReferenceLimiter(const string& name);
    ~ReferenceLimiter();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
