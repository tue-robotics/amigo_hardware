#ifndef OUTPUTLIMITER_HPP
#define OUTPUTLIMITER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class OutputLimiter
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> inport;
    InputPort<bool> safety_inport;
    OutputPort<double> outport;
    
    // Declaring variables
    doubles input;

    public:

    OutputLimiter(const string& name);
    ~OutputLimiter();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
