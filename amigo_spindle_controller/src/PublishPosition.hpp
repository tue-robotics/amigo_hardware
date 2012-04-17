#ifndef PUBLISHPOSITION_HPP
#define PUBLISHPOSITION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class PublishPosition
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> spindle_pos_inport;
    OutputPort<std_msgs::Float64> spindle_pos_outport;
    
    // Declaring message types
    std_msgs::Float64 spindle_position;
    
    // Declaring variables
    doubles current_position;

    public:

    PublishPosition(const string& name);
    ~PublishPosition();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
