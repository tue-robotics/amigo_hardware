#ifndef SPINDLEHOMING_HPP
#define SPINDLEHOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class SpindleHoming
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> encoder_inport;
    InputPort<doubles> errorpos_inport;
    InputPort<double> refpos_inport;
    InputPort<doubles> currentpos_inport;
    OutputPort<doubles> refpos_outport;
    OutputPort<doubles> correction_outport;
    OutputPort<doubles> reset_generator_outport;
    
    // Declaring properties
    Property<double> maxvel_property;
    Property<double> maxacc_property;
    
    // Declaring variables
    bool homed;
    doubles ref_pos;
    double refpos;
    doubles input;
    doubles correction;
    doubles error_pos;
    doubles current_pos;
    doubles generator_reset;
    double maxvel;
    double maxacc;
	double homing_correction;
	
    public:

    SpindleHoming(const string& name);
    ~SpindleHoming();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
