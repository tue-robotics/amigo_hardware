#ifndef CALCULATEFFW_HPP
#define CALCULATEFFW_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>


using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class CalculateFFW
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> refvel_inport;
    InputPort<doubles> accref_inport;
    OutputPort<doubles> FFW_outport;
    
    // Declaring Properties
    Property<double> FFWgrav_property;
    Property<double> FFWstat_property;
    Property<double> FFWdyn_property;
    Property<double> FFWacc_property;
    
    // Declaring Variables
    double FFWgrav;
    double FFWstat;
    double FFWdyn;
    double FFWacc;
    doubles FFW;
    doubles ref_vel;
    doubles ref_acc;

    public:

    CalculateFFW(const string& name);
    ~CalculateFFW();

    bool configureHook();
    bool startHook();
    void updateHook();
    int sign(double);
    };
}
#endif
