#ifndef SPINDLEHOMINGADMITTANCE_HPP
#define SPINDLEHOMINGADMITTANCE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class SpindleHomingAdmittance
  : public RTT::TaskContext
    {
    private:

    InputPort<std_msgs::Bool> endswitch_inport;

    OutputPort< doubles> ref_outport;
    OutputPort< bool > homingfinished_outport;

    
    // Declaring variables
    bool homed;

    double home_vel;
    double home_acc;
    double stroke;
    double endpos;
    double reference;
    double referencestep;
    long long int starttime; //Debugging
    long int cntr;
    
    doubles ref;

    //double homing_correction;

	
    
    protected:
    OperationCaller<bool(string)> StartBodyPart;
    OperationCaller<bool(string)> StopBodyPart;
    OperationCaller<void(int,double)> ResetEncoder;

    public:

    SpindleHomingAdmittance(const string& name);
    ~SpindleHomingAdmittance();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
