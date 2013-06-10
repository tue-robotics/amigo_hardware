#ifndef SPINDLEHOMING_HPP
#define SPINDLEHOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class SpindleHoming
  : public RTT::TaskContext
    {
    private:

    InputPort<std_msgs::Bool> endswitch_inport;

    OutputPort< vector<doubles> > ref_outport;

    
    // Declaring variables
    bool homed;

    double home_vel;
    double home_acc;
    double stroke;
    double endpos;
    long long int starttime; //Debugging
    
    vector<doubles> ref;

    //double homing_correction;

	
    
    protected:
    OperationCaller<bool(string)> StartBodyPart;
    OperationCaller<bool(string)> StopBodyPart;
    OperationCaller<void(int,double)> ResetEncoder;

    public:

    SpindleHoming(const string& name);
    ~SpindleHoming();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
