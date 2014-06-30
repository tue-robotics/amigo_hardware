#ifndef SPINDLEHOMING_HPP
#define SPINDLEHOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace AMIGO
{
  typedef vector<double> doubles;

  class SpindleHoming
  : public RTT::TaskContext
    {
    private:

	// inports
    InputPort<std_msgs::Bool> endswitch_inport;
    InputPort<doubles> pos_inport;

	// outports
    OutputPort< vector<doubles> > ref_outport;
    OutputPort< bool > homingfinished_outport;
    OutputPort<sensor_msgs::JointState> resetRefPort;
    
    // Properties
	bool homed;
    double home_vel;
    double home_acc;
    double stroke;
    double endpos;
    
    // constants
    double referencestep;
    
	// variables
	bool homed_;
	bool goToEndpos;
	bool homingfinished;
	int cntr;
    double reference;
    doubles position;
	sensor_msgs::JointState out_msg;    
    vector<doubles> ref;
   
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
