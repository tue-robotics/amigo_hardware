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

    InputPort<std_msgs::Bool> endswitch_inport;
    InputPort<doubles> pos_inport;

    OutputPort< vector<doubles> > ref_outport;
    OutputPort< bool > homingfinished_outport;
    OutputPort<sensor_msgs::JointState> resetRefPort;
    
    // Declaring variables
    bool homed;
	bool homed_;
	bool goToEndpos;
	bool homingfinished;
	int cntr;

    double home_vel;
    double home_acc;
    double stroke;
    double endpos;
    double reference;
    double referencestep;
    doubles position;
    long long int starttime; //Debugging
	sensor_msgs::JointState out_msg;
    
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
