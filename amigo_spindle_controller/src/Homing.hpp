#ifndef HOMING_HPP
#define HOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  typedef vector<int> ints;

  class Homing
  : public RTT::TaskContext
    {
    private:
    // Declaring Inports
    InputPort<std_msgs::Bool> endSwitch_inport;
    InputPort<ints> absPos_inport;
    InputPort<doubles> force_inport;
    InputPort<doubles> servoError_inport;
    InputPort<doubles> relPos_inport;

    // Declaring Outports
    OutputPort< vector<doubles> > ref_outport;


    // Declaring system variables
    bool homed;
    bool HomingConstraintMet;
    bool GoToMidPos;
    uint N;
    uint JntNr;

    // homing variables
    string homing_body;
    doubles homing_type;
    ints homing_order;
    doubles homing_refPos;
    doubles homing_refVel;
    doubles homing_midpos;
    doubles homing_endpos;
    doubles homing_stroke;

    //ref
    vector<doubles> ref;

    // Current value variables
    ints absPos;
    std_msgs::Bool endSwitch;
    doubles servoErrors;
    doubles forces;
    doubles relPos;

    // Homing criterion variables
    ints homing_absPos;
    doubles homing_force;
    doubles homing_error;

    protected:
    OperationCaller<bool(string)> StartBodyPart;
    OperationCaller<bool(string)> StopBodyPart;
    OperationCaller<void(int,double)> ResetEncoder;
    
    public:

    Homing(const string& name);
    ~Homing();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
