#ifndef WriteArmDataMsg_HPP
#define WriteArmDataMsg_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float32MultiArray.h>

using namespace RTT;

namespace AMIGO
{

  typedef vector<double> doubles;

  class PlotData
  : public RTT::TaskContext
    {
    private:

    OutputPort<std_msgs::Float32MultiArray> dataPort;
    InputPort<doubles> port1;
    InputPort<doubles> port2;
    InputPort<doubles> port3;
    InputPort<doubles> port4;
    InputPort<doubles> port5;
    InputPort<doubles> port6;
    InputPort<doubles> port7;
    InputPort<doubles> port8;
    int selectJoint, selectMotor1, selectMotor2;
    doubles port1Data, port2Data, port3Data, port4Data, port5Data, port6Data, port7Data, port8Data;
    std_msgs::Float32MultiArray dataMsg;
	
	public:

    PlotData(const string& name);
    ~PlotData();

    bool configureHook();
    bool startHook();
    void updateHook();
    
    };
}
#endif
