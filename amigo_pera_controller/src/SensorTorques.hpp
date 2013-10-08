#ifndef SensorTorques_HPP
#define SensorTorques_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#define N 9
using namespace std;
using namespace RTT;

namespace AMIGO
{
  typedef vector<double> doubles;
  
  class SensorTorques
  : public RTT::TaskContext
    {
    private:

    InputPort<doubles> voltage_inport;
    OutputPort<doubles> joint_torques_outport;
    OutputPort<doubles> measured_torques_outport;

	doubles Ksensor;
	doubles Voffset;
	doubles Xoffset;
	doubles Stiffness;
	doubles Vmeasured;
	doubles Tmeasured;
	doubles Tjoint;
	doubles PivotDistance;

    public:

    SensorTorques(const string& name);
    ~SensorTorques();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
