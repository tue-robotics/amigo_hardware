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
    OutputPort<doubles> measured_torques_outport;

	doubles c1;
	doubles c2;
	doubles c3;
	doubles Vmeasured;
	doubles Tmeasured;

    public:

    SensorTorques(const string& name);
    ~SensorTorques();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
