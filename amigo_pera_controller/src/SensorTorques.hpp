#ifndef SensorTorques_HPP
#define SensorTorques_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>

using namespace std;
using namespace RTT;

namespace AMIGO
{
  typedef vector<double> doubles;
  
  class SensorTorques
  : public RTT::TaskContext
    {
    private:

    InputPort<soem_beckhoff_drivers::AnalogMsg> voltage_inport1;
    InputPort<soem_beckhoff_drivers::AnalogMsg> voltage_inport2;
    InputPort<soem_beckhoff_drivers::AnalogMsg> voltage_inport3;
    OutputPort<doubles> joint_torques_outport;
    OutputPort<doubles> measured_torques_outport;
	
	uint N;
	uint NS;
	doubles Ksensor;
	doubles Voffset;
	doubles Xoffset;
	doubles Stiffness;
	doubles SlaveLayout;
	soem_beckhoff_drivers::AnalogMsg Vmeasured_1;
	soem_beckhoff_drivers::AnalogMsg Vmeasured_2;
	soem_beckhoff_drivers::AnalogMsg Vmeasured_3;
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
