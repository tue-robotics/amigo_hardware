#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SensorTorques.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

SensorTorques::SensorTorques(const string& name) : TaskContext(name, PreOperational), Vmeasured(N, 0.0), Tmeasured(N, 0.0)
{
	addProperty( "c1", c1).doc("Calibration Coefficient c1");
	addProperty( "c2", c2).doc("Calibration Coefficient c1");
	addProperty( "c3", c3).doc("Calibration Coefficient c1");
	
	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorques::~SensorTorques(){}

bool SensorTorques::configureHook()
{	
	return true;
}

bool SensorTorques::startHook()
{
	Logger::In in("SensorTorques::startHook()");
	
	if (c1.size()!=N || c2.size()!=N || c3.size()!=N ) {
		log(Error)<<"Parameters missing! Check the sizes of c1, c2 and c3 arrays."<< endlog();
		return false;
	}	
	if (!voltage_inport.connected()) {
		log(Error)<<"Inputport not connected!"<<endlog();
		return false;
	}
	if (!measured_torques_outport.connected()) {
		log(Warning)<<"Motor torques outport not connected!"<<endlog();
	}
	return true;
}

void SensorTorques::updateHook()
{
	voltage_inport.read(Vmeasured);
	
	for (unsigned int i=0; i<N; i++) {
        Tmeasured[i] = (c1[i]/(Vmeasured[i] + c2[i])+c3[i]); // Differential (gear) torques
	}
	
	measured_torques_outport.write(Tmeasured);

}

ORO_CREATE_COMPONENT(SensorTorques)
