#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SensorTorques2.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

SensorTorques2::SensorTorques2(const string& name) : TaskContext(name, PreOperational), Vmeasured(N, 0.0), Tmeasured(N, 0.0), Tjoint(N, 0.0)
{
    addProperty( "Ncolumns", Ncolumns );
    addProperty( "Nrows", Nrows );
	addProperty( "Ksensor", Ksensor).doc("Ksensor [Vm]");
	addProperty( "Voffset", Voffset).doc("Voffset [V]");
	addProperty( "Xoffset", Xoffset).doc("Xoffset [m]");
	addProperty( "Stiffness", Stiffness).doc("Stiffness [N/m]");
	addProperty( "PivotDistance", PivotDistance).doc("Distance from the force act point to the pivot point [m]");

	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("joint_torques_out", joint_torques_outport).doc("Joint torques [Nm]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorques2::~SensorTorques2(){}

bool SensorTorques2::configureHook()
{	
    for ( uint i = 0; i < Nrows; i++ )
    {
      string name = "function"+to_string(i+1);
      addProperty( name, function[i]);
    }
	return true;
}

bool SensorTorques2::startHook()
{
    Logger::In in("SensorTorques2::startHook()");
	
	if (Ksensor.size()!=N || Voffset.size()!=N || Xoffset.size()!=N || Stiffness.size()!=N || PivotDistance.size()!=N) {
		log(Error)<<"Parameters missing! Check the sizes of Ksensor, Voffset, Stiffness and PivotDistance arrays."<< endlog();
		return false;
	}	
	if (!voltage_inport.connected()) {
		log(Error)<<"Inputport not connected!"<<endlog();
		return false;
	}
	if (!joint_torques_outport.connected()) {
		log(Warning)<<"Outputport1 not connected!"<<endlog();
	}
	if (!measured_torques_outport.connected()) {
		log(Warning)<<"Outputport2 not connected!"<<endlog();
	}
	return true;
}

void SensorTorques2::updateHook()
{
	voltage_inport.read(Vmeasured);
	
	for (unsigned int i=0; i<N; i++) {
		Tmeasured[i] = (Ksensor[i]/(Vmeasured[i] + Voffset[i])-Xoffset[i])*Stiffness[i]*PivotDistance[i]; // Differential (gear) torques
	}
	
	measured_torques_outport.write(Tmeasured);
    joint_torques_outport.write(Tjoint);
}

ORO_CREATE_COMPONENT(SensorTorques2)
