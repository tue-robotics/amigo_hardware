#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SensorTorquesE.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

SensorTorquesE::SensorTorquesE(const string& name) : TaskContext(name, PreOperational), Vmeasured(N, 0.0), Tmeasured(N, 0.0), Tjoint(N, 0.0)
{
	addProperty( "Ksensor", Ksensor).doc("Ksensor [Vm]");
	addProperty( "Voffset", Voffset).doc("Voffset [V]");
	addProperty( "Xoffset", Xoffset).doc("Xoffset [m]");
	addProperty( "Stiffness", Stiffness).doc("Stiffness [N/m]");
	addProperty( "PivotDistance", PivotDistance).doc("Distance from the force act point to the pivot point [m]");
	
	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("joint_torques_out", joint_torques_outport).doc("Joint torques [Nm]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorquesE::~SensorTorquesE(){}

bool SensorTorquesE::configureHook()
{	
	return true;
}

bool SensorTorquesE::startHook()
{
	Logger::In in("SensorTorques::startHook()");
	
	if (Ksensor.size()!=N || Voffset.size()!=N || Xoffset.size()!=N || Stiffness.size()!=N || PivotDistance.size()!=N) {
		log(Error)<<"Parameters missing! Check the sizes of Ksensor, Voffset, Stiffness and PivotDistance arrays."<< endlog();
		return false;
	}	
	if (!voltage_inport.connected()) {
		log(Error)<<"Inputport not connected!"<<endlog();
		return false;
	}
	//if (!joint_torques_outport.connected()) {
    //    log(Info)<<"Outputport1 not connected!"<<endlog();
	//}
	if (!measured_torques_outport.connected()) {
        log(Warning)<<"Outputport2 not connected!"<<endlog();
	}

	return true;
	
}

void SensorTorquesE::updateHook()
{
	voltage_inport.read(Vmeasured);
	
	for (unsigned int i=0; i<9; i++) {
        Tmeasured[i] = (Ksensor[i]/(Vmeasured[i] + Voffset[i])-Xoffset[i])*Stiffness[i]*PivotDistance[i]; // Differential (gear) torques
	}
	
    // Joint torques                                     // TO DO: check these values, but better way would be to multiply the sensor torques with the same matrix as in erpera.ops
    //Tjoint[0] = -Tmeasured[0] + Tmeasured[1];
    //Tjoint[1] =  Tmeasured[0] + Tmeasured[1];
    //Tjoint[2] =  Tmeasured[3];
    //Tjoint[3] =  Tmeasured[4] + Tmeasured[5];
    //Tjoint[4] = -Tmeasured[4] + Tmeasured[5];
    //Tjoint[5] =  Tmeasured[6] + Tmeasured[7];
    //Tjoint[6] =  Tmeasured[6] - Tmeasured[7];
    //Tjoint[7] =  Tmeasured[8];
             
	measured_torques_outport.write(Tmeasured);
    //joint_torques_outport.write(Tjoint);
}

ORO_CREATE_COMPONENT(SensorTorquesE)
