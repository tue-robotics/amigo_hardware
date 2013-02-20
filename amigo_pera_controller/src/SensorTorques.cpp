#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SensorTorques.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

SensorTorques::SensorTorques(const string& name) : TaskContext(name, PreOperational), Vmeasured(N, 0.0), Tmeasured(N, 0.0), Tjoint(N, 0.0)
{
	addProperty( "Ksensor", Ksensor).doc("Ksensor [Vm]");
	addProperty( "Voffset", Voffset).doc("Voffset [V]");
	addProperty( "Xoffset", Xoffset).doc("Xoffset [m]");
	addProperty( "Stiffness", Stiffness).doc("Stiffness [N/m]");
	addProperty( "PivotDistance", PivotDistance).doc("Distance from the force act point to the pivot point [m]");
	addProperty( "SlaveLayout", SlaveLayout).doc("The layout of the connection of the motors to the slaves [-]");
	
	addEventPort("voltage_in1", voltage_inport1).doc("Optical sensors voltage1 [V]");
	addEventPort("voltage_in2", voltage_inport2).doc("Optical sensors voltage2 [V]");
	addEventPort("voltage_in3", voltage_inport3).doc("Optical sensors voltage3 [V]");	
	addPort("joint_torques_out", joint_torques_outport).doc("Joint torques [Nm]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorques::~SensorTorques(){}

bool SensorTorques::configureHook()
{	
	// This finds out some parameters needed to make the code generic. 
	NS = SlaveLayout.size(); // uncorrected size of the slavelayout (in our case this is 9 (3 slaves x 3 motor positions))

	uint counter = 0; 	
	
	for (uint i=0; i<N; i++)
	  if (SlaveLayout[i] == 1)
	    counter++;
	    
	N = counter; // corrected size of the slavelayout (in our case this is 8 (9 - 1) since we use only two motors on the first slave
	
	log(Info)<< "Creating " << N << "inputs for SensorTorques " <<endlog();
	    	  
	return true;
}

bool SensorTorques::startHook()
{
	Logger::In in("SensorTorques::startHook()");
	
	if (Ksensor.size()!=N || Voffset.size()!=N || Xoffset.size()!=N || Stiffness.size()!=N || PivotDistance.size()!=N) {
		log(Error)<<"Parameters missing! Check the sizes of Ksensor, Voffset, Stiffness and PivotDistance arrays."<< endlog();
		return false;
	}	
	if (!voltage_inport1.connected()) {
		log(Error)<<"Inputport1 not connected!"<<endlog();
		return false;
	}
		if (!voltage_inport2.connected()) {
		log(Error)<<"Inputport2 not connected!"<<endlog();
		return false;
	}
		if (!voltage_inport3.connected()) {
		log(Error)<<"Inputport3 not connected!"<<endlog();
		return false;
	}
		if (NS > 9) {
		log(Error)<<"Size of the SlaveLayout is higher than 9. Either your SlaveLayout parameter is wrong or you are trying to connect more than 3 slaves wich is hardcoded in SensorTorques"<<endlog();
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

void SensorTorques::updateHook()
{
	// Unpacking of 3 AnalogMsg Ports to a doubles (Vmeasured)
	voltage_inport1.read(Vmeasured_1);
	voltage_inport2.read(Vmeasured_2);
	voltage_inport3.read(Vmeasured_3);
	
	
	// To Do: make this fully generic for more than 3 slaves and for empty slots on slaves.
    // Slave 1002, this loop goes to 2, since there are only two ports on this slave
    for ( uint i = 0; i < 2; i++)
      Vmeasured[i] = Vmeasured_1.values[i];
    for ( uint i = 0; i < 3; i++)
      Vmeasured[i+2] = Vmeasured_2.values[i];
    for ( uint i = 0; i < 3; i++)
      Vmeasured[i+5] = Vmeasured_3.values[i];

	for (unsigned int i=0; i<N; i++) {
		Tmeasured[i] = (Ksensor[i]/(Vmeasured[i] + Voffset[i])-Xoffset[i])*Stiffness[i]*PivotDistance[i]; // Differential (gear) torques
	}
	
	// Joint torques
	Tjoint[0] =  Tmeasured[0] - Tmeasured[1];
	Tjoint[1] = -(Tmeasured[0] + Tmeasured[1]);
	Tjoint[2] =  Tmeasured[4];
	Tjoint[3] =  Tmeasured[2] + Tmeasured[3];
	Tjoint[4] =  Tmeasured[2] - Tmeasured[3];
	Tjoint[5] =  Tmeasured[6] + Tmeasured[7];
	Tjoint[6] =  Tmeasured[6] - Tmeasured[7];
	Tjoint[7] =  Tmeasured[5];
		
	measured_torques_outport.write(Tmeasured);
    joint_torques_outport.write(Tjoint);
}

ORO_CREATE_COMPONENT(SensorTorques)
