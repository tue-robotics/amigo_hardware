#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SensorTorquesParamEstimator.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

SensorTorquesParamEstimator::SensorTorquesParamEstimator(const string& name) : TaskContext(name, PreOperational), Vmeasured_in(M, 0.0), Vmeasured(N, 0.0), Tmeasured(N, 0.0), Tmean(N, 0.0)
{
	addProperty( "c1", c1).doc("Calibration Coefficient c1");
	addProperty( "c2", c2).doc("Calibration Coefficient c2");
	addProperty( "c3", c3).doc("Calibration Coefficient c3");
	
	addEventPort("voltage_in", voltage_inport).doc("Optical sensors voltage [V]");
	addPort("measured_torques_out", measured_torques_outport).doc("Differential (gear) torques [Nm]");
	
}
SensorTorquesParamEstimator::~SensorTorquesParamEstimator(){}

bool SensorTorquesParamEstimator::configureHook()
{	
	return true;
}

bool SensorTorquesParamEstimator::startHook()
{
	Logger::In in("SensorTorquesParamEstimator::startHook()");
	
	if (c1.size()!=N || c2.size()!=N || c3.size()!=N ) {
		log(Error)<<"SensorTorquesParamEstimator: Parameters missing! Check the sizes of c1, c2 and c3 arrays."<< endlog();
		return false;
	}	
	if (!voltage_inport.connected()) {
		log(Error)<<"SensorTorquesParamEstimator: Inputport not connected!"<<endlog();
		return false;
	}
	if (!measured_torques_outport.connected()) {
		log(Warning)<<"SensorTorquesParamEstimator: Motor torques outport not connected!"<<endlog();
	}
	return true;
	
	cntr = 0;
	EstimationComplete = false;
	EstimationStarted = false;
}

void SensorTorquesParamEstimator::updateHook()
{
	// read size 9 input doubles
	voltage_inport.read(Vmeasured_in);
	// remove obsolete joint from doubles
	Vmeasured[0] = Vmeasured_in[0];
	Vmeasured[1] = Vmeasured_in[1];
	for (unsigned int i=2; i<N; i++) {
		Vmeasured[i] = Vmeasured_in[i+1];
	}
	
	for (unsigned int i=0; i<N; i++) {
        Tmeasured[i] = (c1[i]/(Vmeasured[i] + c2[i])+c3[i]); // Differential (gear) torques
	}
	
	if (EstimationComplete == false) {
		if (cntr >= 500  && EstimationStarted == false) {
			EstimationStarted = true;
			cntr = 0;
		}
		else if (cntr >= 1000  && EstimationStarted == true) {
			EstimationComplete = true;
			c3 = 0.0;//
		}
		else if (EstimationStarted == true) {
			doubles Tmean_(N, 0.0)
			Tmean_ = Tmean
			for (unsigned int i=0; i<N; i++) {
				Tmean[i] = (Tmean_[i]*(double cntr) + Tmeasured[i])/((double cntr)+1)
			}
		}		
		cntr++
	}
	else {
		measured_torques_outport.write(Tmeasured);		
	}
}

ORO_CREATE_COMPONENT(SensorTorquesParamEstimator)
