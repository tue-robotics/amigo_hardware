#ifndef SensorTorquesParamEstimator_HPP
#define SensorTorquesParamEstimator_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#define N 8
#define M 9
using namespace std;
using namespace RTT;

namespace AMIGO
{
  typedef vector<double> doubles;
  
  class SensorTorquesParamEstimator
  : public RTT::TaskContext
    {
    private:

    InputPort<doubles> voltage_inport;
    OutputPort<doubles> measured_torques_outport;

	int cntr;
	bool EstimationComplete;
	bool EstimationStarted;

	doubles c1;
	doubles c2;
	doubles c3;
	doubles Vmeasured_in;
	doubles Vmeasured;
	doubles Tmeasured;
	doubles Tmean;

    public:

    SensorTorquesParamEstimator(const string& name);
    ~SensorTorquesParamEstimator();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
