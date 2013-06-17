#ifndef SensorTorques2_HPP
#define SensorTorques2_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#define N 8 // todo remove this hardcoded parameter
#define maxN 10 //Maximum matrix size. Still a workaround.
using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace AMIGO
{
  typedef vector<double> doubles;
  
  class SensorTorques2
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

    uint Nrows;
    uint Ncolumns;
    doubles function[maxN];


	
    public:

    SensorTorques2(const string& name);
    ~SensorTorques2();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
