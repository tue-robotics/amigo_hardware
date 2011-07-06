#ifndef PERA_FF_REFGEN_HPP
#define PERA_FF_REFGEN_HPP

#define maxN 10 //Maximum matrix size. Still a workaround.

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace PERA
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class PERA_Refgen
  : public RTT::TaskContext
    {
    private:
    
    int cnt;
    double dt, actJnt;
	os::TimeService::nsecs initTime;
	os::TimeService::nsecs timeNow;

    // Declaring input- and output_ports
    InputPort<doubles> posinport;
    OutputPort<doubles> posoutport;
    OutputPort<doubles> veloutport;
    OutputPort<doubles> accoutport;

    uint NrInterpolators;

    // Declaring global variables
    std::vector<refgen::RefGenerator> mRefGenerators;
    std::vector<amigo_msgs::ref_point> mRefPoints;
    doubles desiredPos;
    doubles interpolators[maxN];
    double InterpolDt, InterpolEps, T;

    public:

    PERA_Refgen(const string& name);
    ~PERA_Refgen();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
