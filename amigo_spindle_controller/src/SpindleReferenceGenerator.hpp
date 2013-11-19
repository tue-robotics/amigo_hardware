#ifndef SPINDLEREFERENCEGENERATOR_HPP
#define SPINDLEREFERENCEGENERATOR_HPP

#define maxN 10 //Maximum matrix size. Still a workaround.

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SPINDLE
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class SpindleReferenceGenerator
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<doubles> posinport;
    InputPort< vector<doubles> > refinport;
    InputPort<doubles> actualposinport;
    InputPort<doubles> resetPort;
    OutputPort<doubles> posoutport;
    OutputPort<doubles> veloutport;
    OutputPort<doubles> accoutport;
    InputPort<bool> homingfinished_inport;

    uint NrInterpolators;

    // Declaring global variables
    std::vector<refgen::RefGenerator> mRefGenerators;
    std::vector<amigo_msgs::ref_point> mRefPoints;
    
    vector<doubles> refin;
    doubles desiredPos;
    doubles desiredVel;
    doubles desiredAcc;
    doubles interpolators[maxN];
    double InterpolDt, InterpolEps;
    bool finishedhoming;

    public:

    SpindleReferenceGenerator(const string& name);
    ~SpindleReferenceGenerator();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
