/*! 
 * \author Max Baeten
 * \date February, 2014
 * \version 1.0 
 */

#ifndef WRITEARMJOINTSMSG_HPP
#define WRITEARMJOINTSMSG_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

namespace AMIGO {
  class HEADEnabler
  : public RTT::TaskContext
    {
    private:
    OutputPort<bool> outport;

    public:

    HEADEnabler(const string& name);
    ~HEADEnabler();

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
    };
}
#endif
