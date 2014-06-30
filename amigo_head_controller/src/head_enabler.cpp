#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "head_enabler.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

HEADEnabler::HEADEnabler(const string& name) : TaskContext(name, PreOperational)
{
  addPort( "out", outport );
}

HEADEnabler::~HEADEnabler(){}

bool HEADEnabler::configureHook()
{
	return true;
}

bool HEADEnabler::startHook()
{
	if ( !outport.connected() ) {
		log(Warning)<<"HEADEnabler: Outputport not connected!"<<endlog();
		return false;
	}

	outport.write(true);

	return true;
}


void HEADEnabler::updateHook()
{
	return;
}

void HEADEnabler::stopHook()
{
	outport.write(false);
	return;
}

ORO_CREATE_COMPONENT(AMIGO::HEADEnabler)
