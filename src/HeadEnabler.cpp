#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "HeadEnabler.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGOHEAD;

HeadEnabler::HeadEnabler(const string& name) : TaskContext(name, PreOperational)
{
  addPort( "out", outport );
}

HeadEnabler::~HeadEnabler(){}

bool HeadEnabler::configureHook()
{
	return true;
}

bool HeadEnabler::startHook()
{
	if ( !outport.connected() ) {
		log(Warning)<<"HeadEnabler: Outputport not connected!"<<endlog();
		return false;
	}

	outport.write(true);

	return true;
}


void HeadEnabler::updateHook()
{
	return;
}

void HeadEnabler::stopHook()
{
	outport.write(false);
	return;
}

ORO_CREATE_COMPONENT(AMIGOHEAD::HeadEnabler)
