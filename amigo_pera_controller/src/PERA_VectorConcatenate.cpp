/**************************************************************************
 *                                                                        *
 *   B. Willems                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/


#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "PERA_VectorConcatenate.hpp"

using namespace std;
using namespace RTT;
using namespace PERA;

	Concatenate::Concatenate(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addEventPort("in1", invector1Port);
			addEventPort("in2", invector2Port);
			
			/// Outports
			addPort("out",outPort);
		
	  }

	Concatenate::~Concatenate(){}

	bool Concatenate::configureHook(){
		invector1.resize(7,0.0);
		invector2.resize(1,0.0);
		outvector.resize(8,0.0);
		return true;
	}

	bool Concatenate::startHook(){
		return true;
	}

	void Concatenate::updateHook(){
		
		invector1Port.read(invector1);
		invector2Port.read(invector2);
		
		outvector[0]=invector1[0];
		outvector[1]=invector1[1];
		outvector[2]=invector1[2];
		outvector[3]=invector1[3];
		outvector[4]=invector1[4];
		outvector[5]=invector1[5];
		outvector[6]=invector1[6];
		outvector[7]=invector2[0];
		
		outPort.write(outvector);
		
	}

void Concatenate::stopHook(){}

ORO_CREATE_COMPONENT(PERA::Concatenate)
