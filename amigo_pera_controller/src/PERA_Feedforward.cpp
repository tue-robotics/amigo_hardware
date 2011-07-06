/**************************************************************************
 *                                                                        *
 *   B. Willems                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/

#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <math.h>
#include "PERA_Feedforward.hpp"

using namespace RTT;
using namespace PERA;

	FEEDFORWARD::FEEDFORWARD(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addEventPort("velport", velport);
			
			/// Outports
			addPort("out",outport);
			
			// Loading properties
			/// Coulomb friction feedforward
			addProperty( "kfcFeedForward", KFC_VALUES );
			addProperty( "ffdirections", FF_DIR );

	  }


	FEEDFORWARD::~FEEDFORWARD(){}

	bool FEEDFORWARD::configureHook(){

		return true;

	}

	bool FEEDFORWARD::startHook(){

		log(Info)<<"StartHook complete "<<endlog();

		return true;

	  }

	void FEEDFORWARD::updateHook(){
		
		doubles velocities;
		doubles cmdEffort;
		velocities.resize(8);
		cmdEffort.resize(8);
		
		velport.read( velocities );	

		///Coulomb friction Fc
		for ( uint i = 0; i < 8; i++ ){
			
			if (velocities[i] > 0.0){
				cmdEffort[i]=FF_DIR[i]*KFC_VALUES[i];
			}
			else if (velocities[i] < 0.0){
				cmdEffort[i]=-FF_DIR[i]*KFC_VALUES[i];
			}
			
		}
		
		outport.write(cmdEffort);

	}

	void FEEDFORWARD::stopHook(){


	}

ORO_CREATE_COMPONENT(PERA::FEEDFORWARD)

