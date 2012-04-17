#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "ReferenceLimiter.hpp"

#define STROKE 0.413
#define SAFETYMARGIN 0.005
#define MAXVEL 1
#define MAXACC 10

using namespace std;
using namespace RTT;
using namespace AMIGO;

ReferenceLimiter::ReferenceLimiter(const string& name) : TaskContext(name, PreOperational)


{
  // Creating ports:
  addPort( "ref_pos_in", refpos_inport );
  addPort( "ref_vel_in", refvel_inport );
  addEventPort( "ref_acc_in", refacc_inport );
  addPort( "ref_pos_out", refpos_outport );
  addPort( "ref_vel_out", refvel_outport );
  addPort( "ref_acc_out", refacc_outport );
}
ReferenceLimiter::~ReferenceLimiter(){}

bool ReferenceLimiter::configureHook()
{
  return true;
}

bool ReferenceLimiter::startHook()
{
  // Check validity of Ports:
  if ( !refpos_inport.connected() ) {
    log(Error)<<"ReferenceLimiter::One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  
  if ( !refpos_outport.connected() ) {
    log(Warning)<<"ReferenceLimiter::Outputport not connected!"<<endlog();
  }
  return true;
}

void ReferenceLimiter::updateHook()
{
		double ref_pos;
			double ref_vel;
				double ref_acc;
  // Read the inputports
  refpos_inport.read(ref_pos);
  refvel_inport.read(ref_vel);
  refacc_inport.read(ref_acc);
 
  double minimum_pos = SAFETYMARGIN;
  double maximum_pos = STROKE - SAFETYMARGIN;
 
 	// Limiting the references to the minimum and maximum if necessary
	ref_pos = min(maximum_pos,max(minimum_pos,ref_pos));
	//ref_vel = min(maximum_vel,max(minimum_vel,ref_vel));
	//ref_acc = min(maximum_acc,max(minimum_acc,ref_acc));

  // Write data to ports
  refpos_outport.write( ref_pos );
  refvel_outport.write( ref_vel );
  refacc_outport.write( ref_acc );
}

ORO_CREATE_COMPONENT(ReferenceLimiter)
