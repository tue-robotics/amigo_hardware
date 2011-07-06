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

using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	class FEEDFORWARD : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //Ports
	  InputPort<doubles> velport;

	  OutputPort<doubles> outport;
	  	  
	  // Vector witch coulomb friction feedforward values
	  doubles KFC_VALUES;
	  doubles FF_DIR;

	public:

	  FEEDFORWARD(const std::string& name);
	  
	  ~FEEDFORWARD();
	  
	  bool configureHook();

	  bool startHook();
	  
	  void updateHook();

	  void stopHook();
	  	  
	};
	
};

