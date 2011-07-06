/**************************************************************************
 *                                                                        *
 *   B. Willems                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	class CONCATENATE : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //Ports
	  InputPort<doubles> invector1Port;
	  InputPort<doubles> invector2Port;
	  OutputPort<doubles> outPort;
	  
	  doubles invector1;
	  doubles invector2;
	  doubles outvector;
	  
	public:

	  CONCATENATE(const std::string& name);
	  
	  ~CONCATENATE();
	  
	  bool configureHook();

	  bool startHook();
	  
	  void updateHook();

	  void stopHook();
	  	  
	};
	
};

