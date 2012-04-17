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

using namespace std;
using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	/*! \class Concatenate
	 *  \brief Defines Orocos component for vector concatenation (PERA
	 *   only, non generic)
	 * 
	 * This component was created because the standard vector 
	 * concatenate does not use eventsports thus running at a fixed
	 * frequency. Component only developed for PERA application
	 * therefor not generic.
	 */
	
	class Concatenate : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //! Inputport for first vector
	  InputPort<doubles> invector1Port;
	  //! Inputport for second vector
	  InputPort<doubles> invector2Port;
	  //! Outputport for outputting the concatenated vector
	  OutputPort<doubles> outPort;
	  
	  //! First vector
	  doubles invector1;
	  //! Second vector
	  doubles invector2;
	  //! Outputvector
	  doubles outvector;
	  
	public:

	  Concatenate(const std::string& name);
	  
	  ~Concatenate();
	  
	  //! Configuration sequence, executed before startHook()
	  bool configureHook();
	  //! Starting sequence, executed once upon startup of the component
	  bool startHook();
	  //! Update sequence, performed at specified rate
	  void updateHook();
	  //! Stopping sequence, executed once upon stop of the component
	  void stopHook();
	  	  
	};
	
};

