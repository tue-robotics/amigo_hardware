/** BaseSafety.hpp
 *
 * @class BaseSafety
 *
 * \author Tim Clephas
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef BASESAFETY_HPP
#define BASESAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

namespace AMIGO
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class BaseSafety
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;

    /* Declaring and output port*/
    InputPort<doubles> errorport;
    InputPort<doubles> voltport;
    OutputPort<bool> outport;

    /* Declaring property variables */
    doubles max_errors;
    double max_voltage;

    /* Declaring global variables */
    bool safe;

    public:

    BaseSafety(const string& name);
    ~BaseSafety();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
