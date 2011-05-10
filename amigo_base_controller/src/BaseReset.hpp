/** BaseReset.hpp
 *
 * @class BaseReset
 *
 * \author Tim Clephas
 * \date May, 2011
 * \version 1.0
 *
 */

#ifndef BASERESET_HPP
#define BASERESET_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace RTT;

namespace AMIGO
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class BaseReset
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;

    /* Declaring and output port*/
    InputPort<bool> safeport;
    InputPort<std_msgs::Bool> rosport;
    OutputPort<bool> resetport;

    InputPort<doubles> posport;
    OutputPort<doubles> integratorresetport;

    /* Declaring property variables */
    doubles max_velocities;
    doubles max_errors;
    double max_voltage;

    /* Declaring global variables */
    bool safe;

    public:

    BaseReset(const string& name);
    ~BaseReset();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
