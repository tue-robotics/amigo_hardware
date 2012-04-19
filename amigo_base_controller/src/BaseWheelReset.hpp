/** BaseWheelReset.hpp
 *
 * @class BaseWheelReset
 *
 * \author Janno Lunenburg
 * \date April, 2012
 * \version 1.0
 *
 */

#ifndef BASEWHEELRESET_HPP
#define BASEWHEELRESET_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO
{
/**
 * @brief Component that outputs the input references in case of normal operation and
 * outputs the measured position in case of unsafe operation or pressed emergency button.
 * Note that switching between both case does need an additional interpolator.
 *
 * @param No parameters
 */

class BaseWheelReset
: public RTT::TaskContext
  {
  private:

    typedef vector<double> doubles;

    /* Declaring and output port*/
    InputPort<bool> safePort;
    InputPort<std_msgs::Bool> rosEmergencyPort;
    InputPort<doubles> inputRefPort;
    InputPort<doubles> measuredPosPort;

    OutputPort<doubles> outputRefPort;

    /* Declaring global variables */
    bool safe;

    enum Status {
        NO_INFO = 0, // TODO can be given a unique number
        OK = 0,
        UNSAFE = 1

    };

    Status currentStatus;
    Status previousStatus;

    doubles correctionPos;

    doubles measInPos;
    doubles refInPos;
    doubles OutPos;

    public:

        BaseWheelReset(const string& name);
        ~BaseWheelReset();

        bool configureHook();
        bool startHook();
        void updateHook();

    };
  }
#endif
