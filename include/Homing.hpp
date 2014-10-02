#ifndef HOMING_HPP
#define HOMING_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace AMIGOGENERIC
{
    /*! \class Homing
    *  	\brief Defines Orocos component for homing hardware
    *
    *   Component should run on Ts
    *
    *   prestate 0 - send homing reference
    *   state 1 - evaluate homing criterion     true    -> send joint to midpos and go to state 2
    *                                           false   -> wait for true
    *   state 2 - evaluate mid pos reached      true    -> go to next joint and prestate 0
    *                                           false   -> wait for true
    *
    *   functions
    *   evaluateHomingCriterion(jointNr)
    */


    typedef vector<double> doubles;
    typedef vector<int> ints;

    class Homing
    : public RTT::TaskContext
    {
        private:

        // inports
        InputPort<doubles> pos_inport;
        InputPort<std_msgs::Bool> endswitch_inport;
        InputPort<doubles> jointerrors_inport;
        InputPort<ints> absPos_inport;
        InputPort<doubles> forces_inport;

        // outports
        OutputPort< vector<doubles> > ref_outport;
        OutputPort< bool > homingfinished_outport;

        // Properties
        uint N;
        string bodypart;
        string prefix;

        vector<string> homing_type;
        doubles require_homing;
        doubles homing_order;
        doubles homing_direction;
        doubles homing_velocity;
        doubles homing_stroke;
        doubles homing_midpos;
        doubles homing_endpos;

        doubles homing_forces;
        doubles homing_errors;
        vector<uint> homing_absPos;

        // Implicit properties
        bool endswitchhoming;
        bool absolutehoming;
        bool forcehoming;
        bool errorhoming;

        // variables
        int jointNr;
        int state;
        bool joint_finished;
        bool firsttime;
        doubles position;
        vector<doubles> ref_out;
        vector<doubles> ref_out_prev;


        protected:
        OperationCaller<bool(string)> StartBodyPart;
        OperationCaller<bool(string)> StopBodyPart;
        OperationCaller<void(int,double)> ResetEncoder;

        public:

        Homing(const string& name);
        ~Homing();

        bool configureHook();
        bool startHook();
        void updateHook();
        bool evaluateHomingCriterion(uint jointNr);
        void updateHomingRef( uint jointNr);
        void updateMidposRef( uint jointNr);

    };
}
#endif
