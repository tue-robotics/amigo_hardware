#ifndef REFERENCELIMITER_HPP
#define REFERENCELIMITER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <amigo_msgs/tip_ref.h>


using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;

  class ReferenceLimiter
  : public RTT::TaskContext
    {
    private:

    // Declaring input- and output_ports
    InputPort<amigo_msgs::tip_ref> right_tip_inport;
    InputPort<amigo_msgs::tip_ref> left_tip_inport;
    InputPort<doubles> spindle_position_inport;
    InputPort<double> refpos_inport;
    OutputPort<double> refpos_outport;
    OutputPort<amigo_msgs::tip_ref> reporter_port;
    
    // Declaring message types
    amigo_msgs::tip_ref right_tip;
    amigo_msgs::tip_ref left_tip;
    
	// Declaring variables
	bool onceleft;
	bool onceright;
	double ref_pos;
	doubles current_position;
	double publish_countera;
	double publish_counterb;
	double publish_counterc;

    public:

    ReferenceLimiter(const string& name);
    ~ReferenceLimiter();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
