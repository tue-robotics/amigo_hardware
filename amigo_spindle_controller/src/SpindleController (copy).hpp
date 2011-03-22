#ifndef SPINDLECONTROLLER_HPP
#define SPINDLECONTROLLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>


using namespace RTT;

namespace AMIGO // Just because it looks nice
{  
  
  typedef std::vector<double> doubles; 
    	  
  class SpindleController 
    : public RTT::TaskContext
  {
  private:  

    //Declaring input- and output_ports
    OutputPort<double> analout_port;
    OutputPort<double> error_port_pos;
    OutputPort<double> position_ref;
    OutputPort<double> velocity_ref;
    OutputPort<double> acceleration_ref;
    OutputPort<double> checkVoltOutput;
    OutputPort<double> control_output_port;
    OutputPort<double> current_position_port;
    OutputPort<bool>   SpindleBrakePort;
    OutputPort<double> noise_port;
    OutputPort<std_msgs::Float64> spindlePosPort;		
    InputPort<amigo_msgs::spindle_setpoint> spindleSetpointPort;
    InputPort<doubles> readEncoders_Port;
    OutputPort<amigo_msgs::spindle_setpoint> PosAfterHoming_Port;

	
    //Declaring variables editable in orocos
    Property<double> gain_property;
    Property<bool>	 addnoise_property;
    Property<double> noise_property;
    Property<double> ref_position;
    Property<double> maximum_velocity;
    Property<double> maximum_acceleration;
    Property<double> controller_numa_property;
    Property<double> controller_numb_property;
    Property<double> controller_dena_property;
    Property<double> controller_denb_property;
    Property<double> grav_FFW;
    Property<double> statFric_FFW;
    Property<double> dynFric_FFW;
    
    //Declaring message types
    amigo_msgs::ref_point ref;
    amigo_msgs::spindle_setpoint spindle_setpoint;
    std_msgs::Float64 spindle_position;
    
    // Declare lots of different controllers. 
    amigofilters::TransferFunctionFilter controller;
    
    //create refgen object
	refgen::RefGenerator traj;
	
	//declaring vector
	std::vector< double > controller_num;
	std::vector< double > controller_den;
 
    //Declaring variables
    double Gain;    
    double ref_pos;
    double dt;
    int i;    
    double noise;
    float max_noise;
    bool add_noise;
    double ienc;
    double enc_position;    
    double previous_enc_position;
    double current_position;
    double start_position;
    double end_position;
    bool safetycheck;
    double output;
    double limited_output;
    double maximum_pos;
    double minimum_pos;
	double max_vel;
	double max_acc;
	bool ready;
	bool stop;
	double eps_tune;
	double pos_ref, vel_ref, acc_ref;
	double init_time;
	double old_time;
	bool homed;
	double FFW_gravity;
	double FFW_statFric;
	double FFW_dynFric;
  
  
  public:
    SpindleController(const std::string& name);
    ~SpindleController();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    
    
  private:
    void References();
    void checkSafety();
    int	sign(double);
    void addNoise(); 
    float rand_FloatRange(float, float);   

  };

}
#endif
