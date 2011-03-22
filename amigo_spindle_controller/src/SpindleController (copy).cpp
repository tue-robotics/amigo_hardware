#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>
#include <stdint.h>
#include <amigo_ref_interpolator/interpolator.h>
#include <amigo_msgs/ref_point.h>
#include <amigo_msgs/spindle_setpoint.h>
#include <std_msgs/Float64.h>
#include <amigo_filters/transferfunction.h>

#include "SpindleController.hpp"
	
#define SPOED 0.004	
#define ENCODERSTEPS 2000
#define TRANSMISSION 4.8
#define VOLTLIMIT 1.0
#define	BRAKEOFF true
#define BRAKEON	false
#define	SAFETYMARGIN 0.02
#define STROKE 0.41

using namespace RTT;
using namespace AMIGO;
  
  
    SpindleController::SpindleController(const std::string& name)
        : TaskContext(name),
      //Naming the _ports and variables
      analout_port("volt_out"),
      error_port_pos("error_pos"),
      position_ref("pos_ref"),
      velocity_ref("vel_ref"),
      acceleration_ref("acc_ref"),
      checkVoltOutput("check_volt_out"),
      control_output_port("control_output"),
      current_position_port("current_position"),
      SpindleBrakePort("spindle_brake"),
      noise_port("noise"),
      spindlePosPort("spindle_position"),
      spindleSetpointPort("spindle_setpoint"),
      readEncoders_Port("read_encoders"),
      PosAfterHoming_Port("pos_after_homing"),
      
      gain_property("Gain_value","Vector", 1.0),
      addnoise_property("add_noise","Vector", false),
      noise_property("max_noise","Vector", 0.0),
      ref_position("Ref_position","Vector", 0.0),
      maximum_velocity("Max_vel","Vector", 0.1),
      maximum_acceleration("Max_acc","Vector", 0.1),
      controller_numa_property("controller_numa","Vector",0.0),
      controller_numb_property("controller_numb","Vector",0.0),
      controller_dena_property("controller_dena","Vector",0.0),
      controller_denb_property("controller_denb","Vector",0.0),
      grav_FFW("grav_FFW","Vector",0.07),
      statFric_FFW("statFric_FFW","Vector",0.05),
      dynFric_FFW("dynFric_FFW","Vector",0.4)
           
      {
        //Creating _ports
        addPort(analout_port);
        addPort(error_port_pos);
        addPort(position_ref);
        addPort(velocity_ref);
        addPort(acceleration_ref); 
        addPort(checkVoltOutput);
        addPort(control_output_port);
        addPort(current_position_port);
        addPort(SpindleBrakePort);
        addPort(noise_port);
        addPort(spindlePosPort);
        addPort(spindleSetpointPort);
        addEventPort(readEncoders_Port);
        addPort(PosAfterHoming_Port);
        
        addProperty(gain_property);
        addProperty(addnoise_property);
        addProperty(noise_property);
        addProperty(ref_position);
        addProperty(maximum_velocity);
        addProperty(maximum_acceleration);
        addProperty(controller_numa_property);
        addProperty(controller_numb_property);
        addProperty(controller_dena_property);
        addProperty(controller_denb_property);
        addProperty(grav_FFW);
        addProperty(statFric_FFW);
        addProperty(dynFric_FFW);
        
        //Initialising variables
        i = 0;
        ienc = 0.0;
        start_position = 0.0;
        FFW_gravity = 0.07;
        FFW_statFric = 0.05;
        FFW_dynFric = 0.4;
        noise = 0.0;
        controller_num.assign(2,0.0);
        controller_den.assign(2,0.0);
      }
    
    SpindleController::~SpindleController(){}

    bool SpindleController::configureHook()
    {
		homed = false;
		Gain = gain_property.get();
		max_vel = maximum_velocity.get();
		max_acc = maximum_acceleration.get();
		minimum_pos = SAFETYMARGIN;
		maximum_pos = STROKE - SAFETYMARGIN;
		
		//configuring the controller
		controller_num[0] = controller_numa_property.get();
		controller_num[1] = controller_numb_property.get();
		controller_den[0] = controller_dena_property.get();
		controller_den[1] = controller_denb_property.get();
		log(Info)<<"controller_num: "<<controller_num[0]<<" "<<controller_num[1]<<endlog();
		log(Info)<<"controller_den: "<<controller_den[0]<<" "<<controller_den[1]<<endlog();
		controller.configure(controller_num,controller_den);
		
		return true;
    }
 
    bool SpindleController::startHook()
    {
		doubles input(1,0.0);
		readEncoders_Port.read(input);
		start_position = input[0];
		homed = false;
		ready = false;
		stop  = false;
		eps_tune = 1.0;
		
		traj.setRefGen(0.0);
      
		init_time = (os::TimeService::Instance()->getNSecs())*1e-9;
		old_time = init_time;
		return true;
    }

    void SpindleController::updateHook()
    {
		//homed = true;
		
		Gain = gain_property.get();
		max_vel = maximum_velocity.get();
		max_acc = maximum_acceleration.get();
		FFW_gravity = grav_FFW.get();
		FFW_statFric = statFric_FFW.get();
		FFW_dynFric = dynFric_FFW.get();
		
		References();
		
		doubles input(1,0.0);
		readEncoders_Port.read(input);
		current_position = start_position - input[0];
		acceleration_ref.write(input[0]);
		
		checkSafety();
		addNoise();

		current_position_port.write(current_position);
		
		//used for the homing-procedure
		if(homed == false)
		{
			ref_pos = 0.5;
			max_vel = 0.01;
			max_acc = 0.1;
		}
		
		spindle_position.data = current_position;
		spindlePosPort.write(spindle_position);        
        
		double current_time = os::TimeService::Instance()->getNSecs()*1e-9;
		double dT = current_time - old_time;
		
		//generate vel and acc limited trajectory
		ref = traj.generateReference(ref_pos, max_vel, max_acc, dT, stop, eps_tune);
		
		//get reference position, speed and acceleration
		pos_ref = ref.pos;
		vel_ref = ref.vel;
		acc_ref = ref.acc;
		ready = ref.ready;
		position_ref.write(pos_ref);
		velocity_ref.write(vel_ref);
		acceleration_ref.write(acc_ref);
        	
		old_time = current_time;
              
		double error_pos = pos_ref - current_position;
		log(Info)<<"error_pos: "<<error_pos<<endlog();
		double T = Gain * controller.update(error_pos);
		control_output_port.write(T);
		log(Info)<<"controller output: "<<T<<endlog();
		
		//adding Feed Forward
		double FFW = FFW_gravity + FFW_statFric*sign(vel_ref) + FFW_dynFric*vel_ref;
		T += FFW;
		
		//adding noise if required
		T += noise;
		
		error_port_pos.write(error_pos); 
		output = -T;	
			
		if(safetycheck == false)
		{
		    output = 0.0;
		    SpindleBrakePort.write(BRAKEON);
		}
		else
		{
			SpindleBrakePort.write(BRAKEOFF);  
		}
		
		//when error builds up during homing, it is assumed to be homed
		if(abs(error_pos) > 0.005 && homed == false)
		{
			homed = true;
			output = 0.0;
			spindle_setpoint.pos = 0.35;
			PosAfterHoming_Port.write(spindle_setpoint);
			SpindleBrakePort.write(BRAKEON);
			readEncoders_Port.read(input);
			end_position = input[0];
			start_position = end_position + STROKE;
			traj.setRefGen(STROKE);			
		}
						
		limited_output =  max(-VOLTLIMIT, min(VOLTLIMIT,output) );
		checkVoltOutput.write(limited_output);
		analout_port.write(limited_output);
    }

    void SpindleController::stopHook()
    {
		output = 0.0;
        analout_port.write(output);
        SpindleBrakePort.write(BRAKEON);
    }

    void SpindleController::cleanupHook()
    {
    }
    
    void SpindleController::References()
    {
		spindleSetpointPort.read(spindle_setpoint);
		ref_pos = spindle_setpoint.pos;
		stop = spindle_setpoint.stop;		;
		
		if(homed == true)
		{
			if(ref_pos < minimum_pos)
			{
				ref_pos = minimum_pos;
			}
			if(ref_pos > maximum_pos)
			{
				ref_pos = maximum_pos;
			}
		} 	
	}
	    
	void SpindleController::checkSafety()
	{
		//	check arm_position
	
		//	check max_pos
		//if(enc_position = max_pos)
		//{
		//	return safetycheck = false;
		//}
		//  check min_pos
		//if(enc_position = min_pos)
		//{
		//	return safetycheck = false;
		//}	
	
		//  check blocking_issues
		//if(enc_position = 
		safetycheck = true;
	}
	
	int SpindleController::sign(double sign_vel)
    {
	  if (sign_vel > 0.0)
	  return 1;
	  else if (sign_vel < 0.0)
	  return -1;
	  else
	  return 0; 
	}
	
	void SpindleController::addNoise()
	{
		add_noise = addnoise_property.get();

		if(add_noise == true)
		{
			max_noise = noise_property.get(); 
			noise = rand_FloatRange(-max_noise,max_noise);
		}
		else
			noise = 0.0;
			
		noise_port.write(noise);
	}
	
	float SpindleController::rand_FloatRange(float a, float b)
    {
      return ((b-a)*((float)rand()/RAND_MAX))+a;
    }
	

ORO_CREATE_COMPONENT(SpindleController)
