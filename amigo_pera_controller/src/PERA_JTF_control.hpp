/*! 
 * \author Sava Marinkov
 * \date May, 2011
 * \version 1.0 
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <math.h>
#include <Eigen/Eigen>

#define NUM_OF_JOINTS			8
#define PARAMS_PER_JOINT		5

using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{
	typedef std::vector<double> doubles;

	/*! \class PERA_JTF_control
	 *  \brief  Joint torque sensory feedback adaptive controller
	 * 			for the Phillips Experimental Arm
	 * 
	 */
	 	
	class PERA_JTF_control : 

	public RTT::TaskContext	{
	private:
	
		//! Inputport for the measured joint position
		InputPort<doubles> jointPosMeasuredInPort;
		//! Inputport for the measured joint velocity
		InputPort<doubles> jointVelMeasuredInPort;
		//! Inputport for the measured joint acceleration
		InputPort<doubles> jointAccMeasuredInPort;
		
		//! Inputport for the reference joint angles
		InputPort<doubles> jointPosReferenceInPort;
		//! Inputport for the reference joint velocity
		InputPort<doubles> jointVelReferenceInPort;
		//! Inputport for the reference joint acceleration
		InputPort<doubles> jointAccReferenceInPort;
				
		//! Inputport for the measured joint torques
		InputPort<doubles> jointTorqueMeasuredInPort;
		
		//! Outputport for the calculated joint control torques
		OutputPort<doubles> jointTorqueControlOutPort;
		//! Outputport for the parameter vector
		OutputPort<doubles> parameterVectorOutPort;		
		
		//! Array of joint inertias
		doubles J;
		//! Array of coulomb friction coefs
		doubles Fc;
		//! Array of viscous friction coefs
		doubles Fv;
		//! Array of torque gain coefs
		doubles k;
		//! Array of torque offset coefs
		doubles g;
		//! Array of Lambda gain values
		doubles xLambda;
		//! Array of Gamma gain values
		doubles xGamma;
		//! Array of L gain values
		doubles xL;
		//! Array of motor constants
		doubles mc;
		
		//! previous update time
		long double old_time;
		
		//! Measured joint angles
		Eigen::MatrixXd q;
		//! Measured joint angular velocities
		Eigen::MatrixXd qd;
		//! Measured joint angular accelerations
		Eigen::MatrixXd qdd;
		
		//! Reference joint angles
		Eigen::MatrixXd r;
		//! Reference joint angular velocities
		Eigen::MatrixXd rd;
		//! Reference joint angular accelerations
		Eigen::MatrixXd rdd;

		//! Calculated joint control torque
		Eigen::MatrixXd tm;

		//! Control law variable v
		Eigen::MatrixXd v;
		//! Control law variable vd
		Eigen::MatrixXd vd;		
		//! Control law variable s
		Eigen::MatrixXd s;
		
		//! Parameter vector
		Eigen::MatrixXd theta;
		//! Regressor
		Eigen::MatrixXd Y;
		//! Lambda matrix
		Eigen::MatrixXd Lambda;
		//! L matrix
		Eigen::MatrixXd L;
		//! Gamma matrix
		Eigen::MatrixXd Gamma;
		
		//! Determine the time interval between the current and the previous update
		double determineDt();
		
		doubles m_pos;
		doubles m_vel;
		doubles m_acc;
		doubles r_pos;
		doubles r_vel;
		doubles r_acc;
		doubles m_tor;		
		doubles jointControlTorque;
		doubles parameterVector;
		
		Eigen::MatrixXd y;
	public:
	
		//! Class constructor
		PERA_JTF_control(const std::string& name);
		//! Class destructor
		~PERA_JTF_control();
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

