/**************************************************************************
 *                                                                        *
 *   B. Willems                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/

#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <vector>
#include <math.h>

using namespace RTT;

namespace PERA
{
	typedef std::vector<double> doubles;
	
	class Controller {
    public:
        Controller();
        
        double Evaluate( double err );
        
        int SetParameters( double kp, double ki, double kd, double ilim, double tS );

    private:
        double xample;

        double mKp;
        double mKi;
        double mKd;
        double mILim;

        double mErr;

        double mOutP;
        double mOutD;
        double mSumI;
        double mOut;

    };
	
	class PERACONTROLLERS : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  // Loop counter
	  int cntr;

	  // Return value for functions
	  int rv;
	  
	  // Actuator constant
	  int Kact;
	  
	  // Variables for time measurements
	  double dt;
	  os::TimeService::nsecs initTime;
	  os::TimeService::nsecs timeNow;
	  os::TimeService::nsecs prevTime;
	  os::TimeService::nsecs deltaTime;
	  
	  //Logging ports
	  InputPort<doubles> errPort;

	  OutputPort<doubles> tmpCmdEffortPort;
	  	  
	  // Vectors with controllers parameters
	  doubles q1ConVal, q2ConVal, q3ConVal, q4ConVal, q5ConVal, q6ConVal, q7ConVal, q8ConVal;
	  size_t nrControllers;
	  	  
	  // Vector witch coulomb friction feedforward values
	  doubles kfcValues;
	  
	  // Vector with gravity compensation torque factors
	  doubles gCTF;
	  	  
	  // Sampling time
	  double Ts;
	  
	  // Gravity Compensation parameters
	  ///DH parameters
	  Eigen::MatrixXd a;
	  Eigen::MatrixXd d;
	  Eigen::MatrixXd alpha;
	  /// Centers of gravity
	  Eigen::MatrixXd coglist;
	  /// Masses of each link
	  Eigen::MatrixXd mlist;
	  /// Inertia matrices of each link
	  Eigen::MatrixXd Istore;
	  /// Gravitational acceleration
	  Eigen::MatrixXd grav;
	  /// Joint angle vector
	  Eigen::MatrixXd q;
	  /// Gravity compensation torques
	  Eigen::MatrixXd gravComp;

	public:
	  // Vector containing the controllers
	  std::vector<Controller> mControllers;

	  PERACONTROLLERS(const std::string& name);
	  
	  ~PERACONTROLLERS();
	  
	  bool configureHook();

	  bool startHook();
	  
	  void updateHook();

	  void stopHook();
	  
	  Eigen::Matrix3d ComputeRotationMatrix(double d,double alpha,double q);

	  Eigen::Matrix3d eul2rot(double phi, double theta, double psi);

	  Eigen::MatrixXd ComputeGravity(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd grav);

	  Eigen::MatrixXd rne(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd qd,Eigen::MatrixXd qdd,Eigen::MatrixXd grav, Eigen::MatrixXd Fext);
	  
	};
	
};

