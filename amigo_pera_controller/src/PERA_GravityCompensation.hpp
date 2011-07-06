/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0 
 */

#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <vector>
#include <math.h>

using namespace RTT;

//! Philips Experimental Robotic Arm namespace
namespace PERA
{
	typedef std::vector<double> doubles;
	
	/*! \class GRAVCOM
	 *  \brief Defines Orocos component for computation of torque in the
	 *         joints as a result of the gravitaty
	 * 
	 * Using the Recursive Newton-Euler procedure the torques in the 
	 * joints resulting from the gravity are computed. The 
	 * Denavit-Hartenberg convention is used to provide a robot model. 
	 * Note that at only revolute joints are supported by the algorithm.
	 */
	
	class GRAVCOM : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  //! Inputport for the measured joint angles
	  InputPort<doubles> jointAnglesPort;
	  //! Outputport for the computed torques resulting from gravity
	  OutputPort<doubles> gravCompPort;
	  
	  //! Return value for functions
	  int rv;
	  //! Vector with gravity compensation torque factors
	  doubles gCTF;
	  //! Total number of joints received from PERA_USB_IO
	  size_t nrJoints;
	  //! Number of joints that will be included in gravity compensation
	  size_t nrCompensatedJoints;
	  
	  //! Denavit-Hartenberg parameter a (link lengths)
	  Eigen::MatrixXd a;
	  //! Denavit-Hartenberg parameter d (link offsets)
	  Eigen::MatrixXd d;
	  //! Denavit-Hartenberg parameter alpha (link twists)
	  Eigen::MatrixXd alpha;
	  //! Centers of gravity
	  Eigen::MatrixXd coglist;
	  //! Masses of each link
	  Eigen::MatrixXd mlist;
	  //! Inertia matrices of each link
	  Eigen::MatrixXd Istore;
	  //! Gravitational acceleration
	  Eigen::MatrixXd grav;
	  //! Denavit-Hartenberg parameter q (joint angles)
	  Eigen::MatrixXd q;
	  //! Gravity compensation torques
	  Eigen::MatrixXd gravComp;

	public:

	  //! Class constructor
	  GRAVCOM(const std::string& name);
	  //! Class destructor
	  ~GRAVCOM();
	  
	  //! Configuration sequence, executed before startHook()
	  bool configureHook();
	  //! Starting sequence, executed once upon startup of the component
	  bool startHook();
	  //! Update sequence, performed at specified rate
	  void updateHook();
	  //! Stopping sequence, executed once upon stop of the component
	  void stopHook();
	  
	  //! Function for computing the rotation matrix from link i-1 to link i using the DH-parameters
	  Eigen::Matrix3d ComputeRotationMatrix(double d,double alpha,double q);
	  //! Function for computing the rotation matrix from Euler angles
	  Eigen::Matrix3d eul2rot(double phi, double theta, double psi);
	  //! Compute Recursive Newton Euler (only for revolute joints)
	  Eigen::MatrixXd rne(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd qd,Eigen::MatrixXd qdd,Eigen::MatrixXd grav, Eigen::MatrixXd Fext);
	  //! Function determining the gravitational part of the solution of the RNE procedure
	  Eigen::MatrixXd ComputeGravity(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd grav);
	  
	};
	
};

