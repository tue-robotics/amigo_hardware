#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/thread/condition.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <amigo_msgs/arm_joints.h>
#include <amigo_redundancy_search/arm_config.h>


namespace redundancy_ns {


class Redundancy {
    public:
       Redundancy(std::string ns);
       ~Redundancy();
  
       //functions
       bool computeJacobian(KDL::JntArray q, KDL::Jacobian& jac);
       bool computeJacobianPinv(KDL::Jacobian& jac, Eigen::MatrixXf& jac_, Eigen::MatrixXf& jac_pinv_);
       bool computeRedundancyMatrix(Eigen::MatrixXf& jac_, Eigen::MatrixXf& jac_pinv_, Eigen::MatrixXf& R);
       bool computeRedundantJointSpace(int direction, Eigen::MatrixXf& R, Eigen::VectorXf& q_c);
       bool limCheck(KDL::JntArray q);
       bool Eigen2KDLvector(Eigen::VectorXf& eig,KDL::JntArray& kdl);
       bool computeNormVector(Eigen::VectorXf& q, double& norm);
       bool PublishRedundancy(KDL::JntArray q, int maxIter, KDL::JntArray& q_new);
       int searchRedundancy(KDL::JntArray& q, int direction, double iter_scale, KDL::JntArray& q_new);

       //KDL chains
       KDL::Chain chain;
       KDL::JntArray joint_min, joint_max;
       KDL::ChainFkSolverPos_recursive* fk_solver;
       boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; 
       boost::scoped_ptr<KDL::SVD_HH> svd;      
       
       kinematics_msgs::KinematicSolverInfo info;
       std::string root_name, tip_name;
       
       unsigned int num_joints;       
       bool error;
       ros::NodeHandle nh;

       
    private:
       void constructSolvers();
       ros::NodeHandle nh_private;
	   bool init(std::string ns);
       bool loadModel(const std::string xml);
       bool readJoints(urdf::Model &robot_model);
	   
	   double eps;	
       double iter_factor;

       
};
};

