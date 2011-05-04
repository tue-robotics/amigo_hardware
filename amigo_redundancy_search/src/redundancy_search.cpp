#include <ros/ros.h>
#include <amigo_redundancy_search/redundancy_search.h>

using namespace redundancy_ns;



Redundancy::Redundancy(std::string ns): nh_private ("~"){ 
  error = false;
  
  //initialize object
  if (!init(ns)) {
    ROS_ERROR("Could not initialize reducndancy class");
    error = true;
  }
  
  //construct solvers
  constructSolvers();

}

Redundancy::~Redundancy(){
}

bool Redundancy::init(std::string ns) {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    nh.searchParam(urdf_xml,full_urdf_xml); 
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        error = true;
        return false;
    }

    // Get Root and Tip From Parameter Service
    if (!nh_private.getParam("root_name", root_name)) {
        ROS_FATAL("GenericIK: No root_name found on parameter server (namespace: %s)",ns.c_str());
        error = true;
        return false;
    }
    
    if (!nh_private.getParam("tip_name", tip_name)) {
        ROS_FATAL("GenericIK: No tip name found on parameter server (namespace: %s)",ns.c_str());
        error = true;
        return false;
    }

    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        error = true;
        return false;
    }

    nh_private.param<double>(ns+"eps",eps,0.000001);
    nh_private.param<double>("iter_factor",iter_factor,0.002);

    return true;
}

bool Redundancy::loadModel(const std::string xml) {
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        error = true;
        return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        error = true;
        return false;
    }
    if (!tree.getChain(root_name, tip_name, chain)) {
        ROS_ERROR("Could not initialize chain object");
        error = true;
        return false;
    }

    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        error = true;
        return false;
    }
 

    return true;
}

bool Redundancy::readJoints(urdf::Model &robot_model) {
    num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    //root to tip
    while (link && link->name != root_name) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            error = true;
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.limits[index].joint_name = joint->name;
            info.limits[index].has_position_limits = hasLimits;
            info.limits[index].min_position = lower;
            info.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }


    return true;
}

void Redundancy::constructSolvers(){
            
    //construct jacobian solver        
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain));

    //construct fk Solver
    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
}

bool Redundancy::computeJacobian(KDL::JntArray q, KDL::Jacobian& jac){

	if (q.rows()!=jac.columns() || jac.rows()!=6){
      ROS_ERROR("Jacobian ill sized, size is [%d x %d]",jac.rows(),jac.columns());
      ROS_ERROR("Size joint array is =%d",q.rows());
      return false;
    }
	
	//compute jacobian
	int valid = jnt_to_jac_solver_->JntToJac(q, jac);
	if (valid != 0)
	  return false;
	  
	return true;
   
}

bool Redundancy::computeJacobianPinv(KDL::Jacobian& jac, Eigen::MatrixXf& jac_, Eigen::MatrixXf& jac_pinv_){
	
	int maxiter = 100;
    
    //declare and initialize matrices
    std::vector<KDL::JntArray> U;
    KDL::JntArray S;
    std::vector<KDL::JntArray> V;
    U.resize(6,KDL::JntArray(chain.getNrOfJoints()));
    S.resize(chain.getNrOfJoints());
    V.resize(chain.getNrOfJoints(),KDL::JntArray(chain.getNrOfJoints()));
    
    //declare and initialize Eigen objects
    Eigen::MatrixXf UU = Eigen::MatrixXf::Zero(6,chain.getNrOfJoints());
    Eigen::MatrixXf VV = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),chain.getNrOfJoints());
    
    Eigen::VectorXf S_pinv = Eigen::VectorXf::Zero(chain.getNrOfJoints());
    Eigen::MatrixXf SS_pinv = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),6);
    
    Eigen::MatrixXf UUp = Eigen::MatrixXf::Zero(6,chain.getNrOfJoints()); //permuted
    Eigen::MatrixXf UUpt = Eigen::MatrixXf::Zero(6,6); //permuted and trimmed
    Eigen::MatrixXf UUt = Eigen::MatrixXf::Zero(6,6); //permuted, trimmed and transposed
    
    //resize output matrices
    jac_ = Eigen::MatrixXf::Zero(6,chain.getNrOfJoints());
	jac_pinv_ = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),6);
    
    unsigned int i,j;
	
    //convert jacobian to Eigen matrix
    for (i=0;i<jac.rows();i++){
	  for (j=0;j<jac.columns();j++){
		  jac_(i,j) = jac(i,j);
	  }
    }
	
	//perform Singular value decomposition, A = U S V'
	if (svd->calculate(jac,U,S,V,maxiter)!=0)
	  return false;
	    
    // convert U and V to Eigen matrices
    for (i=0;i<jac.rows();i++){
	  for (j=0;j<jac.columns();j++){
		  UU(i,j)=U[i](j);		  
	  }
	}
	
	for (i=0;i<jac.columns();i++){
	  for (j=0;j<jac.columns();j++){
		  VV(i,j)=V[i](j);		  
	  }
	}
    //pseudo inverse is computed as J_pinv = V * S_pinv * U'
	//Hereto, first invert S, truncate and convert to Eigen vector
	int num = 0;
	bool first = true;
	
	for (i=0;i<jac.columns();i++){
      if (fabs(S(i))<eps){
		S_pinv(i) = 0.0;
		if (first){
		  num = i;
		  first = false;
	    }
	  }
	  else
        S_pinv(i) = 1.0/S(i);
    }  
     
    //create permutation matrix to move zero column to the end 
	Eigen::MatrixXf P1 = Eigen::MatrixXf::Identity(chain.getNrOfJoints(),chain.getNrOfJoints());
    P1(num,6) = 1;
    P1(6,num) = 1;
    P1(num,num) = 0;
    P1(6,6) = 0;
	
	//permute matrix U
	UUp = UU * P1;
	//trim matrix to have size [6 x 6]
	for (i=0;i<6;i++){
	  for (j=0;j<6;j++){
	    UUpt(i,j) = UUp(i,j);
	  }
    }
	
	//transpose to get U'
	UUt = UUpt.transpose();
	
	//permute S_pinv
	S_pinv(num) = S_pinv(6);
	S_pinv(6) = 0.0;
	
	//populate SS_pinv matrix
	for (i=0;i<6;i++)
	  SS_pinv(i,i) = S_pinv(i);
  	
	//compute pseudo inverse J_pinv = V * S_pinv * U' (here, VV permuted as well)
	jac_pinv_ = VV * P1 * SS_pinv * UUt;


   return true;
}

bool Redundancy::computeRedundancyMatrix(Eigen::MatrixXf& jac_, Eigen::MatrixXf& jac_pinv_, Eigen::MatrixXf& R){
	
	Eigen::MatrixXf I = Eigen::MatrixXf::Identity(chain.getNrOfJoints(),chain.getNrOfJoints());
	R = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),chain.getNrOfJoints());
	
	R = (I - jac_pinv_ * jac_);
	
	return true;
  }

bool Redundancy::computeRedundantJointSpace(int direction, Eigen::MatrixXf& R, Eigen::VectorXf& q_c){
	
	Eigen::VectorXf input = Eigen::VectorXf::Ones(chain.getNrOfJoints());
	q_c = Eigen::VectorXf::Zero(chain.getNrOfJoints());
	
	direction = std::min<int>(std::max<int>(direction,-1),1);
	q_c = R * input * direction;
	

	return true;
  }

bool Redundancy::Eigen2KDLvector(Eigen::VectorXf& eig,KDL::JntArray& kdl){
	
	if ((int)eig.rows()!=(int)kdl.rows())
	  return false;

	for (int i=0;i<eig.rows();i++)
	  kdl(i) = eig(i);
	
	
	return true;
}

bool Redundancy::computeNormVector(Eigen::VectorXf& q, double& norm){
    norm = 0.0;
    for (int i=0;i<(int)chain.getNrOfJoints();i++)
	  norm += q(i) * q(i);
	norm = sqrt(norm);

    return true;
}
	
bool Redundancy::limCheck(KDL::JntArray q){
	bool lim = false;
	for (int i=0;i<(int)chain.getNrOfJoints();i++){
      if ((q(i) <= info.limits[i].min_position) || q(i) >= (info.limits[i].max_position)){
         ROS_DEBUG("limit violated for q_%d = %f, lim_min = %f, lim_max = %f",i,q(i),info.limits[i].min_position,info.limits[i].max_position);
         lim = true;
	  }
	  
	}
	if (lim) 
	  return false;
	  
	return true;
}

bool Redundancy::PublishRedundancy(KDL::JntArray q, int maxIter, KDL::JntArray& q_new){
	
    //KDL objects
    KDL::Jacobian jacobian_;
	jacobian_.resize(chain.getNrOfJoints());
    KDL::JntArray q_c_dot_kdl;
	q_c_dot_kdl.resize(chain.getNrOfJoints());
	KDL::JntArray q_temp;
	q_temp.resize(chain.getNrOfJoints());
	
	//Eigen objects
	Eigen::MatrixXf jacobian_eig = Eigen::MatrixXf::Zero(6,chain.getNrOfJoints());
	Eigen::MatrixXf jac_pinv_eig = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),6);
	Eigen::MatrixXf R = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),chain.getNrOfJoints());
	Eigen::VectorXf q_c_dot = Eigen::VectorXf::Zero(chain.getNrOfJoints());
    Eigen::VectorXf q_c_dot_norm  = Eigen::VectorXf::Zero(chain.getNrOfJoints());
	double norm;
	
	svd.reset(new KDL::SVD_HH(jacobian_));
	int iter;
	int direction = 1;
	bool full_range = false;
	
	//declare topic
	ros::Publisher joints_pub = nh_private.advertise<amigo_msgs::arm_joints>("arm_position_right", 50);  
	amigo_msgs::arm_joints joints_msg;
	
	ros::Rate rate(100);
	for (iter=0;iter<maxIter;iter++){
	  
	  //compute Jacobian
	  if (!computeJacobian(q,jacobian_)){
	    ROS_ERROR("Jacobian could not be computed");
	    return false;
	  }
	  //compute pseudo inverse of Jacobian
      if (!computeJacobianPinv(jacobian_,jacobian_eig,jac_pinv_eig)){
	    ROS_ERROR("Jacobian pseudo inverse could not be computed");
	    return false; 
      }
      //compute redundancy matrix R = (I - pinv(J) * J) 
      if (!computeRedundancyMatrix(jacobian_eig,jac_pinv_eig,R)){
	    ROS_ERROR("Redundancy matrix not be computed");
	    return false;
      }
      //compute vector in nullspace
      if (!computeRedundantJointSpace(direction, R, q_c_dot)){
	    ROS_ERROR("Nullspace Jacobian could not be computed");
	    return false; 
      }
      ///ROS_INFO("q_c_dot: %f %f %f %f %f %f %f",q_c_dot(0) ,q_c_dot(1),q_c_dot(2),q_c_dot(3),q_c_dot(4),q_c_dot(5),q_c_dot(6));
    
      //compute norm of vector
      if (!computeNormVector(q_c_dot, norm)){
        ROS_ERROR("Norm q_c_dot could not be computed");
        return false;
      }
      
      //check if solution is non-zero
      if (norm < 0.001 && direction ==1){
        ROS_INFO("Jacobian singularity reached after %d iterations",iter);
		direction = -1 *direction;
	  } 
	  else if (norm < 0.001 && direction ==-1){
        ROS_INFO("Jacobian singularity reached after %d iterations",iter);
        return true;
      }
      
      //normalize and scale
      q_c_dot_norm = iter_factor * q_c_dot / norm;

      //convert vector
      if (!Eigen2KDLvector(q_c_dot_norm,q_c_dot_kdl)){
        ROS_ERROR("Conversion to KDL failed");
        return false;
	  }
	  //update joint vector
      for (int i=0;i<7;i++)
        q_temp(i) = q(i) + q_c_dot_kdl(i);
      
      ROS_INFO("q_temp: %f %f %f %f %f %f %f",q_temp(0) ,q_temp(1),q_temp(2),q_temp(3),q_temp(4),q_temp(5),q_temp(6));
        
      //check if new position is within joint limits  
      if ((!limCheck(q_temp)) && (direction == 1)){
        direction = -1 *direction;
      }
      else if ((!limCheck(q_temp)) && (direction == -1)){
        ROS_WARN("Collision free joint state could not be found after %d iterations",iter);
        full_range = true;
        return true;
      }
      else{
		//update joint configuration  
        for (int i=0;i<7;i++)
          q(i) = q_temp(i);
          
      //populate msg
      for (int i=0;i<7;i++)
        joints_msg.pos[i].data = q(i);
      
      //publish msg  
      joints_pub.publish(joints_msg);           
          
      }
      rate.sleep();
    }
    if (!full_range)
      ROS_WARN("Too few iterations set to cover entire redundant space");
    
    ROS_INFO("end_reached");
    return true;
}

int Redundancy::searchRedundancy(KDL::JntArray& q, int direction, double iter_scale, KDL::JntArray& q_update){
	
	if (direction * direction != 1 ){
	  ROS_ERROR("direction needs to be 1 or -1");
	  return -7;
    }
	
    //KDL objects
    KDL::JntArray q_c_dot_kdl;
	q_c_dot_kdl.resize(chain.getNrOfJoints());
	KDL::JntArray q_temp;
	q_temp.resize(chain.getNrOfJoints());
    KDL::Jacobian jacobian_;
	jacobian_.resize(chain.getNrOfJoints());
    svd.reset(new KDL::SVD_HH(jacobian_));
	
	//Eigen objects
	Eigen::MatrixXf jacobian_eig = Eigen::MatrixXf::Zero(6,chain.getNrOfJoints());
	Eigen::MatrixXf jac_pinv_eig = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),6);
	Eigen::MatrixXf R = Eigen::MatrixXf::Zero(chain.getNrOfJoints(),chain.getNrOfJoints());
	Eigen::VectorXf q_c_dot = Eigen::VectorXf::Zero(chain.getNrOfJoints());
    Eigen::VectorXf q_c_dot_norm  = Eigen::VectorXf::Zero(chain.getNrOfJoints());
	double norm;
	
	  //compute Jacobian
	  if (!computeJacobian(q,jacobian_)){
	    ROS_ERROR("Jacobian could not be computed");
	    return -1;
	  }
	  //compute pseudo inverse of Jacobian
      if (!computeJacobianPinv(jacobian_,jacobian_eig,jac_pinv_eig)){
	    ROS_ERROR("Jacobian pseudo inverse could not be computed");
	    return -2; 
      }
      //compute redundancy matrix R = (I - pinv(J) * J) 
      if (!computeRedundancyMatrix(jacobian_eig,jac_pinv_eig,R)){
	    ROS_ERROR("Redundancy matrix could not be computed");
	    return -3;
      }
      //compute vector in nullspace q_c_dot = R * arbitr_input
      if (!computeRedundantJointSpace(direction, R, q_c_dot)){
	    ROS_ERROR("Nullspace Jacobian could not be computed");
	    return -4; 
      }
      ///ROS_INFO("q_c_dot: %f %f %f %f %f %f %f",q_c_dot(0) ,q_c_dot(1),q_c_dot(2),q_c_dot(3),q_c_dot(4),q_c_dot(5),q_c_dot(6));
    
      //compute norm of vector
      if (!computeNormVector(q_c_dot, norm)){
        ROS_ERROR("Norm q_c_dot could not be computed");
        return -5;
      }
      //normalize and scale
      q_c_dot_norm = iter_scale * q_c_dot / norm; //value was 0.005 before
      
      //check if solution is non-zero
      if (norm < 0.001)
        return 1;

      //convert vector
      if (!Eigen2KDLvector(q_c_dot_norm,q_c_dot_kdl)){
        ROS_ERROR("Conversion to KDL failed");
        return -6;
	  }
	  //update joint vector
      for (int i=0;i<7;i++)
        q_temp(i) = q(i) + q_c_dot_kdl(i);
      
      ///ROS_INFO("q_temp: %f %f %f %f %f %f %f",q_temp(0) ,q_temp(1),q_temp(2),q_temp(3),q_temp(4),q_temp(5),q_temp(6));
        
      //check if new position is within joint limits  
      if (!limCheck(q_temp)) //if lim reached
        return 1;
      else{
		//update joint configuration  
        for (int i=0;i<7;i++)
          q_update(i) = q_temp(i);
  
        return 0;
      }
}
