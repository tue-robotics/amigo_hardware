#include <ros/ros.h>
#include <amigo_redundancy_search/redundancy_search.h>
#include <amigo_redundancy_search/main.h>


using namespace redundancy_ns;




bool JointPosCallback(amigo_redundancy_search::arm_config::Request &request,
                         amigo_redundancy_search::arm_config::Response &response){
	
	 q_init(0) = request.q1;				
	 q_init(1) = request.q2;				
	 q_init(2) = request.q3;				
	 q_init(3) = request.q4;				
	 q_init(4) = request.q5;				
	 q_init(5) = request.q6;				
	 q_init(6) = request.q7;				
		
	 new_command = true;							
     return true;							 
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "redundancy search");
    
    //get namespace
    std::string ns = ros::this_node::getName();
  
    //construct InvKin object 
    Redundancy k(ns);
    
    ros::NodeHandle n;
    ros::ServiceServer arm_srv = n.advertiseService(ns+"/search_arm_config",JointPosCallback);
	
    //declare vectors
    KDL::JntArray q_new;
	q_new.resize(7);
    q_init.resize(7);
	new_command = false;
	 
	ros::Rate rate(1.0); 
	while (ros::ok()){
	  if (new_command){
	    k.PublishRedundancy(q_init,1500,q_new);	 
	    new_command = false;
	  }
	  	    
	  rate.sleep();
	  ros::spinOnce();
    }
	
    return 0;
}
