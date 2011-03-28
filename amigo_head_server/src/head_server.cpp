#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amigo_head_server/move_headAction.h>
#include <amigo_msgs/head_ref.h>

class HeadServer
{
protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<amigo_head_server::move_headAction> as_;
  std::string action_name_;
  amigo_head_server::move_headFeedback feedback_;
  amigo_head_server::move_headResult result_;
  amigo_msgs::head_ref goal_;
  amigo_msgs::head_ref head_msg;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  double tol_head_pan, tol_head_tilt;
  
public:
    
  HeadServer(std::string name) : 
    as_(nh_, name),
    action_name_(name)
  {
	//get parameters from parameter server
    nh_.param<double> (name+"/tol_head_pan", tol_head_pan,0.025);  
    nh_.param<double> (name+"/tol_head_tilt", tol_head_tilt,0.025);   
	  
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&HeadServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&HeadServer::preemptCB, this));

    pub_ = nh_.advertise<amigo_msgs::head_ref>(name+"/set_Head", 50);
    sub_ = nh_.subscribe(name+"/measured_head_position", 1, &HeadServer::headCallback, this);
  }

  ~HeadServer(void)
  {
  }


  void goalCB()
  {
	  // accept the new goal
      goal_ = as_.acceptNewGoal()->position;
  
      ROS_INFO("Head server: Goal accepted...");

	  head_msg.head_pan = goal_.head_pan;
	  head_msg.head_tilt = goal_.head_tilt;    
	  
	  pub_.publish(head_msg);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void headCallback(const amigo_msgs::head_ref::ConstPtr& msg)
  {
	  
	// make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    int status_head[2] = {0,0};
    
    int reached = 0;
    
    //populate feedback msg
    feedback_.position = *msg;
  
    //publish feedback
    as_.publishFeedback(feedback_);

    //determine if position is within tolerance
    if (msg->head_pan <= (head_msg.head_pan + tol_head_pan/2) && msg->head_pan >= (head_msg.head_pan - tol_head_pan/2))
      status_head[0] = 1;
    if (msg->head_tilt <= (head_msg.head_tilt + tol_head_tilt/2) && msg->head_tilt >= (head_msg.head_tilt - tol_head_tilt/2))
      status_head[1] = 1;  

    for (int i=0;i<2;i++)
      reached += status_head[i];  

    //if position reached  
    if (reached == 2){
      result_.position = feedback_.position;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
 	
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_server");
  
  std::string ns = ros::this_node::getName();
  
  //construct object
  HeadServer server_object(ns);
  
  ROS_INFO("Action server '%s' is active and spinning...",ros::this_node::getName().c_str());
  
  ros::spin();

  return 0;
}

