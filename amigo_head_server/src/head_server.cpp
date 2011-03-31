#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amigo_head_server/move_headAction.h>
#include <amigo_msgs/head_ref.h>
#include <sensor_msgs/JointState.h>

class HeadServer
{
	protected:

		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<amigo_head_server::move_headAction> as_;
		std::string action_name_, field_tilt, field_pan;
		amigo_head_server::move_headFeedback feedback_;
		amigo_head_server::move_headResult result_;
		amigo_msgs::head_ref goal_;
		amigo_msgs::head_ref head_msg;
		ros::Subscriber sub_;
		ros::Publisher pub_;
		double tol_head_pan, tol_head_tilt;
		int pan_index, tilt_index;

	public:

		HeadServer(std::string name) : 
			as_(nh_, name),
			action_name_(name)
			{
			//get parameters from parameter server
			nh_.param<double> (name+"/tol_head_pan", tol_head_pan,0.015);  
			nh_.param<double> (name+"/tol_head_tilt", tol_head_tilt,0.015);   
			nh_.param<std::string> (name+"/field_pan", field_pan,"neck_pan_joint");
			nh_.param<std::string> (name+"/field_tilt", field_tilt,"neck_tilt_joint");   

			//register the goal and feeback callbacks
			as_.registerGoalCallback(boost::bind(&HeadServer::goalCB, this));
			as_.registerPreemptCallback(boost::bind(&HeadServer::preemptCB, this));

			pub_ = nh_.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 50);
			sub_ = nh_.subscribe("/joint_states", 1, &HeadServer::headCallback, this);
			pan_index=-1;
			tilt_index=-1;
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

		void headCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{

			// make sure that the action hasn't been canceled
			if (!as_.isActive())
				return;

			if (pan_index == -1) {
				for (unsigned int i = 0; i < msg->name.size(); i++) {
					if (field_pan.compare(msg->name[i]) == 0) {
						pan_index = i;
						break;
					}
				}
			}

			if (tilt_index == -1) {
				for (unsigned int i = 0; i < msg->name.size(); i++) {
					if (field_tilt.compare(msg->name[i]) == 0) {
						tilt_index = i;
						break;
					}
				}
			}			

			if ((pan_index == -1) || (tilt_index == -1)) {
				ROS_WARN("Head server: pan and tilt fields not found in the JointState msg!");
				return;
			}

			bool reached = false;

			//populate feedback msg
			feedback_.position.head_pan = msg->position[pan_index];
			feedback_.position.head_tilt = msg->position[tilt_index];

			//publish feedback
			as_.publishFeedback(feedback_);

			//determine if position is within tolerance
			if ((msg->position[pan_index] <= (head_msg.head_pan + tol_head_pan) && msg->position[pan_index] >= (head_msg.head_pan - tol_head_pan))
					&& (msg->position[tilt_index] <= (head_msg.head_tilt + tol_head_tilt) && msg->position[tilt_index] >= (head_msg.head_tilt - tol_head_tilt)))
				reached =true;  

			//if position reached  
			if (reached){
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

