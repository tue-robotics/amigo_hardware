#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ocl/Component.hpp>
#include <tf/transform_broadcaster.h>

#include "PublishOdometry.hpp"

using namespace RTT;
using namespace AMIGO;

PublishOdometry::PublishOdometry(const std::string& name) : TaskContext(name)
{
  // Creating ports
  addPort( "pos", pos_port );
  addPort( "odom", odom_port );

  // Initialising variables

}
PublishOdometry::~PublishOdometry(){}

bool PublishOdometry::configureHook()
{
  return true;
}

bool PublishOdometry::startHook()
{
  old_time = os::TimeService::Instance()->getNSecs()*1e-9;
  prev_pos.assign(3,0.0);
  global_px = 0.0;
  global_py = 0.0;
  return true;
}

void PublishOdometry::updateHook()
{
  ros::Time current_time = ros::Time::now(); // Change to wall-time?
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  double dt = new_time - old_time;

  doubles pos(3);
  pos_port.read(pos);
  doubles vel(3);
  for ( uint i = 0; i <= 2; i++ )
  {
    vel[i] = ( pos[i] - prev_pos[i] )/dt;
    prev_pos[i] = pos[i];
  }

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos[2]);

  //we'll publish the odometry message over ROS
  geometry_msgs::TransformStamped tf;


  double costh = cos(pos[2]);
  double sinth = sin(pos[2]);

  double global_vx = (vel[0] * costh - vel[1] * sinth);
  double global_vy = (vel[0] * sinth + vel[1] * costh);

  double delta_x = global_vx * dt;
  double delta_y = global_vy * dt;

  global_px = global_px + delta_x;
  global_py = global_py + delta_y;

  //populate odom msg
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = current_time;
  odom.pose.pose.position.x = global_px; ///(current_pos_corr_x * costh - current_pos_corr_y * sinth);
  odom.pose.pose.position.y = global_py; ///(current_pos_corr_x * sinth + current_pos_corr_y * costh);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vel[0];
  odom.twist.twist.linear.y = vel[1];
  odom.twist.twist.angular.z = vel[2];

  ///btQuaternion quat_trans;
  ///quat_trans.setRPY(0.0, 0.0, current_pos_corr_phi);

  //populate tf msg
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  tf.header.stamp = current_time;
  tf.transform.translation.x = global_px; ///(current_pos_corr_x * costh - current_pos_corr_y * sinth);
  tf.transform.translation.y = global_py; ///(current_pos_corr_x * sinth + current_pos_corr_y * costh);
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = odom_quat;
  ///tf.transform.rotation.x = quat_trans.x();
  ///tf.transform.rotation.y = quat_trans.y();
  ///tf.transform.rotation.z = quat_trans.z();
  ///tf.transform.rotation.w = quat_trans.w();

  for (int cov_i = 0; cov_i < 36; cov_i++)
    odom.pose.covariance[cov_i] = 0.0;

  if(fabs(vel[0]) <= 1e-8 && fabs(vel[1]) <= 1e-8 && fabs(vel[2]) <= 1e-8)
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    odom.pose.covariance[0] = 1e-12;
    odom.pose.covariance[7] = 1e-12;
    odom.pose.covariance[35] = 1e-12;
    odom.pose.covariance[1] = 1e-12;
    odom.pose.covariance[6] = 1e-12;
    odom.pose.covariance[31] = 1e-12;
    odom.pose.covariance[11] = 1e-12;
    odom.pose.covariance[30] = 1e-12;
    odom.pose.covariance[5] =  1e-12;
  }
  else
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    double sigma_x = 0.002; //WHY?
    double sigma_y = 0.002; //WHY?
    double sigma_phi = 0.017; //WHY?
    double cov_x_y = 0.0;
    double cov_x_phi = 0.0;
    double cov_y_phi = 0.0;
    odom.pose.covariance[0] = pow(sigma_x,2);
    odom.pose.covariance[7] = pow(sigma_y,2);
    odom.pose.covariance[35] = pow(sigma_phi,2);
    odom.pose.covariance[1] = cov_x_y;
    odom.pose.covariance[6] = cov_x_y;
    odom.pose.covariance[31] = cov_y_phi;
    odom.pose.covariance[11] = cov_y_phi;
    odom.pose.covariance[30] = cov_x_phi;
    odom.pose.covariance[5] = cov_x_phi;
  }
  odom.pose.covariance[14] = DBL_MAX;
  odom.pose.covariance[21] = DBL_MAX;
  odom.pose.covariance[28] = DBL_MAX;

  odom.twist.covariance = odom.pose.covariance;
  //publish the message
  odom_port.write(odom);
}


ORO_CREATE_COMPONENT(AMIGO::PublishOdometry)
