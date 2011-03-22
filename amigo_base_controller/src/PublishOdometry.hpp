#ifndef PUBLISHODOMETRY_HPP
#define PUBLISHODOMETRY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ocl/Component.hpp>


using namespace RTT;

namespace AMIGO
{
  typedef std::vector<double> doubles; 

  class PublishOdometry
  : public RTT::TaskContext
    {
    private:

    //Declaring input- and output_ports
    InputPort<doubles> pos_port;
    OutputPort<nav_msgs::Odometry> odom_port;

    //Declaring global variables
    doubles prev_pos;
    long double old_time;
    double global_px;
    double global_py;

    public:

    PublishOdometry(const std::string& name);
    ~PublishOdometry();

    bool configureHook();
    bool startHook();
    void updateHook();

    private:

    void publishOdometry();
    };
}
#endif
