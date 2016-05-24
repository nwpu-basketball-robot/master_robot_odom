#ifndef BASKETBALL_ODOM
#define BASKETBALL_ODOM

#include <basketball_msgs/robot_state.h>
#include <basketball_msgs/robot_message.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class RobotOdom
{
public:
    RobotOdom(ros::NodeHandle &node) ;
    ~RobotOdom() ;
    void mainRun() ;
protected:
private:
    void pubOdom(const double x ,const double y ,const double yaw) ;
    void odomCallBack(const basketball_msgs::robot_state::ConstPtr &ptr) ;
    void odomBroadcaster(const double x , const double y , const double yaw) ;
    void pubOdomRequest(const uint8_t func) ; 
    void setOdomVelocity(double d_x , double d_y , double d_z,nav_msgs::Odometry &odom) ;
private:
    ros::Time last_time_ ;
    ros::NodeHandle nh_ ;
    ros::Publisher odom_pub_ ;
    ros::Publisher odom_request_pub_ ; 
    ros::Subscriber odom_data_sub_ ;
    tf::TransformBroadcaster odom_broadcaster_ ;

    double last_x_ ;
    double last_y_ ;
    double last_yaw_ ;

    double current_x_ ;
    double current_y_ ;
    double current_yaw_ ;

    int base_cmd_id_ ; 
} ;

#endif // BASKETBALL_ODOM

