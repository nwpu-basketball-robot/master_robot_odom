<<<<<<< HEAD
=======
/*
*Team Unware Basketball Robot NWPU
*
*接收下位机传上来的里程计数据，并发布在tf_tree中
*
*Author = liao-zhihan
*
*first_debug_date:2016-01-20
*测试通过
*/

>>>>>>> 0b6064f447f1e965e3bb077a8ed15cc56c5b3df3
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
<<<<<<< HEAD
    void pubOdom(const double x ,const double y ,const double yaw) ;
    void odomCallBack(const basketball_msgs::robot_state::ConstPtr &ptr) ;
    void odomBroadcaster(const double x , const double y , const double yaw) ;
    void pubOdomRequest(const uint8_t func) ; 
=======
    //发布向odom话题发布数据
    void pubOdom(const double x ,const double y ,const double yaw) ;
    //下位机odom回调函数
    void odomCallBack(const basketball_msgs::robot_state::ConstPtr &ptr) ;
    //将odom数据发布在tf_tree中
    void odomBroadcaster(const double x , const double y , const double yaw) ;
    //向下位机请求里程计数据
    void pubOdomRequest(const uint8_t func) ;
>>>>>>> 0b6064f447f1e965e3bb077a8ed15cc56c5b3df3
    void setOdomVelocity(double d_x , double d_y , double d_z,nav_msgs::Odometry &odom) ;
private:
    ros::Time last_time_ ;
    ros::NodeHandle nh_ ;
    ros::Publisher odom_pub_ ;
<<<<<<< HEAD
    ros::Publisher odom_request_pub_ ; 
=======
    ros::Publisher odom_request_pub_ ;
>>>>>>> 0b6064f447f1e965e3bb077a8ed15cc56c5b3df3
    ros::Subscriber odom_data_sub_ ;
    tf::TransformBroadcaster odom_broadcaster_ ;

    double last_x_ ;
    double last_y_ ;
    double last_yaw_ ;

    double current_x_ ;
    double current_y_ ;
    double current_yaw_ ;

<<<<<<< HEAD
    int base_cmd_id_ ; 
} ;

#endif // BASKETBALL_ODOM

=======
    int base_cmd_id_ ;
} ;

#endif // BASKETBALL_ODOM
>>>>>>> 0b6064f447f1e965e3bb077a8ed15cc56c5b3df3
