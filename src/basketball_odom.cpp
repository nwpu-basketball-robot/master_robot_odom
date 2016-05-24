//author : rescuer liao
//https://github.com/rescuer-liao
//date : 2016 - 1 - 22
//Team Explorer(rescue robot)
//Team Unware (NWPU Basketball robot)
//this package get odometry data from handware and send these data to tf_tree and ros stack

#include <basketball_odom/basketball_odom.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/Quaternion.h>

//#define DEBUG
RobotOdom::RobotOdom(ros::NodeHandle &node):
    nh_(node),
    last_time_(ros::Time::now()),
    last_x_(0.0),
    last_y_(0.0),
    last_yaw_(0.0),
    current_x_(0.0),
    current_y_(0.0),
    current_yaw_(0.0),
    base_cmd_id_(0x01)
{
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50) ;
    odom_request_pub_ = nh_.advertise<basketball_msgs::robot_message>("robot_cmd",1000) ;
    std::stringstream sub_name ;
    sub_name<<"/RecvData/"<<(int)base_cmd_id_ ;
    odom_data_sub_ = nh_.subscribe(sub_name.str(),100, &RobotOdom::odomCallBack,this) ;
}

RobotOdom::~RobotOdom()
{
    nh_.shutdown() ;
}


void RobotOdom::pubOdomRequest(const uint8_t func)
{
	basketball_msgs::robot_message robot_cmd_msg ;
	robot_cmd_msg.data.resize(6 , 0) ;
	uint8_t *data_ptr = robot_cmd_msg.data.data() ;
	int data_len = 1 ;
	data_ptr[0] = data_ptr[1] = 0xff ;
	data_ptr[2] = base_cmd_id_ ;
	data_ptr[3] = (u_int8_t)(data_len>>8) ;
    data_ptr[4] = (u_int8_t)(data_len & 0xff) ;
    data_ptr[5] = func ;
	odom_request_pub_.publish(robot_cmd_msg) ;
}

void RobotOdom::setOdomVelocity(double d_x , double d_y , double d_z,nav_msgs::Odometry &odom)
{
    double d_time = (ros::Time::now() - last_time_).toSec() ;
    double vx = d_x / d_time ;
    double vy = d_y / d_time ;
    double vth = d_z / d_time ;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
}

void RobotOdom::mainRun()
{
    ros::Rate r(10) ;
    while(ros::ok())
    {
        pubOdom(current_x_ , current_y_ , current_yaw_) ;
        odomBroadcaster(current_x_ , current_y_ , current_yaw_) ;
        last_x_ = current_x_ ;
        last_y_ = current_y_ ;
        last_yaw_ = current_yaw_ ;
        last_time_ = ros::Time::now() ;
        pubOdomRequest(0x04) ;
        r.sleep() ;
        ros::spinOnce() ;
    }
}

void RobotOdom::pubOdom(const double x ,const double y ,const double yaw)
{
    nav_msgs::Odometry odom ;
    odom.header.stamp = ros::Time::now() ;
    odom.header.frame_id = "odom" ;
    odom.pose.pose.position.x = x ;
    odom.pose.pose.position.y = y ;
    odom.pose.pose.position.z = 0.0 ;

    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw) ;

    setOdomVelocity(x - last_x_ , y - last_y_ , yaw - last_yaw_,odom) ;

    odom_pub_.publish(odom) ;
}

//get the odometry data from handware and send these data to tf tree
void RobotOdom::odomBroadcaster(const double x , const double y , const double yaw)
{
    geometry_msgs::TransformStamped odom_transform ;
    odom_transform.header.stamp = ros::Time::now() ;

    odom_transform.header.frame_id = "/odom" ;
    odom_transform.child_frame_id = "/base_link" ;

    odom_transform.transform.translation.x = x ;
    odom_transform.transform.translation.y = y ;
    odom_transform.transform.translation.z = 0.0 ;
    odom_transform.transform.rotation = tf::createQuaternionMsgFromYaw(yaw) ;
    odom_broadcaster_.sendTransform(odom_transform) ;
}

//get the odometry data from handware and send these data to navigation stack
void RobotOdom::odomCallBack(const basketball_msgs::robot_state::ConstPtr &ptr)
{
    double x = ptr->data.at(0) ;
    double y = ptr->data.at(1) ;
    double yaw = ptr->data.at(2) ;
    #ifdef DEBUG
        ROS_INFO("the id is %d\n" , ptr->id) ;
        ROS_INFO("current length is %d ,  current odom is x = %lf , y = %lf , yaw = %lf\n" , ptr->data.size(),  x , y ,yaw) ;
    #endif // DEBUG
    current_x_ = x ;
    current_y_ = y ;
    current_yaw_ = yaw ;
}

int main(int argc , char **argv)
{
    ros::init(argc , argv , "robot_odom") ;
    ros::NodeHandle node ;
    RobotOdom robot_odom(node) ;
    robot_odom.mainRun() ;
    return 0 ;
}
