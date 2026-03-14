#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

bool got_finish = false;
double odom_distance = 0;

// 里程计回调函数，实时获取机器人x方向位置
void odom_callback(const nav_msgs::OdometryConstPtr&odom_msg)
{
    double odom_x = odom_msg->pose.pose.position.x;
    odom_distance = odom_x;
    // 当x方向走够2米，标记为完成
    if (odom_x > 2.0) {
        if (!got_finish) {
            got_finish = true;
            ROS_INFO("Finish!!!!!!!!"); // 终端打印完成提示
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;
    // 订阅/odom话题，获取里程计数据，触发回调函数
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odom_callback);
    // 发布/cmd_vel话题，控制机器人速度
    ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist cmd_vel;

    // 循环控制，实时判断是否走够2米
    while (ros::ok())
    {
        if (got_finish) {
            // 走够2米，速度置0，停止
            cmd_vel.linear.x = 0.0;
        }
        else {
            // 没走够，继续以0.2m/s前进
            cmd_vel.linear.x = 0.2;
        }
        cmd_pub_.publish(cmd_vel);
        ros::Duration(0.05).sleep();
        ros::spinOnce(); // 触发回调函数，更新里程计数据
    }
    return 0;
}
