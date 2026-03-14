#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include "std_msgs/Int16MultiArray.h"

// 全局变量：保留原逻辑，无新增
bool last_bump_trigger = false;
bool is_backing = false;
bool is_stop = false;
ros::Time back_start_time;
ros::Publisher cmd_pub_;

// 里程计回调：完全保留原逻辑
void odom_callback(const nav_msgs::OdometryConstPtr&odom_msg)
{
    double odom_x = odom_msg->pose.pose.position.x;
}

// 碰撞传感器回调：完全保留原逻辑
void bump_callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    last_bump_trigger = (msg->data[0] == 1) || (msg->data[1] == 1) || (msg->data[2] == 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bump_avoid_move");
    ros::NodeHandle nh;

    // 优化1：增大订阅/发布队列大小（从10→200），杜绝碰撞/速度指令丢失
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 200, odom_callback);
    ros::Subscriber bump_sub = nh.subscribe("/robot/bump_sensor", 200, bump_callback);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 200);

    geometry_msgs::Twist cmd_vel;
    // 优化2：提升刷新频率（从20Hz→100Hz），碰撞响应快5倍，无延迟
    ros::Rate rate(100);
    // 固定参数：0.05m/s后退4秒，前进0.2m/s（按你要求）
    const double BACK_SPEED = -0.05;
    const double FORWARD_SPEED = 0.2;
    const double BACK_DURATION = 4.0;

    while (ros::ok())
    {
        if (!is_stop && !is_backing && last_bump_trigger)
        {
            is_backing = true;
            back_start_time = ros::Time::now();
            ROS_WARN("Collision Detected! Start backing up for 4s at 0.05m/s...");
        }

        if (is_backing)
        {
            ros::Duration back_elapsed = ros::Time::now() - back_start_time;
            if (back_elapsed.toSec() < BACK_DURATION)
            {
                cmd_vel.linear.x = BACK_SPEED;
            }
            else
            {
                is_backing = false;
                is_stop = true;
                cmd_vel.linear.x = 0.0;
                ROS_INFO("Back up finished! Stop moving completely...");
            }
        }
        else if (!is_stop)
        {
            cmd_vel.linear.x = FORWARD_SPEED;
        }
        else
        {
            cmd_vel.linear.x = 0.0;
        }

        // 强制初始化所有运动轴为0，杜绝底盘指令异常，仅纯前后运动
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        
        cmd_pub_.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
