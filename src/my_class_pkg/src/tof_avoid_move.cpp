#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Range.h"

// 状态变量
bool has_obstacle   = false;   // 当前是否检测到障碍
bool is_backing     = false;   // 是否正在后退中
bool is_stop        = false;   // 是否已经完成后退并停止

ros::Time back_start_time;
ros::Publisher cmd_pub_;

// 里程计回调（本例未使用，保留为空）
void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {}

// TOF2 回调：实时判断障碍物（距离 < 0.4m 即为有障碍）
void tof2_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    // 去掉下限 0.05，只要小于 0.4 就认为有障碍
    if (msg->range < 0.4)
    {
        has_obstacle = true;
    }
    else
    {
        has_obstacle = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tof_avoid_move");
    ros::NodeHandle nh;

    ros::Subscriber tof2_sub = nh.subscribe("/us/tof2", 500, tof2_callback);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 500);

    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(200);

    const double FORWARD_SPEED  =  0.2;
    const double BACK_SPEED     = -0.05;
    const double BACK_DURATION  =  4.0;

    while (ros::ok())
    {
        // 如果已经停止，不再处理任何动作
        if (!is_stop)
        {
            // 检测到障碍且尚未开始后退 → 启动后退
            if (!is_backing && has_obstacle)
            {
                is_backing = true;
                back_start_time = ros::Time::now();
                ROS_WARN("障碍物<0.4m，开始后退4秒");
            }

            if (is_backing)
            {
                double dt = (ros::Time::now() - back_start_time).toSec();
                if (dt < BACK_DURATION)
                {
                    cmd_vel.linear.x = BACK_SPEED;   // 后退中
                }
                else
                {
                    // 后退结束，永久停止
                    is_backing = false;
                    is_stop = true;
                    cmd_vel.linear.x = 0.0;
                    ROS_INFO("后退完成，机器人永久停止");
                }
            }
            else
            {
                // 无障碍且未后退 → 正常前进
                cmd_vel.linear.x = FORWARD_SPEED;
            }
        }
        else
        {
            // 已经停止，速度保持0
            cmd_vel.linear.x = 0.0;
        }

        cmd_vel.angular.z = 0.0;
        cmd_pub_.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
