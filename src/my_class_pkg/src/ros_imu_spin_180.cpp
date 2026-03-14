#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// IMU 数据与解析
sensor_msgs::Imu current_imu_data;
bool imu_data_received = false;
double init_yaw = 0.0;
double current_yaw = 0.0;
bool yaw_init = false;

// 180° 自旋控制参数
const double TARGET_RAD = M_PI;       // 180° 对应弧度
const double ROTATE_SPEED = 1.0;      // 自旋速度（拉满，保证能转）
const double ANGLE_TOL = 0.03;        // 角度误差容忍
bool is_rotating = false;
bool rotate_complete = false;

ros::Publisher cmd_vel_pub;

// 手动四元数转航向角 yaw（不依赖 tf 库，不用改 CMake）
double quat_to_yaw(double qx, double qy, double qz, double qw)
{
    double yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    return atan2(sin(yaw), cos(yaw)); // 归一化到 [-π, π]
}

// IMU 回调：接收数据 + 解析四元数
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    current_imu_data = *imu_msg;
    imu_data_received = true;

    // 解析四元数为航向角
    current_yaw = quat_to_yaw(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w
    );

    // 首次收到数据时初始化基准航向角
    if (!yaw_init)
    {
        init_yaw = current_yaw;
        yaw_init = true;
        ROS_INFO("IMU 初始化完成，初始航向角: %.2f°", init_yaw * 180 / M_PI);
    }
}

// 计算相对初始角度的旋转量
double calc_relative_angle()
{
    double rel_angle = current_yaw - init_yaw;
    rel_angle = atan2(sin(rel_angle), cos(rel_angle));
    return fabs(rel_angle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_spin_180_node");
    ros::NodeHandle nh;

    // 订阅 IMU、发布速度指令
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 100, imu_callback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::Rate rate(200); // 高频发布，保证指令不中断
    geometry_msgs::Twist cmd_vel;

    // 等待 IMU 初始化（不阻塞输入，Ctrl+C 可正常退出）
    ROS_INFO("等待 IMU 数据...");
    while (ros::ok() && !yaw_init)
    {
        ros::spinOnce();
        rate.sleep();
    }
    is_rotating = true;
    ROS_INFO("开始自旋 180°");

    // 主循环：控制自旋 + 持续发指令
    while (ros::ok())
    {
        if (is_rotating && !rotate_complete)
        {
            double rotated_angle = calc_relative_angle();
            ROS_INFO_THROTTLE(0.5, "已旋转: %.2f° / 180°", rotated_angle * 180 / M_PI);

            // 未到 180°：发旋转指令
            if (rotated_angle < TARGET_RAD - ANGLE_TOL)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = ROTATE_SPEED;
            }
            // 已到 180°：停止
            else
            {
                cmd_vel.angular.z = 0.0;
                rotate_complete = true;
                is_rotating = false;
                ROS_INFO("自旋 180° 完成！");
            }
        }
        // 自旋完成后持续发停止指令
        else if (rotate_complete)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }

        // 每轮循环都发指令，保证底盘稳定接收
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
