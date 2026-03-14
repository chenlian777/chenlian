#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

enum State { MOVE_FORWARD, TURN_LEFT, FINISH };
State current_state = MOVE_FORWARD;

double start_x = 0.0, start_y = 0.0;
tfScalar start_yaw = 0.0;
double target_distance = 1.0; // 1米
double target_yaw = M_PI / 2; // 90度
int side_count = 0;
const int total_sides = 4;

void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
    static bool init = false;
    if (!init) {
        start_x = odom_msg->pose.pose.position.x;
        start_y = odom_msg->pose.pose.position.y;
        tf::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        tfScalar roll, pitch; // 新增临时变量
        m.getRPY(roll, pitch, start_yaw); // 用变量接收，不再传0
        init = true;
        ROS_INFO("初始位置已校准，开始执行1×1米方格！");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_move");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odom_callback);
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(20);

    while (ros::ok() && current_state != FINISH) {
        ros::spinOnce();
        nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh, ros::Duration(1.0));
        if (!odom) continue;

        double current_x = odom->pose.pose.position.x;
        double current_y = odom->pose.pose.position.y;
        tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        tfScalar current_yaw, roll, pitch; // 新增临时变量
        m.getRPY(roll, pitch, current_yaw); // 用变量接收，不再传0

        if (current_state == MOVE_FORWARD) {
            double distance = sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
            if (distance < target_distance) {
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.0;
            } else {
                cmd_vel.linear.x = 0.0;
                cmd_pub.publish(cmd_vel);
                ros::Duration(0.5).sleep();
                current_state = TURN_LEFT;
                ROS_INFO("第%d条边前进完成，准备左转90度", side_count + 1);
            }
        } else if (current_state == TURN_LEFT) {
            double yaw_diff = fabs(static_cast<double>(current_yaw) - static_cast<double>(start_yaw));
            if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
            if (yaw_diff < target_yaw) {
                cmd_vel.angular.z = 0.5;
                cmd_vel.linear.x = 0.0;
            } else {
                cmd_vel.angular.z = 0.0;
                cmd_pub.publish(cmd_vel);
                ros::Duration(0.5).sleep();
                side_count++;
                if (side_count >= total_sides) {
                    current_state = FINISH;
                    ROS_INFO("1×1米方格执行完成！");
                } else {
                    start_x = current_x;
                    start_y = current_y;
                    start_yaw = current_yaw;
                    current_state = MOVE_FORWARD;
                    ROS_INFO("左转完成，开始第%d条边", side_count + 1);
                }
            }
        }
        cmd_pub.publish(cmd_vel);
        rate.sleep();
    }
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_pub.publish(cmd_vel);
    return 0;
}
