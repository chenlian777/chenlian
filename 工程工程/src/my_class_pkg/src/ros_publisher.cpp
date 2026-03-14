#include<ros/ros.h>
 #include<std_msgs/String.h>
 int main(int argc, char **argv) {
     // 初始化ROS节点
     ros::init(argc, argv, "my_publisher");
     // 创建节点句柄
     ros::NodeHandle nh;
     // 定义发布者，主题my_topic，队列大小10
     ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);
     // 1秒发布一次
     ros::Rate rate(1.0);
     while (ros::ok())
     {
         std_msgs::String msg;
         msg.data = "Hello, world!";
         // 发布消息
         pub.publish(msg);
         // 休眠1秒
         rate.sleep();
     }
     return 0;
 }
