#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

int main(int argc, char** argv) {
  // 设置本地化环境，确保日志输出中文不乱码
  setlocale(LC_ALL, "");

  // 执行节点初始化
  ros::init(argc, argv, "imu_publisher_node");

  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建发布者对象
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  // 设置循环频率
  ros::Rate loop_rate(200);

  double t = 0.0;
  const double dt = 1.0 / 200.0;

  // 输出日志
  while (ros::ok()) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    // 模拟角速度 (rad/s)
    imu_msg.angular_velocity.x = 0.1 * std::sin(t);
    imu_msg.angular_velocity.y = 0.1 * std::cos(t);
    imu_msg.angular_velocity.z = 0.05 * std::sin(2 * t);

    // 模拟线加速度 (m/s^2)
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;  // 静止时的重力

    pub.publish(imu_msg);
    loop_rate.sleep();
  }

  return 0;
}
