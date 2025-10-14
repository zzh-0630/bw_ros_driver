#include <fcntl.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <string>
#include <vector>

#include "bw_ros_driver/bcd_utils.hpp"

namespace {
int open_serial(const std::string& port, int baud) {
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("open");
    return -1;
  }
  termios tio{};
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  speed_t sp = B9600;
  switch (baud) {
    case 115200:
      sp = B115200;
      break;
    case 57600:
      sp = B57600;
      break;
    case 38400:
      sp = B38400;
      break;
    case 19200:
      sp = B19200;
      break;
    case 9600:
    default:
      sp = B9600;
      break;
  }
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // 8N1, 关闭硬件流控
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);       // 关闭软件流控
  tio.c_cc[VMIN] = 0;                           // 非阻塞
  tio.c_cc[VTIME] = 2;                          // 200ms 读超时
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    perror("tcsetattr");
    close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bw_ah_auto_imu_node");
  ros::NodeHandle nh;        // 全局命名空间（便于发布到全局话题）
  ros::NodeHandle pnh("~");  // 私有参数

  std::string port = "/dev/ttyUSB0", frame_id = "imu_link", topic = "/imu/data";
  std::string mag_unit = "gauss";  // gauss | uT | tesla
  int baud = 115200;
  bool debug = true;
  double ori_cov = -1.0, gyr_cov = -1.0, acc_cov = -1.0;  // -1 表示未知
  pnh.param("port", port, port);
  pnh.param("baud", baud, baud);
  pnh.param("frame_id", frame_id, frame_id);
  pnh.param("topic", topic, topic);
  pnh.param("mag_unit", mag_unit, mag_unit);
  pnh.param("debug", debug, debug);
  pnh.param("cov_orientation", ori_cov, ori_cov);
  pnh.param("cov_angular_velocity", gyr_cov, gyr_cov);
  pnh.param("cov_linear_acceleration", acc_cov, acc_cov);

  int fd = open_serial(port, baud);
  if (fd < 0) {
    ROS_FATAL("Open %s failed", port.c_str());
    return 1;
  }
  ROS_INFO("Opened %s @ %d, publishing %s", port.c_str(), baud, topic.c_str());

  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(topic, 200);
  ros::Publisher mag_pub =
      nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 200);

  std::vector<uint8_t> buf;
  buf.reserve(256);
  enum { WAIT_77, READ_LEN, READ_PAYLOAD } st = WAIT_77;
  uint8_t need = 0;
  uint64_t ok_cnt = 0, bad_cnt = 0;
  ros::Time t0 = ros::Time::now();

  const double DEG2RAD = M_PI / 180.0;
  const double G2MS2 = 9.80665;

  ros::Rate idle(50);  
  while (ros::ok()) {
    uint8_t tmp[512];
    ssize_t n = read(fd, tmp, sizeof(tmp));
    if (n < 0 && errno != EAGAIN) {
      if (debug) ROS_WARN_THROTTLE(1.0, "read() error: %d", errno);
    }
    for (ssize_t i = 0; i < n; ++i) {
      uint8_t b = tmp[i];
      if (st == WAIT_77) {
        if (b == 0x77) {
          buf.clear();
          buf.push_back(b);
          st = READ_LEN;
        }
      } else if (st == READ_LEN) {
        buf.push_back(b);
        need = b;
        st = READ_PAYLOAD;
      } else {
        buf.push_back(b);
        if (buf.size() == static_cast<size_t>(1 + need)) {
          // 完整帧
          uint8_t cs = sum8(&buf[1], need - 1);
          uint8_t chk = buf.back();
          if (cs == chk) {
            ++ok_cnt;
            uint8_t cmd = buf[3];
            const uint8_t* d = &buf[4];  // 数据域起始
            size_t left =
                need -
                4;  // 数据域字节数 = need - 4 （LEN, ADDR, CMD, DATA..., CHK）

            sensor_msgs::Imu msg;
            bool publish = false;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id;
            // 默认协方差
            for (int k = 0; k < 9; ++k) {
              msg.orientation_covariance[k] = 0;
              msg.angular_velocity_covariance[k] = 0;
              msg.linear_acceleration_covariance[k] = 0;
            }
            msg.orientation_covariance[0] = ori_cov;
            msg.angular_velocity_covariance[0] = gyr_cov;
            msg.linear_acceleration_covariance[0] = acc_cov;

            if (cmd == 0x59) {
              // 新版 0x59：数据域固定 52 字节（角度→加速度→角速度→磁场→四元数）
              if (left >= 52) {
                // 固定偏移解析
                double P = bcd3_angle_or_dps(d[0], d[1], d[2]);
                double R = bcd3_angle_or_dps(d[3], d[4], d[5]);
                double Y = bcd3_angle_or_dps(d[6], d[7], d[8]);

                double Ax_g = bcd3_acc_g(d[9], d[10], d[11]);
                double Ay_g = bcd3_acc_g(d[12], d[13], d[14]);
                double Az_g = bcd3_acc_g(d[15], d[16], d[17]);

                double Gx_dps = bcd3_angle_or_dps(d[18], d[19], d[20]);
                double Gy_dps = bcd3_angle_or_dps(d[21], d[22], d[23]);
                double Gz_dps = bcd3_angle_or_dps(d[24], d[25], d[26]);

                double Mx = bcd3_mag_frac5(d[27], d[28], d[29]);
                double My = bcd3_mag_frac5(d[30], d[31], d[32]);
                double Mz = bcd3_mag_frac5(d[33], d[34], d[35]);

                double q0 = bcd4_q_to_double(d[36], d[37], d[38], d[39]);
                double q1 = bcd4_q_to_double(d[40], d[41], d[42], d[43]);
                double q2 = bcd4_q_to_double(d[44], d[45], d[46], d[47]);
                double q3 = bcd4_q_to_double(d[48], d[49], d[50], d[51]);

                // 填充 Imu
                msg.orientation.w = q0;
                msg.orientation.x = q1;
                msg.orientation.y = q2;
                msg.orientation.z = q3;
                msg.angular_velocity.x = Gx_dps * DEG2RAD;
                msg.angular_velocity.y = Gy_dps * DEG2RAD;
                msg.angular_velocity.z = Gz_dps * DEG2RAD;
                msg.linear_acceleration.x = Ax_g * G2MS2;
                msg.linear_acceleration.y = Ay_g * G2MS2;
                msg.linear_acceleration.z = Az_g * G2MS2;

                // 发布磁场（单位转 Tesla）
                double scale = 1.0;  // 默认 tesla
                if (mag_unit == "gauss")
                  scale = 1e-4;  // 1 G = 1e-4 T
                else if (mag_unit == "uT" || mag_unit == "ut")
                  scale = 1e-6;  // 1 µT = 1e-6 T
                sensor_msgs::MagneticField m;
                m.header = msg.header;
                m.magnetic_field.x = Mx * scale;
                m.magnetic_field.y = My * scale;
                m.magnetic_field.z = Mz * scale;
                m.magnetic_field_covariance[0] = 0;  // 若未知可保持 0 或 -1
                mag_pub.publish(m);

                publish = true;

                if (debug) {
                  static double last = 0;
                  double now = ros::Time::now().toSec();
                  if (now - last > 1.0) {
                    double dt = (ros::Time::now() - t0).toSec();
                    double hz = (dt > 0) ? ok_cnt / dt : 0;
                    ROS_INFO(
                        "AUTO BCD52 %.1f Hz  P=%+.2f R=%+.2f Y=%+.2f  "
                        "G[deg/s]=[%+.2f %+.2f %+.2f]  A[g]=[%+.2f %+.2f "
                        "%+.2f]  M=[%+.5f %+.5f %+.5f]  q=[%+.6f %+.6f %+.6f "
                        "%+.6f]  (bad=%lu)",
                        hz, P, R, Y, Gx_dps, Gy_dps, Gz_dps, Ax_g, Ay_g, Az_g,
                        Mx, My, Mz, q0, q1, q2, q3, (unsigned long)bad_cnt);
                    last = now;
                  }
                }
              } else {
                // 兼容旧设备/短数据域：按旧版顺序尽量解析
                size_t off = 0;
                bool hasEuler = false, hasAcc = false, hasGyro = false,
                     hasMag = false, hasQuat = false;
                double P = 0, R = 0, Y = 0, Ax = 0, Ay = 0, Az = 0, Gx = 0,
                       Gy = 0, Gz = 0, Mx = 0, My = 0, Mz = 0, q0 = 0, q1 = 0,
                       q2 = 0, q3 = 0;
                auto take3_angle = [&](double& v) {
                  if (off + 3 <= left) {
                    v = bcd3_angle_or_dps(d[off], d[off + 1], d[off + 2]);
                    off += 3;
                    return true;
                  }
                  return false;
                };
                auto take3_acc = [&](double& v) {
                  if (off + 3 <= left) {
                    v = bcd3_acc_g(d[off], d[off + 1], d[off + 2]);
                    off += 3;
                    return true;
                  }
                  return false;
                };
                auto take3_mag = [&](double& v) {
                  if (off + 3 <= left) {
                    v = bcd3_mag_frac5(d[off], d[off + 1], d[off + 2]);
                    off += 3;
                    return true;
                  }
                  return false;
                };
                auto take4q = [&](double& v) {
                  if (off + 4 <= left) {
                    v = bcd4_q_to_double(d[off], d[off + 1], d[off + 2],
                                         d[off + 3]);
                    off += 4;
                    return true;
                  }
                  return false;
                };
                hasEuler = take3_angle(P) && take3_angle(R) && take3_angle(Y);
                hasAcc = take3_acc(Ax) && take3_acc(Ay) && take3_acc(Az);
                hasGyro = take3_angle(Gx) && take3_angle(Gy) && take3_angle(Gz);
                hasMag = take3_mag(Mx) && take3_mag(My) && take3_mag(Mz);
                hasQuat = take4q(q0) && take4q(q1) && take4q(q2) && take4q(q3);
                if (hasQuat) {
                  msg.orientation.w = q0;
                  msg.orientation.x = q1;
                  msg.orientation.y = q2;
                  msg.orientation.z = q3;
                } else if (hasEuler) {
                  tf2::Quaternion q;
                  q.setRPY(R * DEG2RAD, P * DEG2RAD, Y * DEG2RAD);
                  msg.orientation.x = q.x();
                  msg.orientation.y = q.y();
                  msg.orientation.z = q.z();
                  msg.orientation.w = q.w();
                }
                if (hasGyro) {
                  msg.angular_velocity.x = Gx * DEG2RAD;
                  msg.angular_velocity.y = Gy * DEG2RAD;
                  msg.angular_velocity.z = Gz * DEG2RAD;
                } else {
                  msg.angular_velocity_covariance[0] = -1;
                }
                if (hasAcc) {
                  msg.linear_acceleration.x = Ax * G2MS2;
                  msg.linear_acceleration.y = Ay * G2MS2;
                  msg.linear_acceleration.z = Az * G2MS2;
                } else {
                  msg.linear_acceleration_covariance[0] = -1;
                }
                if (hasMag) {
                  double scale =
                      (mag_unit == "gauss"
                           ? 1e-4
                           : (mag_unit == "uT" || mag_unit == "ut" ? 1e-6
                                                                   : 1.0));
                  sensor_msgs::MagneticField m;
                  m.header = msg.header;
                  m.magnetic_field.x = Mx * scale;
                  m.magnetic_field.y = My * scale;
                  m.magnetic_field.z = Mz * scale;
                  mag_pub.publish(m);
                }
                publish = hasEuler || hasQuat || hasGyro || hasAcc || hasMag;
              }
            } else if (cmd == 0x84 && left >= 9) {
              // 三轴角（仅姿态，BCD）
              double P = bcd3_angle_or_dps(d[0], d[1], d[2]);
              double R = bcd3_angle_or_dps(d[3], d[4], d[5]);
              double Y = bcd3_angle_or_dps(d[6], d[7], d[8]);
              tf2::Quaternion q;
              q.setRPY(R * DEG2RAD, P * DEG2RAD, Y * DEG2RAD);
              msg.orientation.x = q.x();
              msg.orientation.y = q.y();
              msg.orientation.z = q.z();
              msg.orientation.w = q.w();
              msg.angular_velocity_covariance[0] = -1;
              msg.linear_acceleration_covariance[0] = -1;
              publish = true;
              if (debug) {
                static double last = 0;
                double now = ros::Time::now().toSec();
                if (now - last > 1.0) {
                  double dt = (ros::Time::now() - t0).toSec();
                  double hz = (dt > 0) ? ok_cnt / dt : 0;
                  ROS_INFO(
                      "EULER BCD %.1f Hz  P=%+.2f R=%+.2f Y=%+.2f  (bad=%lu)",
                      hz, P, R, Y, (unsigned long)bad_cnt);
                  last = now;
                }
              }
            }

            if (publish) pub.publish(msg);
          } else {
            ++bad_cnt;
            if (debug)
              ROS_WARN_THROTTLE(1.0, "Checksum BAD (bad=%lu)",
                                (unsigned long)bad_cnt);
          }
          st = WAIT_77;  // 校验成功/失败都重新寻帧
        }
      }
    }
    if (debug && ok_cnt == 0) {
      ROS_INFO_THROTTLE(2.0, "Waiting for frames on %s ...", port.c_str());
    }
    ros::spinOnce();
    idle.sleep();
  }

  close(fd);
  return 0;
}