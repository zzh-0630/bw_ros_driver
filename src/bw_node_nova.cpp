#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>

// Standard headers
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// Project headers
#include "bw_ros_driver/nova_parser.hpp"
#include "bw_ros_driver/serial_port.hpp"

namespace
{
double magScaleToTesla(const std::string& unit)
{
  if (unit == "gauss")
  {
    return 1e-4;
  }
  if (unit == "uT" || unit == "ut")
  {
    return 1e-6;
  }
  return 1.0;
}

void fillCovariance(double value, boost::array<double, 9>& refCov)
{
  for (size_t i = 0; i < 9; ++i)
  {
    refCov[i] = 0.0;
  }
  refCov[0] = value;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bw_node_nova");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //-------------------------------------------------------------------//
  // Parameters                                                        //
  //-------------------------------------------------------------------//
  std::string port      = "/dev/ttyUSB0";
  int baud              = 115200;

  std::string frame_id  = "imu_link";
  std::string imu_topic = "/imu/data";
  std::string mag_topic = "/imu/mag";
  std::string mag_unit  = "gauss";

  bool debug = true;

  double cov_orientation        = -1.0;
  double cov_angular_velocity   = -1.0;
  double cov_linear_acceleration = -1.0;

  pnh.param("port", port, port);
  pnh.param("baud", baud, baud);
  pnh.param("frame_id", frame_id, frame_id);
  pnh.param("imu_topic", imu_topic, imu_topic);
  pnh.param("mag_topic", mag_topic, mag_topic);
  pnh.param("mag_unit", mag_unit, mag_unit);
  pnh.param("debug", debug, debug);

  pnh.param("cov_orientation", cov_orientation, cov_orientation);
  pnh.param("cov_angular_velocity", cov_angular_velocity, cov_angular_velocity);
  pnh.param("cov_linear_acceleration", cov_linear_acceleration, cov_linear_acceleration);

  //-------------------------------------------------------------------//
  // Open serial port                                                  //
  //-------------------------------------------------------------------//
  bw::SerialPort serial(port, baud);

  if (!serial.openSerial())
  {
    ROS_FATAL("Failed to open serial port %s", port.c_str());
    return 1;
  }

  ROS_INFO("Opened %s @ %d (Nova/F3), publishing IMU=%s MAG=%s",
           port.c_str(), baud, imu_topic.c_str(), mag_topic.c_str());

  //-------------------------------------------------------------------//
  // Publishers                                                        //
  //-------------------------------------------------------------------//
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 200);
  ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>(mag_topic, 200);

  //-------------------------------------------------------------------//
  // Parser                                                            //
  //-------------------------------------------------------------------//
  bw::NovaParser parser;

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  constexpr double kGToMps2  = 9.80665;

  const double mag_scale = magScaleToTesla(mag_unit);

  //-------------------------------------------------------------------//
  // Debug stats                                                       //
  //-------------------------------------------------------------------//
  ros::Time t_window = ros::Time::now();
  uint64_t last_ok = 0;

  ros::Rate idle(500.0);

  //-------------------------------------------------------------------//
  // Main loop                                                         //
  //-------------------------------------------------------------------//
  while (ros::ok())
  {
    uint8_t buffer[1024];
    const ssize_t n = serial.readSome(buffer, sizeof(buffer));

    if (n < 0 && errno != EAGAIN)
    {
      if (debug)
      {
        ROS_WARN_THROTTLE(1.0, "F3 serial read error: errno=%d (%s)", errno, std::strerror(errno));
      }
    }

    if (n > 0)
    {
      std::vector<bw::F3Sample> samples;
      samples.reserve(8);

      const size_t m = parser.feed(buffer, static_cast<size_t>(n), samples);

      for (size_t i = 0; i < m; ++i)
      {
        const bw::F3Sample& s = samples[i];

        if (!s.valid)
        {
          continue;
        }

        //-----------------------------------------------------------------//
        // Build IMU message                                                 //
        //-----------------------------------------------------------------//
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;

        fillCovariance(cov_orientation, msg.orientation_covariance);
        fillCovariance(cov_angular_velocity, msg.angular_velocity_covariance);
        fillCovariance(cov_linear_acceleration, msg.linear_acceleration_covariance);

        const double roll  = static_cast<double>(s.roll_deg) * kDegToRad;
        const double pitch = static_cast<double>(s.pitch_deg) * kDegToRad;
        const double yaw   = static_cast<double>(s.yaw_deg) * kDegToRad;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        msg.orientation.w = q.getW();
        msg.orientation.x = q.getX();
        msg.orientation.y = q.getY();
        msg.orientation.z = q.getZ();

        msg.angular_velocity.x = static_cast<double>(s.gyro_x_dps) * kDegToRad;
        msg.angular_velocity.y = static_cast<double>(s.gyro_y_dps) * kDegToRad;
        msg.angular_velocity.z = static_cast<double>(s.gyro_z_dps) * kDegToRad;

        msg.linear_acceleration.x = static_cast<double>(s.acc_x_g) * kGToMps2;
        msg.linear_acceleration.y = static_cast<double>(s.acc_y_g) * kGToMps2;
        msg.linear_acceleration.z = static_cast<double>(s.acc_z_g) * kGToMps2;

        imu_pub.publish(msg);

        //-----------------------------------------------------------------//
        // Magnetic field                                                   //
        //-----------------------------------------------------------------//
        sensor_msgs::MagneticField mmsg;
        mmsg.header = msg.header;

        mmsg.magnetic_field.x = static_cast<double>(s.mag_x) * mag_scale;
        mmsg.magnetic_field.y = static_cast<double>(s.mag_y) * mag_scale;
        mmsg.magnetic_field.z = static_cast<double>(s.mag_z) * mag_scale;

        mag_pub.publish(mmsg);

        //-----------------------------------------------------------------//
        // Debug output                                                     //
        //-----------------------------------------------------------------//
        if (debug)
        {
          const ros::Time now = ros::Time::now();
          if ((now - t_window).toSec() >= 1.0)
          {
            const uint64_t ok_now = parser.ok();
            const double hz = (ok_now >= last_ok) ? static_cast<double>(ok_now - last_ok) : 0.0;

            ROS_INFO(
              "F3 RX ~ %.1f Hz (ok=%lu bad=%lu crc_bad=%lu) "
              "Euler[deg]=[P=%+.2f R=%+.2f Y=%+.2f] "
              "G[dps]=[%+.2f %+.2f %+.2f] "
              "A[g]=[%+.3f %+.3f %+.3f] "
              "M=[%+.3f %+.3f %+.3f] "
              "Temp[C]=%.2f time_us=%u",
              hz,
              static_cast<unsigned long>(parser.ok()),
              static_cast<unsigned long>(parser.bad()),
              static_cast<unsigned long>(parser.crcBad()),
              s.pitch_deg, s.roll_deg, s.yaw_deg,
              s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps,
              s.acc_x_g, s.acc_y_g, s.acc_z_g,
              s.mag_x, s.mag_y, s.mag_z,
              s.temperature_deg_c,
              static_cast<unsigned>(s.time_us));

            last_ok = ok_now;
            t_window = now;
          }
        }
      }
    }
    else
    {
      if (debug && parser.ok() == 0)
      {
        ROS_INFO_THROTTLE(2.0, "F3: waiting for frames on %s ...", port.c_str());
      }
    }

    ros::spinOnce();
    idle.sleep();
  }

  serial.closeSerial();
  return 0;
}