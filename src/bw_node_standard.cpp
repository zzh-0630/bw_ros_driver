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
#include "bw_ros_driver/serial_port.hpp"
#include "bw_ros_driver/standard_parser.hpp"

namespace
{
double magScaleToTesla(const std::string& unit)
{
  // Default assumes already in tesla.
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
  ros::init(argc, argv, "bw_node_standard");

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

  ROS_INFO("Opened %s @ %d, publishing IMU=%s MAG=%s", port.c_str(), baud, imu_topic.c_str(), mag_topic.c_str());

  //-------------------------------------------------------------------//
  // Publishers                                                        //
  //-------------------------------------------------------------------//
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 200);
  ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>(mag_topic, 200);

  //-------------------------------------------------------------------//
  // Parser                                                            //
  //-------------------------------------------------------------------//
  bw::StandardParser parser;

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  constexpr double kGToMps2  = 9.80665;

  const double mag_scale = magScaleToTesla(mag_unit);

  //-------------------------------------------------------------------//
  // Debug stats                                                       //
  //-------------------------------------------------------------------//
  ros::Time t_window = ros::Time::now();
  uint64_t last_ok = 0;

  bw::DataSample last_sample;
  bool last_sample_valid = false;

  ros::Rate idle(200.0);

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
        ROS_WARN_THROTTLE(1.0, "serial read error: errno=%d (%s)", errno, std::strerror(errno));
      }
    }

    if (n > 0)
    {
      std::vector<bw::DataSample> samples;
      samples.reserve(8);

      const size_t m = parser.feed(buffer, static_cast<size_t>(n), samples);

      for (size_t i = 0; i < m; ++i)
      {
        const bw::DataSample& s = samples[i];

        //-----------------------------------------------------------------//
        // Build IMU message                                                 //
        //-----------------------------------------------------------------//
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id;

        fillCovariance(cov_orientation, imu_msg.orientation_covariance);
        fillCovariance(cov_angular_velocity, imu_msg.angular_velocity_covariance);
        fillCovariance(cov_linear_acceleration, imu_msg.linear_acceleration_covariance);

        // Orientation
        if (s.has_quat)
        {
          imu_msg.orientation.w = s.q0;
          imu_msg.orientation.x = s.q1;
          imu_msg.orientation.y = s.q2;
          imu_msg.orientation.z = s.q3;
        }
        else if (s.has_euler)
        {
          const double roll  = s.R * kDegToRad;
          const double pitch = s.P * kDegToRad;
          const double yaw   = s.Y * kDegToRad;

          tf2::Quaternion q;
          q.setRPY(roll, pitch, yaw);
          q.normalize();

          imu_msg.orientation.w = q.getW();
          imu_msg.orientation.x = q.getX();
          imu_msg.orientation.y = q.getY();
          imu_msg.orientation.z = q.getZ();
        }
        else
        {
          imu_msg.orientation_covariance[0] = -1.0;
        }

        // Angular velocity [deg/s] -> [rad/s]
        if (s.has_gyro)
        {
          imu_msg.angular_velocity.x = s.gx_dps * kDegToRad;
          imu_msg.angular_velocity.y = s.gy_dps * kDegToRad;
          imu_msg.angular_velocity.z = s.gz_dps * kDegToRad;
        }
        else
        {
          imu_msg.angular_velocity_covariance[0] = -1.0;
        }

        // Linear acceleration [g] -> [m/s^2]
        if (s.has_acc)
        {
          imu_msg.linear_acceleration.x = s.ax_g * kGToMps2;
          imu_msg.linear_acceleration.y = s.ay_g * kGToMps2;
          imu_msg.linear_acceleration.z = s.az_g * kGToMps2;
        }
        else
        {
          imu_msg.linear_acceleration_covariance[0] = -1.0;
        }

        imu_pub.publish(imu_msg);

        //-----------------------------------------------------------------//
        // Magnetic field (optional)                                        //
        //-----------------------------------------------------------------//
        if (s.has_mag)
        {
          sensor_msgs::MagneticField mag_msg;
          mag_msg.header = imu_msg.header;

          mag_msg.magnetic_field.x = s.mx * mag_scale;
          mag_msg.magnetic_field.y = s.my * mag_scale;
          mag_msg.magnetic_field.z = s.mz * mag_scale;

          mag_pub.publish(mag_msg);
        }

        last_sample = s;
        last_sample_valid = true;
      }
    }

    //-------------------------------------------------------------------//
    // Debug output                                                      //
    //-------------------------------------------------------------------//
    if (debug)
    {
      const ros::Time now = ros::Time::now();
      if ((now - t_window).toSec() >= 1.0)
      {
        const uint64_t ok_now = parser.ok();
        const uint64_t bad_now = parser.bad();
        const double hz = (ok_now >= last_ok) ? static_cast<double>(ok_now - last_ok) : 0.0;

        if (last_sample_valid)
        {
          ROS_INFO(
            "STD RX ~ %.1f Hz (ok=%lu bad=%lu) "
            "Euler[deg]=[P=%+.2f R=%+.2f Y=%+.2f] "
            "G[dps]=[%+.2f %+.2f %+.2f] "
            "A[g]=[%+.3f %+.3f %+.3f] "
            "M=[%+.5f %+.5f %+.5f] "
            "q=[%+.6f %+.6f %+.6f %+.6f]",
            hz,
            static_cast<unsigned long>(ok_now),
            static_cast<unsigned long>(bad_now),
            last_sample.P, last_sample.R, last_sample.Y,
            last_sample.gx_dps, last_sample.gy_dps, last_sample.gz_dps,
            last_sample.ax_g, last_sample.ay_g, last_sample.az_g,
            last_sample.mx, last_sample.my, last_sample.mz,
            last_sample.q0, last_sample.q1, last_sample.q2, last_sample.q3);
        }
        else
        {
          ROS_INFO("STD: waiting for frames on %s ... (ok=%lu bad=%lu)",
                   port.c_str(),
                   static_cast<unsigned long>(ok_now),
                   static_cast<unsigned long>(bad_now));
        }

        last_ok = ok_now;
        t_window = now;
      }
    }

    ros::spinOnce();
    idle.sleep();
  }

  serial.closeSerial();
  return 0;
}