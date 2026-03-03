#include <linux/can.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Standard headers
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

// Project headers
#include "bw_ros_driver/can_imu_aggregator.hpp"
#include "bw_ros_driver/can_port.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bw_node_can");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //-------------------------------------------------------------------//
  // Parameters                                                        //
  //-------------------------------------------------------------------//
  std::string can_iface = "can0";
  int can_id = 0x585;
  int rx_timeout_ms = 50;

  std::string imu_topic = "/imu/data";
  std::string frame_id = "imu_link";
  bool debug = false;

  bw::CanImuAggregatorConfig cfg;
  cfg.max_age_ms = 100;
  cfg.prefer_quat = true;
  cfg.allow_angle_fallback = true;
  cfg.require_acc = true;
  cfg.require_orientation = true;

  cfg.cov_orientation = -1.0;
  cfg.cov_angular_velocity = -1.0;
  cfg.cov_linear_acceleration = -1.0;

  pnh.param("can_iface", can_iface, can_iface);
  pnh.param("can_id", can_id, can_id);
  pnh.param("rx_timeout_ms", rx_timeout_ms, rx_timeout_ms);

  pnh.param("imu_topic", imu_topic, imu_topic);
  pnh.param("frame_id", frame_id, frame_id);
  pnh.param("debug", debug, debug);

  pnh.param("max_age_ms", cfg.max_age_ms, cfg.max_age_ms);
  pnh.param("prefer_quat", cfg.prefer_quat, cfg.prefer_quat);
  pnh.param("allow_angle_fallback", cfg.allow_angle_fallback, cfg.allow_angle_fallback);
  pnh.param("require_acc", cfg.require_acc, cfg.require_acc);
  pnh.param("require_orientation", cfg.require_orientation, cfg.require_orientation);

  pnh.param("cov_orientation", cfg.cov_orientation, cfg.cov_orientation);
  pnh.param("cov_angular_velocity", cfg.cov_angular_velocity, cfg.cov_angular_velocity);
  pnh.param("cov_linear_acceleration", cfg.cov_linear_acceleration, cfg.cov_linear_acceleration);

  cfg.frame_id = frame_id;

  //-------------------------------------------------------------------//
  // Publisher                                                         //
  //-------------------------------------------------------------------//
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 200);

  //-------------------------------------------------------------------//
  // CAN init                                                          //
  //-------------------------------------------------------------------//
  bw::CanPort can(can_iface);

  const std::vector<uint32_t> filters = { static_cast<uint32_t>(can_id) };

  if (!can.openCan(filters))
  {
    ROS_FATAL("Failed to open SocketCAN iface=%s: errno=%d (%s)",
              can_iface.c_str(),
              errno,
              std::strerror(errno));
    return 1;
  }

  ROS_INFO("bw_node_can started. iface=%s can_id=0x%X imu_topic=%s frame_id=%s",
           can_iface.c_str(),
           can_id,
           imu_topic.c_str(),
           frame_id.c_str());

  //-------------------------------------------------------------------//
  // Aggregator                                                        //
  //-------------------------------------------------------------------//
  bw::CanImuAggregator agg(cfg);

  //-------------------------------------------------------------------//
  // Debug counters                                                    //
  //-------------------------------------------------------------------//
  uint64_t cnt_rx = 0;
  uint64_t cnt_filter = 0;
  uint64_t cnt_pub = 0;
  uint64_t cnt_drop = 0;
  uint64_t cnt_err = 0;

  ros::Time t_stat = ros::Time::now();

  //-------------------------------------------------------------------//
  // Main loop                                                         //
  //-------------------------------------------------------------------//
  while (ros::ok())
  {
    struct can_frame fr;
    const int r = can.readFrame(fr, rx_timeout_ms);

    if (r < 0)
    {
      ++cnt_err;
      ROS_WARN_THROTTLE(1.0, "CAN read error: errno=%d (%s)", errno, std::strerror(errno));
      ros::spinOnce();
      continue;
    }

    if (r == 0)
    {
      ros::spinOnce();
      continue;
    }

    ++cnt_rx;

    // Accept only standard frames with DLC=8.
    if ((fr.can_id & CAN_EFF_FLAG) != 0)
    {
      ++cnt_filter;
      continue;
    }

    const uint32_t id = fr.can_id & CAN_SFF_MASK;
    if (static_cast<int>(id) != can_id || fr.can_dlc != 8)
    {
      ++cnt_filter;
      continue;
    }

    const ros::Time stamp = ros::Time::now();

    sensor_msgs::Imu msg;
    if (agg.ingestAndMaybeBuildImu(fr.data, stamp, msg))
    {
      imu_pub.publish(msg);
      ++cnt_pub;
    }
    else
    {
      // Expected: most frames are intermediate and do not publish.
      ++cnt_drop;
    }

    if (debug)
    {
      const ros::Time now = ros::Time::now();
      if ((now - t_stat).toSec() >= 1.0)
      {
        ROS_INFO("stats: rx=%lu pub=%lu filter=%lu err=%lu",
                 static_cast<unsigned long>(cnt_rx),
                 static_cast<unsigned long>(cnt_pub),
                 static_cast<unsigned long>(cnt_filter),
                 static_cast<unsigned long>(cnt_err));
        t_stat = now;
      }
    }

    ros::spinOnce();
  }

  can.closeCan();
  return 0;
}
