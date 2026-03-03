/*!
 * \file         can_imu_aggregator.hpp
 * \author       BWSensing
 * \date         2026-02
 * \brief        SocketCAN IMU aggregation helper.
 *
 * The firmware may transmit one "logical IMU sample" as multiple CAN frames.
 * This helper aggregates a sequence of frames into one `sensor_msgs::Imu`.
 *
 * Current aggregation policy (sequence state machine):
 *   ANGLE -> ACC -> GYRO -> QUAT(0/1) -> QUAT(2/3) -> PUBLISH
 *
 * Publication is triggered on the last quaternion frame (Q2/Q3).
 *
 * Copyright (c) 2026 BWSensing
 * Distributed under the MIT License. See LICENSE for more information.
 */

#ifndef INCLUDE_BW_ROS_DRIVER_CAN_IMU_AGGREGATOR_HPP
#define INCLUDE_BW_ROS_DRIVER_CAN_IMU_AGGREGATOR_HPP

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

// Standard headers
#include <cmath>
#include <cstdint>
#include <string>

// Project headers
#include "bw_ros_driver/can_decode_utils.hpp"

namespace bw
{
//---------------------------------------------------------------------//
//- Configuration                                                      -//
//---------------------------------------------------------------------//
struct CanImuAggregatorConfig
{
  int max_age_ms = 100;              //!< Max allowed time window for one group.
  bool prefer_quat = true;           //!< Prefer quaternion if available.
  bool allow_angle_fallback = true;  //!< Use Euler angles if quaternion is missing.

  bool require_acc = true;           //!< Require acceleration for publish.
  bool require_orientation = true;   //!< Require orientation (quat or angle) for publish.

  std::string frame_id = "imu_link";

  double cov_orientation = -1.0;
  double cov_angular_velocity = -1.0;
  double cov_linear_acceleration = -1.0;
};

//---------------------------------------------------------------------//
//- Aggregator                                                         -//
//---------------------------------------------------------------------//
class CanImuAggregator
{
public:
  explicit CanImuAggregator(const CanImuAggregatorConfig& cfg) : cfg_(cfg)
  {
  }

  /*!
   * \brief Ingest one CAN payload (8 bytes) and possibly publish one IMU message.
   *
   * \return true if the current frame completes a valid group and \p refOutMsg is filled.
   */
  bool ingestAndMaybeBuildImu(const uint8_t d[8], const ros::Time& stamp, sensor_msgs::Imu& refOutMsg)
  {
    // Time-out policy: abort a partially collected group when it exceeds max window.
    if (state_ != SeqState::ExpectAngle && !t_cycle_start_.isZero())
    {
      if ((stamp - t_cycle_start_).toSec() > maxAgeSec())
      {
        resetToExpectAngle();
      }
    }

    const CanFrameKind kind = detectKindMux0x585(d);

    //-----------------------------------------------------------------//
    // Group synchronization: only an ANGLE frame can start a new group. //
    //-----------------------------------------------------------------//
    if (kind == CanFrameKind::Angle)
    {
      AngleData angle;
      if (!decodeAngleFrame(d, angle))
      {
        resetToExpectAngle();
        return false;
      }

      startNewCycle(stamp);
      ang_ = angle;
      has_angle_ = true;
      t_angle_ = stamp;

      state_ = SeqState::ExpectAcc;
      return false;
    }

    if (state_ == SeqState::ExpectAngle)
    {
      if (kind == CanFrameKind::Acc)
      {
        AccData acc;
        if (!decodeAccFrame(d, acc))
        {
          resetToExpectAngle();
          return false;
        }

        startNewCycle(stamp);
        acc_ = acc;
        has_acc_ = true;
        t_acc_ = stamp;

        state_ = SeqState::ExpectGyro;
        return false;
      }

      // 仍然允许 Angle 作为起始（原逻辑不动）
      return false;
    }

    //-----------------------------------------------------------------//
    // In-group state machine                                           //
    //-----------------------------------------------------------------//
    switch (state_)
    {
      case SeqState::ExpectAcc:
      {
        if (kind != CanFrameKind::Acc)
        {
          resetToExpectAngle();
          return false;
        }

        AccData acc;
        if (!decodeAccFrame(d, acc))
        {
          resetToExpectAngle();
          return false;
        }

        acc_ = acc;
        has_acc_ = true;
        t_acc_ = stamp;
        state_ = SeqState::ExpectGyro;

        return false;
      }

      case SeqState::ExpectGyro:
      {
        if (kind != CanFrameKind::Gyro)
        {
          resetToExpectAngle();
          return false;
        }

        GyroData gyro;
        if (!decodeGyroFrame(d, gyro))
        {
          resetToExpectAngle();
          return false;
        }

        gyro_ = gyro;
        has_gyro_ = true;
        t_gyro_ = stamp;
        state_ = SeqState::ExpectQ01;

        return false;
      }

      case SeqState::ExpectQ01:
      {
        if (kind != CanFrameKind::Quat2Comp)
        {
          resetToExpectAngle();
          return false;
        }

        QuatComp a;
        QuatComp b;
        if (!decodeQuatFrame2Comp(d, a, b))
        {
          resetToExpectAngle();
          return false;
        }

        if (classifyQuatPair(a, b) != QuatPair::Pair01)
        {
          resetToExpectAngle();
          return false;
        }

        setQuatComp(a, stamp);
        setQuatComp(b, stamp);

        state_ = SeqState::ExpectQ23;
        return false;
      }

      case SeqState::ExpectQ23:
      {
        if (kind != CanFrameKind::Quat2Comp)
        {
          resetToExpectAngle();
          return false;
        }

        QuatComp a;
        QuatComp b;
        if (!decodeQuatFrame2Comp(d, a, b))
        {
          resetToExpectAngle();
          return false;
        }

        if (classifyQuatPair(a, b) != QuatPair::Pair23)
        {
          resetToExpectAngle();
          return false;
        }

        setQuatComp(a, stamp);
        setQuatComp(b, stamp);

        // Group end trigger: attempt to build IMU.
        const bool ok = buildImuOnGroupEnd(stamp, refOutMsg);

        // Always reset after the group ends to avoid cross-group mixing.
        resetToExpectAngle();
        return ok;
      }

      default:
      {
        resetToExpectAngle();
        return false;
      }
    }
  }

  bool hasQuatComplete() const
  {
    return has_q_[0] && has_q_[1] && has_q_[2] && has_q_[3];
  }

private:
  enum class SeqState
  {
    ExpectAngle,
    ExpectAcc,
    ExpectGyro,
    ExpectQ01,
    ExpectQ23
  };

  enum class QuatPair
  {
    Invalid,
    Pair01,
    Pair23
  };

  //-------------------------------------------------------------------//
  //- Internal helpers                                                 -//
  //-------------------------------------------------------------------//
  double maxAgeSec() const
  {
    return static_cast<double>(cfg_.max_age_ms) / 1000.0;
  }

  static bool isFresh(const ros::Time& now, const ros::Time& t, double max_age_sec)
  {
    if (t.isZero())
    {
      return false;
    }

    return (now - t).toSec() <= max_age_sec;
  }

  void resetToExpectAngle()
  {
    state_ = SeqState::ExpectAngle;
    t_cycle_start_ = ros::Time(0);

    has_angle_ = false;
    has_acc_ = false;
    has_gyro_ = false;

    t_angle_ = ros::Time(0);
    t_acc_ = ros::Time(0);
    t_gyro_ = ros::Time(0);

    for (int i = 0; i < 4; ++i)
    {
      has_q_[i] = false;
      t_q_[i] = ros::Time(0);
    }

    q_[0] = 1.0;
    q_[1] = 0.0;
    q_[2] = 0.0;
    q_[3] = 0.0;
  }

  void startNewCycle(const ros::Time& stamp)
  {
    resetToExpectAngle();
    t_cycle_start_ = stamp;
  }

  void setQuatComp(const QuatComp& comp, const ros::Time& stamp)
  {
    if (comp.order < 0 || comp.order > 3)
    {
      return;
    }

    q_[comp.order] = comp.value;
    has_q_[comp.order] = true;
    t_q_[comp.order] = stamp;
  }

  static QuatPair classifyQuatPair(const QuatComp& a, const QuatComp& b)
  {
    const int mn = (a.order < b.order) ? a.order : b.order;
    const int mx = (a.order < b.order) ? b.order : a.order;

    if (mn == 0 && mx == 1)
    {
      return QuatPair::Pair01;
    }

    if (mn == 2 && mx == 3)
    {
      return QuatPair::Pair23;
    }

    return QuatPair::Invalid;
  }

  bool quatFreshAndComplete(const ros::Time& now) const
  {
    if (!(has_q_[0] && has_q_[1] && has_q_[2] && has_q_[3]))
    {
      return false;
    }

    const double age = maxAgeSec();
    return isFresh(now, t_q_[0], age) && isFresh(now, t_q_[1], age) && isFresh(now, t_q_[2], age) &&
           isFresh(now, t_q_[3], age);
  }

  bool angleFresh(const ros::Time& now) const
  {
    return has_angle_ && isFresh(now, t_angle_, maxAgeSec());
  }

  bool accFresh(const ros::Time& now) const
  {
    return has_acc_ && isFresh(now, t_acc_, maxAgeSec());
  }

  bool gyroFresh(const ros::Time& now) const
  {
    return has_gyro_ && isFresh(now, t_gyro_, maxAgeSec());
  }

  bool buildImuOnGroupEnd(const ros::Time& now, sensor_msgs::Imu& refOut) const
  {
    // Semantic minimum: gyro must be present.
    if (!gyroFresh(now))
    {
      return false;
    }

    const bool acc_ok = accFresh(now);
    if (cfg_.require_acc && !acc_ok)
    {
      return false;
    }

    // Orientation strategy
    bool use_quat = false;

    if (cfg_.prefer_quat && quatFreshAndComplete(now))
    {
      use_quat = true;
    }
    else if (cfg_.allow_angle_fallback && angleFresh(now))
    {
      use_quat = false;
    }
    else
    {
      if (cfg_.require_orientation)
      {
        return false;
      }
    }

    refOut = sensor_msgs::Imu();
    refOut.header.stamp = now;
    refOut.header.frame_id = cfg_.frame_id;

    for (int i = 0; i < 9; ++i)
    {
      refOut.orientation_covariance[i] = 0.0;
      refOut.angular_velocity_covariance[i] = 0.0;
      refOut.linear_acceleration_covariance[i] = 0.0;
    }

    refOut.orientation_covariance[0] = cfg_.cov_orientation;
    refOut.angular_velocity_covariance[0] = cfg_.cov_angular_velocity;
    refOut.linear_acceleration_covariance[0] = cfg_.cov_linear_acceleration;

    //-----------------------------------------------------------------//
    // Orientation                                                      //
    //-----------------------------------------------------------------//
    if (use_quat)
    {
      double w = q_[0];
      double x = q_[1];
      double y = q_[2];
      double z = q_[3];

      const double n = std::sqrt(w * w + x * x + y * y + z * z);
      if (std::isfinite(n) && n > 1e-9)
      {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
      }
      else
      {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
      }

      refOut.orientation.w = w;
      refOut.orientation.x = x;
      refOut.orientation.y = y;
      refOut.orientation.z = z;
    }
    else
    {
      constexpr double kPi = 3.14159265358979323846;
      const double deg2rad = kPi / 180.0;

      const double roll  = ang_.roll_deg * deg2rad;
      const double pitch = ang_.pitch_deg * deg2rad;
      const double yaw   = ang_.yaw_deg * deg2rad;

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      q.normalize();

      refOut.orientation.w = q.getW();
      refOut.orientation.x = q.getX();
      refOut.orientation.y = q.getY();
      refOut.orientation.z = q.getZ();
    }

    //-----------------------------------------------------------------//
    // Angular velocity [deg/s] -> [rad/s]                              //
    //-----------------------------------------------------------------//
    constexpr double kPi = 3.14159265358979323846;
    const double deg2rad = kPi / 180.0;

    refOut.angular_velocity.x = gyro_.gx_dps * deg2rad;
    refOut.angular_velocity.y = gyro_.gy_dps * deg2rad;
    refOut.angular_velocity.z = gyro_.gz_dps * deg2rad;

    //-----------------------------------------------------------------//
    // Linear acceleration [g] -> [m/s^2]                               //
    //-----------------------------------------------------------------//
    constexpr double g2ms2 = 9.80665;

    refOut.linear_acceleration.x = acc_.ax_g * g2ms2;
    refOut.linear_acceleration.y = acc_.ay_g * g2ms2;
    refOut.linear_acceleration.z = acc_.az_g * g2ms2;

    return true;
  }

  //-------------------------------------------------------------------//
  //- Private members                                                  -//
  //-------------------------------------------------------------------//
  CanImuAggregatorConfig cfg_;

  SeqState state_ = SeqState::ExpectAngle;
  ros::Time t_cycle_start_;

  AngleData ang_;
  AccData acc_;
  GyroData gyro_;

  double q_[4] = { 1.0, 0.0, 0.0, 0.0 };
  bool has_q_[4] = { false, false, false, false };
  ros::Time t_q_[4];

  bool has_angle_ = false;
  bool has_acc_   = false;
  bool has_gyro_  = false;

  ros::Time t_angle_;
  ros::Time t_acc_;
  ros::Time t_gyro_;
};

}  // namespace bw

#endif  // INCLUDE_BW_ROS_DRIVER_CAN_IMU_AGGREGATOR_HPP
