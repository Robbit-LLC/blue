// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

#include "blue_utils/eigen.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/// @cond
namespace tf2
{

inline double distSq(double x, double y) { return x * x + y * y; }

inline double dist(double x, double y) { return sqrt(distSq(x, y)); }

inline double distSq(double x, double y, double z) { return x * x + y * y + z * z; }

inline double dist(double x, double y, double z) { return sqrt(distSq(x, y, z)); }

inline double dist(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return dist(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

inline void getRpy(const geometry_msgs::msg::Quaternion & q, double & r, double & p, double & y)
{
  tf2::Quaternion tf2_q;
  tf2::fromMsg(q, tf2_q);
  tf2::Matrix3x3(tf2_q).getRPY(r, p, y);
}

inline void setRpy(
  geometry_msgs::msg::Quaternion & q, const double & r, const double & p, const double & y)
{
  tf2::Quaternion tf2_q;
  tf2_q.setRPY(r, p, y);
  q = tf2::toMsg(tf2_q);
}

inline double getYaw(const geometry_msgs::msg::Quaternion & q)
{
  double r;
  double p;
  double y;
  getRpy(q, r, p, y);
  return y;
}

inline void setYaw(geometry_msgs::msg::Quaternion & q, const double & yaw)
{
  double r;
  double p;
  double y;
  getRpy(q, r, p, y);
  setRpy(q, r, p, yaw);
}

inline geometry_msgs::msg::Twist robotToWorldFrame(
  const geometry_msgs::msg::Twist & vel, const double & yaw_f_world)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = vel.linear.x * std::cos(yaw_f_world) - vel.linear.y * sin(yaw_f_world);
  result.linear.y = vel.linear.x * std::sin(yaw_f_world) + vel.linear.y * cos(yaw_f_world);
  result.linear.z = vel.linear.z;
  result.angular.z = vel.angular.z;
  return result;
}

inline bool isZero(const geometry_msgs::msg::Twist & v)
{
  return v.linear.x == 0 && v.linear.y == 0 && v.linear.z == 0 && v.angular.x == 0 &&
         v.angular.y == 0 && v.angular.z == 0;
}

//=====================================================================================
// Time
//=====================================================================================

inline bool valid(const rclcpp::Time & stamp) { return stamp.nanoseconds() > 0; }

// Extend the tf2 namespace to include some commonly used conversions

/**
 * @brief Convert a geometry_msgs::msg::Accel message into an Eigen vector.
 *
 * @param in The Accel message to convert.
 * @param out The Eigen vector that should be populated with the Accel data.
 */
inline void fromMsg(const geometry_msgs::msg::Accel & in, Eigen::Vector6d & out)
{
  Eigen::Vector6d v;
  v << in.linear.x, in.linear.y, in.linear.z, in.angular.x, in.angular.y, in.angular.z;
  out = v;
}

/**
 * @brief Convert a geometry_msgs::msg::Wrench into an Eigen vector.
 *
 * @param in The Wrench message to convert.
 * @param out The Eigen vector that should be populated with the Wrench data.
 */
inline void fromMsg(const geometry_msgs::msg::Wrench & in, Eigen::Vector6d & out)
{
  Eigen::Vector6d v;
  v << in.force.x, in.force.y, in.force.z, in.torque.x, in.torque.y, in.torque.z;
  out = v;
}

/**
 * @brief Convert an Eigen vector into a geometry_msgs::msg::Wrench message.
 *
 * @note This method was renamed to avoid overloading conflicts with the Twist declaration. This
 * is consistent with previous naming applied in the tf2_eigen project.
 *
 * @param in The Eigen vector to convert into a Wrench message.
 * @return geometry_msgs::msg::Wrench
 */
inline geometry_msgs::msg::Wrench toMsg2(const Eigen::Vector6d & in)
{
  geometry_msgs::msg::Wrench msg;
  msg.force.x = in[0];
  msg.force.y = in[1];
  msg.force.z = in[2];
  msg.torque.x = in[3];
  msg.torque.y = in[4];
  msg.torque.z = in[5];
  return msg;
}

inline tf2::Transform poseMsgToTransform(const geometry_msgs::msg::Pose& pose)
{
  tf2::Transform transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

inline geometry_msgs::msg::Pose transformToPoseMsg(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  tf2::toMsg(transform, pose);
  return pose;
}

inline geometry_msgs::msg::Transform transformToTransformMsg(const tf2::Transform & transform)
{
  return tf2::toMsg(transform);
}

inline geometry_msgs::msg::Transform poseMsgToTransformMsg(const geometry_msgs::msg::Pose & pose)
{
  return transformToTransformMsg(poseMsgToTransform(pose));
}

inline geometry_msgs::msg::TransformStamped poseMsgToTransformMsg(
  const geometry_msgs::msg::PoseStamped & msg, const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped result;
  result.header = msg.header;
  result.child_frame_id = child_frame_id;
  result.transform = poseMsgToTransformMsg(msg.pose);
  return result;
}

inline tf2::Transform transformMsgToTransform(const geometry_msgs::msg::Transform & msg)
{
  tf2::Transform transform;
  tf2::fromMsg(msg, transform);
  return transform;
}

inline tf2::Transform transformMsgToTransform(const geometry_msgs::msg::TransformStamped & msg)
{
  return transformMsgToTransform(msg.transform);
}

inline geometry_msgs::msg::PoseStamped transformMsgToPoseMsg(
  const geometry_msgs::msg::TransformStamped & msg)
{
  geometry_msgs::msg::PoseStamped result;
  result.header = msg.header;
  result.pose = transformToPoseMsg(transformMsgToTransform(msg.transform));
  return result;
}

inline geometry_msgs::msg::Pose invert(const geometry_msgs::msg::Pose & pose)
{
  return transformToPoseMsg(poseMsgToTransform(pose).inverse());
}

inline geometry_msgs::msg::PoseStamped invert(
  const geometry_msgs::msg::PoseStamped & msg, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame_id;
  result.header.stamp = msg.header.stamp;
  result.pose = invert(msg.pose);
  return result;
}

//=====================================================================================
// tf2_ros::Buffer functions
//=====================================================================================

inline bool transformWithWait(
  const rclcpp::Logger & logger, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose, int wait_ms)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    out_pose = tf->transform(in_pose, frame, std::chrono::milliseconds(wait_ms));
    return true;
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return false;
  }
}

inline bool transformWithTolerance(
  const rclcpp::Logger & logger, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose, const rclcpp::Duration & tolerance)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // Interpolate
    out_pose = tf->transform(in_pose, frame);
    return true;
  }
  catch (const tf2::ExtrapolationException & e) {
    // Use the most recent transform if possible
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > tolerance) {
      RCLCPP_ERROR(
        logger, "Transform too old when converting from %s to %s", in_pose.header.frame_id.c_str(),
        frame.c_str());
      RCLCPP_ERROR(
        logger, "Data: %ds %uns, transform: %ds %uns", in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec, transform.header.stamp.sec, transform.header.stamp.nanosec);
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return false;
  }
}

inline bool doTransform(
  const std::shared_ptr<tf2_ros::Buffer> & tf, const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose)
{
  if (tf->canTransform(frame, in_pose.header.frame_id, tf2::TimePointZero)) {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(in_pose, out_pose, transform);
    out_pose.header.stamp = in_pose.header.stamp;
    return true;
  } else {
    return false;
  }
}

}  // namespace tf2
/// @endcond

namespace blue::transforms
{

// Coordinate frame IDs
constexpr std::string_view kMapFrameId{"map"};
constexpr std::string_view kMapNedFrameId{"map_ned"};
constexpr std::string_view kBaseLinkFrameId{"base_link"};
constexpr std::string_view kBaseLinkFrdFrameId{"base_link_frd"};

}  // namespace blue::transforms
