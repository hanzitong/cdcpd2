#ifndef KINEMATICS_ROS_TYPES_H
#define KINEMATICS_ROS_TYPES_H

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>

namespace kinematics_ros_types
{
  //////////////////////////////////////////////////////////////////////////////
  // Vector3 operators
  //////////////////////////////////////////////////////////////////////////////
  
  inline geometry_msgs::msg::Vector3 operator+ (
      geometry_msgs::msg::Vector3 l, const geometry_msgs::msg::Vector3& r)
  {
    l.x += r.x;
    l.y += r.y;
    l.z += r.z;
    return l;
  }

  inline geometry_msgs::msg::Vector3 operator- (
      geometry_msgs::msg::Vector3 l, const geometry_msgs::msg::Vector3& r)
  {
    l.x -= r.x;
    l.y -= r.y;
    l.z -= r.z;
    return l;
  }

  inline geometry_msgs::msg::Vector3 operator+= (
      geometry_msgs::msg::Vector3& l, const geometry_msgs::msg::Vector3& r)
  {
    l.x += r.x;
    l.y += r.y;
    l.z += r.z;
    return l;
  }

  inline geometry_msgs::msg::Vector3 operator-= (
      geometry_msgs::msg::Vector3& l, const geometry_msgs::msg::Vector3& r)
  {
    l.x -= r.x;
    l.y -= r.y;
    l.z -= r.z;
    return l;
  }


  inline geometry_msgs::msg::Vector3 operator* (
      geometry_msgs::msg::Vector3 l, const double& r)
  {
    l.x *= r;
    l.y *= r;
    l.z *= r;
    return l;
  }

  inline geometry_msgs::msg::Vector3 operator/ (
      geometry_msgs::msg::Vector3 l, const double& r)
  {
    l.x /= r;
    l.y /= r;
    l.z /= r;
    return l;
  }

  inline geometry_msgs::msg::Vector3 operator*= (
      geometry_msgs::msg::Vector3& l, const double& r)
  {
    l.x *= r;
    l.y *= r;
    l.z *= r;
    return l;
  }

  inline geometry_msgs::msg::Vector3 operator/= (
      geometry_msgs::msg::Vector3& l, const double& r)
  {
    l.x /= r;
    l.y /= r;
    l.z /= r;
    return l;
  }


  inline double norm(const geometry_msgs::msg::Vector3& vec)
  {
    return sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Twist operators
  //////////////////////////////////////////////////////////////////////////////
  
  inline geometry_msgs::msg::Twist operator+ (
    geometry_msgs::msg::Twist l, const geometry_msgs::msg::Twist& r)
  {
    l.linear += r.linear;
    l.angular += r.angular;
    return l;
  }

  inline geometry_msgs::msg::Twist operator- (
    geometry_msgs::msg::Twist l, const geometry_msgs::msg::Twist& r)
  {
    l.linear -= r.linear;
    l.angular -= r.angular;
    return l;
  }

  inline geometry_msgs::msg::Twist operator+= (
    geometry_msgs::msg::Twist &l, const geometry_msgs::msg::Twist& r)
  {
    l.linear += r.linear;
    l.angular += r.angular;
    return l;
  }

  inline geometry_msgs::msg::Twist operator-= (
    geometry_msgs::msg::Twist &l, const geometry_msgs::msg::Twist& r)
  {
    l.linear -= r.linear;
    l.angular -= r.angular;
    return l;
  }


  inline geometry_msgs::msg::Twist operator* (
    geometry_msgs::msg::Twist l, const double& r)
  {
    l.linear *= r;
    l.angular *= r;
    return l;
  }

  inline geometry_msgs::msg::Twist operator/ (
    geometry_msgs::msg::Twist l, const double& r)
  {
    l.linear /= r;
    l.angular /= r;
    return l;
  }

  inline geometry_msgs::msg::Twist operator*= (
    geometry_msgs::msg::Twist &l, const double& r)
  {
    l.linear *= r;
    l.angular *= r;
    return l;
  }

  inline geometry_msgs::msg::Twist operator/= (
    geometry_msgs::msg::Twist &l, const double& r)
  {
    l.linear /= r;
    l.angular /= r;
    return l;
  }

  inline geometry_msgs::msg::Twist operator* (
    const double& l, geometry_msgs::msg::Twist r)
  {
    r.linear *= l;
    r.angular *= l;
    return r;
  }

  inline geometry_msgs::msg::Twist operator/ (
    const double& l, geometry_msgs::msg::Twist r)
  {
    r.linear *= l;
    r.angular /= l;
    return r;
  }


  inline geometry_msgs::msg::Twist zeroVelocity()
  {
    geometry_msgs::msg::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    return vel;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Pose2D operators
  //////////////////////////////////////////////////////////////////////////////
  
  inline geometry_msgs::msg::Pose2D zeroPose2D()
  {
    geometry_msgs::msg::Pose2D pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    return pose;
  }

  /**
   * \brief Determines difference between two SE(2) poses that are in the same
   * plane with the same reference frame.
   *
   * \return A body velocity in the current frame that moves us from \a current
   * to \a desired.
   */
  geometry_msgs::msg::Twist calculateError(const geometry_msgs::msg::Pose2D& desired,
                                      const geometry_msgs::msg::Pose2D& current);

}

#endif
