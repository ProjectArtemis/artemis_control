/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Mohammed Kabir, UASys
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_ARTEMIS_CONTROL_COMMON_H_
#define INCLUDE_ARTEMIS_CONTROL_COMMON_H_

#include <Eigen/Eigen>

#include <assert.h>
#include <deque>

#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <artemis_control/Actuators.h>

#include "artemis_control/parameters.h"

namespace artemis_control {


// General conversions 
inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion& msg) {
  // Make sure this always returns a valid Quaternion, even if the message was
  // uninitialized.
  Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
  if (fabs(quaternion.norm() - 1.0) > 0.001) {
    quaternion.setIdentity();
  }
  return quaternion;
}

inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

// EigenOdometry type
struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position = vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = vector3FromMsg(msg->twist.twist.angular);
}

// EigenTrajectoryPoint type 
struct EigenTrajectoryPoint {
  EigenTrajectoryPoint()
      : time_from_start_ns(0),
        position_W(Eigen::Vector3d::Zero()),
        velocity_W(Eigen::Vector3d::Zero()),
        acceleration_W(Eigen::Vector3d::Zero()),
        jerk_W(Eigen::Vector3d::Zero()),
        snap_W(Eigen::Vector3d::Zero()),
        orientation_W_B(Eigen::Quaterniond::Identity()),
        angular_velocity_W(Eigen::Vector3d::Zero()) {};

  EigenTrajectoryPoint(int64_t _time_from_start_ns,
                       const Eigen::Vector3d& _position,
                       const Eigen::Vector3d& _velocity,
                       const Eigen::Vector3d& _acceleration,
                       const Eigen::Vector3d& _jerk,
                       const Eigen::Vector3d& _snap,
                       const Eigen::Quaterniond& _orientation,
                       const Eigen::Vector3d& _angular_velocity)
      : time_from_start_ns(_time_from_start_ns),
        position_W(_position),
        velocity_W(_velocity),
        acceleration_W(_acceleration),
        jerk_W(_jerk),
        snap_W(_snap),
        orientation_W_B(_orientation),
        angular_velocity_W(_angular_velocity) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t time_from_start_ns;
  Eigen::Vector3d position_W;
  Eigen::Vector3d velocity_W;
  Eigen::Vector3d acceleration_W;
  Eigen::Vector3d jerk_W;
  Eigen::Vector3d snap_W;

  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3d angular_velocity_W;

  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const {
    return yawFromQuaternion(orientation_W_B);
  }
  inline double getYawRate() const {
    return angular_velocity_W.z();
  }
  // WARNING: sets roll and pitch to 0.
  inline void setFromYaw(double yaw) {
    orientation_W_B = quaternionFromYaw(yaw);
  }
  inline void setFromYawRate(double yaw_rate) {
    angular_velocity_W.x() = 0.0;
    angular_velocity_W.y() = 0.0;
    angular_velocity_W.z() = yaw_rate;
  }
};

inline void eigenTrajectoryPointFromPoseMsg(
    const geometry_msgs::PoseStamped& msg,
    EigenTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  trajectory_point->time_from_start_ns = 0;
  trajectory_point->position_W = vector3FromPointMsg(msg.pose.position);
  trajectory_point->orientation_W_B = quaternionFromMsg(msg.pose.orientation);
  trajectory_point->velocity_W.setZero();
  trajectory_point->angular_velocity_W.setZero();
  trajectory_point->acceleration_W.setZero();
  trajectory_point->jerk_W.setZero();
  trajectory_point->snap_W.setZero();
}

inline void eigenTrajectoryPointFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
    EigenTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  if (msg.transforms.empty()) {
    ROS_ERROR("MultiDofJointTrajectoryPoint is empty.");
    return;
  }

  if (msg.transforms.size() > 1) {
    ROS_WARN("MultiDofJointTrajectoryPoint message should have one joint, but has %lu. Using first joint.",
             msg.transforms.size());
  }

  trajectory_point->time_from_start_ns = msg.time_from_start.toNSec();
  trajectory_point->position_W = vector3FromMsg(msg.transforms[0].translation);
  trajectory_point->orientation_W_B = quaternionFromMsg(msg.transforms[0].rotation);
  if (msg.velocities.size() > 0) {
    trajectory_point->velocity_W = vector3FromMsg(msg.velocities[0].linear);
    trajectory_point->angular_velocity_W = vector3FromMsg(msg.velocities[0].angular);
  } else {
    trajectory_point->velocity_W.setZero();
    trajectory_point->angular_velocity_W.setZero();
  }
  if (msg.accelerations.size() > 0) {
    trajectory_point->acceleration_W = vector3FromMsg(msg.accelerations[0].linear);
  } else {
    trajectory_point->acceleration_W.setZero();
  }
  trajectory_point->jerk_W.setZero();
  trajectory_point->snap_W.setZero();
}


inline void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
                                      Eigen::Matrix4Xd* allocation_matrix) {
  assert(allocation_matrix != nullptr);
  allocation_matrix->resize(4, rotor_configuration.rotors.size());
  unsigned int i = 0;
  for (const Rotor& rotor : rotor_configuration.rotors) {
    // Set first row of allocation matrix.
    (*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set second row of allocation matrix.
    (*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set third row of allocation matrix.
    (*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant
                                 * rotor.rotor_moment_constant;
    // Set forth row of allocation matrix.
    (*allocation_matrix)(3, i) = rotor.rotor_force_constant;
    ++i;
  }
  Eigen::FullPivLU<Eigen::Matrix4Xd> lu(*allocation_matrix);
  // Setting the threshold for when pivots of the rank calculation should be considered nonzero.
  lu.setThreshold(1e-9);
  int rank = lu.rank();
  if (rank < 4) {
    std::cout << "The rank of the allocation matrix is " << lu.rank()
              << ", it should have rank 4, to have a fully controllable system,"
              << " check your configuration." << std::endl;
  }

}

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
  *skew_matrix << 0, -vector.z(), vector.y(),
                  vector.z(), 0, -vector.x(),
                  -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
  *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}

#define ARTEMIS_CONTROL_CONCATENATE(x, y) x ## y
#define ARTEMIS_CONTROL_CONCATENATE2(x, y) ARTEMIS_CONTROL_CONCATENATE(x, y)
#define ARTEMIS_CONTROL_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE) \
  typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > ARTEMIS_CONTROL_CONCATENATE2(EIGEN_TYPE, Vector); \
  typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > ARTEMIS_CONTROL_CONCATENATE2(EIGEN_TYPE, Deque); \
 
ARTEMIS_CONTROL_MAKE_ALIGNED_CONTAINERS(EigenTrajectoryPoint)
ARTEMIS_CONTROL_MAKE_ALIGNED_CONTAINERS(EigenOdometry)

}

#endif /* INCLUDE_ARTEMIS_CONTROL_COMMON_H_ */
