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

#include "artemis_control/trajectory_controller_node.h"
//#include "artemis_control/parameters_ros.h"

namespace artemis_control {

TrajectoryControllerNode::TrajectoryControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

  cmd_pose_sub_ = nh.subscribe("command/pose", 1,
      &TrajectoryControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe("command/trajectory", 1,
      &TrajectoryControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe("state", 1,
                               &TrajectoryControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<artemis_control::Actuators>("actuator_controls", 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &TrajectoryControllerNode::TimedCommandCallback, this,
                                  true, false);
}

TrajectoryControllerNode::~TrajectoryControllerNode() { }

void TrajectoryControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  trajectory_controller_.controller_parameters_.position_gain_.x(),
                  &trajectory_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  trajectory_controller_.controller_parameters_.position_gain_.y(),
                  &trajectory_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  trajectory_controller_.controller_parameters_.position_gain_.z(),
                  &trajectory_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  trajectory_controller_.controller_parameters_.velocity_gain_.x(),
                  &trajectory_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  trajectory_controller_.controller_parameters_.velocity_gain_.y(),
                  &trajectory_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  trajectory_controller_.controller_parameters_.velocity_gain_.z(),
                  &trajectory_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  trajectory_controller_.controller_parameters_.attitude_gain_.x(),
                  &trajectory_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  trajectory_controller_.controller_parameters_.attitude_gain_.y(),
                  &trajectory_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  trajectory_controller_.controller_parameters_.attitude_gain_.z(),
                  &trajectory_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  trajectory_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &trajectory_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  trajectory_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &trajectory_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  trajectory_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &trajectory_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &trajectory_controller_.vehicle_parameters_);
  trajectory_controller_.InitializeParameters();
}
void TrajectoryControllerNode::Publish() {
}

void TrajectoryControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  EigenTrajectoryPoint eigen_reference;
  eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  trajectory_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void TrajectoryControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  EigenTrajectoryPoint eigen_reference;
  eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  trajectory_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void TrajectoryControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const EigenTrajectoryPoint eigen_reference = commands_.front();
  trajectory_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void TrajectoryControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("TrajectoryController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  trajectory_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  trajectory_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  artemis_control::ActuatorsPtr actuator_msg(new artemis_control::Actuators);

  actuator_msg->angular_velocities.clear();
  actuator_msg->header.stamp = odometry_msg->header.stamp;
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);

  motor_velocity_reference_pub_.publish(actuator_msg);
  
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_controller_node");

  artemis_control::TrajectoryControllerNode trajectory_controller_node;

  ros::spin();

  return 0;
}
