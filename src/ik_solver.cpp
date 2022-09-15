#include <bio_ik/bio_ik.h>
#include <affordance_primitives/screw_model/screw_execution.hpp>
#include <ap_planning/ik_solver.hpp>

#include <pluginlib/class_list_macros.h>

namespace ap_planning {
bool IKSolver::initialize(const ros::NodeHandle& nh) {
  nh_ = nh;
  const std::string n_name = ros::this_node::getName();

  // Get robot description and load model
  std::string robot_description_name, move_group_name;
  nh_.param<std::string>(n_name + "/robot_description_name",
                         robot_description_name, "/robot_description");
  if (!nh_.getParam(n_name + "/move_group_name", move_group_name)) {
    ROS_ERROR_STREAM("Could not find parameter: "
                     << std::string(n_name + "/move_group_name"));
    return false;
  }

  // Get planning parameters
  nh_.param<double>(n_name + "/waypoint_dist", waypoint_dist_, WAYPOINT_DIST);
  nh_.param<double>(n_name + "/waypoint_ang", waypoint_ang_, WAYPOINT_ANG);
  nh_.param<double>(n_name + "/joint_tolerance", joint_tolerance_,
                    JOINT_TOLERANCE);
  nh_.param<double>(n_name + "/condition_num_limit", condition_num_limit_,
                    CONDITION_NUM_LIMIT);

  robot_model_loader::RobotModelLoader robot_model_loader(
      robot_description_name);
  kinematic_model_ = robot_model_loader.getModel();
  if (!kinematic_model_) {
    ROS_ERROR("Could not load RobotModel");
    return false;
  }
  joint_model_group_ = kinematic_model_->getJointModelGroup(move_group_name);
  if (!joint_model_group_) {
    ROS_ERROR_STREAM("Could not find joint model group: " << move_group_name);
    return false;
  }

  ik_solver_ = joint_model_group_->getSolverInstance();
  if (!ik_solver_) {
    ROS_ERROR("Could not get IK Solver");
    return false;
  } else if (!ik_solver_->supportsGroup(joint_model_group_)) {
    ROS_ERROR_STREAM(
        "IK Solver doesn't support group: " << joint_model_group_->getName());
    return false;
  }

  return true;
}

bool IKSolver::checkPointsAreClose(
    const trajectory_msgs::JointTrajectoryPoint& point_a,
    const trajectory_msgs::JointTrajectoryPoint& point_b) {
  if (point_a.positions.size() != point_b.positions.size()) {
    return ap_planning::INVALID_TRANSITION;
  }

  // Check joint diffs for large deltas (joint reconfigurations)
  for (size_t i = 0; i < point_a.positions.size(); i++) {
    if (fabs(point_a.positions.at(i) - point_b.positions.at(i)) >
        joint_tolerance_) {
      return false;
    }
  }
  return true;
}

ap_planning::Result IKSolver::verifyTransition(
    const trajectory_msgs::JointTrajectoryPoint& point_a,
    const trajectory_msgs::JointTrajectoryPoint& point_b,
    const moveit::core::JointModelGroup* jmg,
    const moveit::core::RobotState& state_b) {
  // Do broad face check to avoid joint reconfigurations
  if (!checkPointsAreClose(point_a, point_b)) {
    return ap_planning::INVALID_TRANSITION;
  }

  // Check joint velocities
  const double wp_duration =
      (point_b.time_from_start - point_a.time_from_start).toSec();
  if (wp_duration <= 0) {
    ROS_WARN_STREAM_THROTTLE(5, "Time between waypoints must be positive");
    return ap_planning::INVALID_TRANSITION;
  }

  // Go through each joint
  size_t i = 0;
  for (const moveit::core::JointModel* joint : jmg->getActiveJointModels()) {
    const auto& bounds = joint->getVariableBounds(joint->getName());
    if (bounds.velocity_bounded_) {
      const double vel =
          (point_b.positions.at(i) - point_a.positions.at(i)) / wp_duration;
      if (vel > bounds.max_velocity_ || vel < bounds.min_velocity_) {
        ROS_WARN_STREAM_THROTTLE(5, "Velocity limit exceeded");
        return ap_planning::INVALID_TRANSITION;
      }
    }
    ++i;
  }

  // Check for a singular position
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(state_b.getJacobian(jmg));
  const double cond_number =
      svd.singularValues()[0] /
      svd.singularValues()[svd.singularValues().size() - 1];
  if (cond_number > condition_num_limit_) {
    ROS_WARN_STREAM_THROTTLE(5, "Singularity: " << cond_number);
    return ap_planning::INVALID_TRANSITION;
  }

  return ap_planning::SUCCESS;
}

bool IKSolver::solveIK(const moveit::core::JointModelGroup* jmg,
                       const geometry_msgs::Pose& target_pose,
                       const std::string& ee_frame,
                       moveit::core::RobotState& robot_state,
                       trajectory_msgs::JointTrajectoryPoint& point) {
  // Solve the IK
  std::vector<double> ik_solution, seed_state;
  robot_state.copyJointGroupPositions(jmg, seed_state);
  moveit_msgs::MoveItErrorCodes err;
  if (!ik_solver_->searchPositionIK(target_pose, seed_state, 0.05, ik_solution,
                                    err)) {
    ROS_WARN_STREAM_THROTTLE(5, "Could not solve IK");
    return false;
  }

  // Update the robot state
  robot_state.setJointGroupPositions(jmg, ik_solution);
  robot_state.update();

  // Copy to the point
  robot_state.copyJointGroupPositions(jmg, point.positions);

  return true;
}

ap_planning::Result IKSolver::plan(
    const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
    const moveit::core::RobotStatePtr& start_state, const std::string& ee_name,
    trajectory_msgs::JointTrajectory& joint_trajectory) {
  // Copy the state
  moveit::core::RobotState current_state = *start_state;

  // Output trajectory setup
  joint_trajectory.header.frame_id = kinematic_model_->getModelFrame();
  joint_trajectory.joint_names = joint_model_group_->getVariableNames();

  // We will check the first IK solution is close to the starting state
  trajectory_msgs::JointTrajectoryPoint starting_point;
  current_state.copyJointGroupPositions(joint_model_group_,
                                        starting_point.positions);

  // Rip through trajectory and plan
  // Note: these waypoints are defined in the screw's (PLANNING) frame
  for (auto& wp : affordance_traj.trajectory) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = wp.time_from_start;
    if (!solveIK(joint_model_group_, wp.pose, ee_name, current_state, point)) {
      return ap_planning::NO_IK_SOLUTION;
    }
    if (joint_trajectory.points.size() < 1) {
      if (!checkPointsAreClose(starting_point, point)) {
        ROS_ERROR_STREAM("Points are not close!\n"
                         << starting_point << "\n\n"
                         << point);
        return ap_planning::INVALID_TRANSITION;
      }
    } else {
      auto transition_result =
          verifyTransition(joint_trajectory.points.back(), point,
                           joint_model_group_, current_state);
      if (transition_result != ap_planning::SUCCESS) {
        return transition_result;
      }
    }
    joint_trajectory.points.push_back(point);
  }

  return ap_planning::SUCCESS;
}

ap_planning::Result IKSolver::plan(
    const affordance_primitive_msgs::AffordancePrimitiveGoal& ap_goal,
    const moveit::core::RobotStatePtr& start_state,
    trajectory_msgs::JointTrajectory& joint_trajectory) {
  // Grab the ee_name
  std::optional<geometry_msgs::TransformStamped> tfmsg_moving_to_task =
      screw_executor_.getTFInfo(ap_goal);
  if (!tfmsg_moving_to_task.has_value()) {
    return ap_planning::INVALID_GOAL;
  }
  const std::string ee_name = tfmsg_moving_to_task->header.frame_id;

  // Figure out how many waypoints to do
  const size_t num_waypoints = calculateNumWaypoints(
      ap_goal.screw, *tfmsg_moving_to_task, ap_goal.screw_distance);

  // Generate the affordance trajectory
  std::optional<affordance_primitives::AffordanceTrajectory> ap_trajectory =
      screw_executor_.getTrajectoryCommands(ap_goal, num_waypoints);
  if (!ap_trajectory.has_value()) {
    return ap_planning::INVALID_GOAL;
  }

  // Do the planning
  return plan(*ap_trajectory, start_state, ee_name, joint_trajectory);
}

size_t IKSolver::calculateNumWaypoints(
    const affordance_primitive_msgs::ScrewStamped& screw_msg,
    const geometry_msgs::TransformStamped& tf_msg, const double theta) {
  // Pure translation case
  if (screw_msg.is_pure_translation) {
    return ceil(fabs(theta) / waypoint_dist_);
  }

  // Find the distance from moving frame to screw axis
  Eigen::Vector3d tf_dist, screw_origin_dist;
  tf2::fromMsg(tf_msg.transform.translation, tf_dist);
  tf2::fromMsg(screw_msg.origin, screw_origin_dist);
  const double screw_distance = (tf_dist + screw_origin_dist).norm();

  // Calculate min waypoints for distance limiting and angle limiting
  const size_t wps_ang = ceil(fabs(theta) / waypoint_ang_);
  const size_t wps_lin = ceil(fabs(theta) * screw_distance / waypoint_dist_);

  // Return whichever required more waypoints
  return std::max(wps_ang, wps_lin);
}
}  // namespace ap_planning

PLUGINLIB_EXPORT_CLASS(ap_planning::IKSolver, ap_planning::IKSolverBase);