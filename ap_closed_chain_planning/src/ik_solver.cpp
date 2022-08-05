#include <bio_ik/bio_ik.h>
#include <affordance_primitives/screw_model/screw_execution.hpp>
#include <ap_closed_chain_planning/ik_solver.hpp>

#include <pluginlib/class_list_macros.h>

namespace ap_closed_chain_planning {
bool IKSolver::initialize(const ros::NodeHandle& nh) {
  nh_ = nh;
  const std::string n_name = ros::this_node::getName();

  // Get robot description and load model
  std::string robot_description_name, move_group_name;
  nh_.param<std::string>(n_name + "/robot_description_name",
                         robot_description_name, "/robot_description");
  if (!nh_.getParam(n_name + "/move_group_name", move_group_name)) {
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
    return false;
  }
  joint_model_group_ = kinematic_model_->getJointModelGroup(move_group_name);
  if (!joint_model_group_) {
    return false;
  }

  return true;
}

ap_planning::Result IKSolver::verifyTransition(
    const trajectory_msgs::JointTrajectoryPoint& point_a,
    const trajectory_msgs::JointTrajectoryPoint& point_b,
    const moveit::core::JointModelGroup* jmg,
    const moveit::core::RobotState& state_b) {
  if (point_a.positions.size() != point_b.positions.size()) {
    return ap_planning::INVALID_TRANSITION;
  }

  // Check joint diffs for large deltas (joint reconfigurations)
  for (size_t i = 0; i < point_a.positions.size(); i++) {
    if (fabs(point_a.positions.at(i) - point_b.positions.at(i)) >
        joint_tolerance_) {
      return ap_planning::INVALID_TRANSITION;
    }
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
  // Set up BioIK options
  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.replace = true;

  // Set up Pose goal
  auto* pose_goal = new bio_ik::PoseGoal();
  pose_goal->setLinkName(ee_frame);
  pose_goal->setPosition(tf2::Vector3(
      target_pose.position.x, target_pose.position.y, target_pose.position.z));
  pose_goal->setOrientation(
      tf2::Quaternion(target_pose.orientation.x, target_pose.orientation.y,
                      target_pose.orientation.z, target_pose.orientation.w));
  ik_options.goals.emplace_back(pose_goal);

  // Avoid joint limits
  auto* avoid_joint_limits_goal = new bio_ik::AvoidJointLimitsGoal();
  ik_options.goals.emplace_back(avoid_joint_limits_goal);

  // Minimize the displacement
  auto* minimal_displacement_goal = new bio_ik::MinimalDisplacementGoal();
  ik_options.goals.emplace_back(minimal_displacement_goal);

  // Solve the IK
  bool found_ik = false;
  try {
    found_ik = robot_state.setFromIK(
        jmg, geometry_msgs::Pose(), ee_frame, 0.0,
        moveit::core::GroupStateValidityCallbackFn(), ik_options);
  } catch (std::runtime_error& ex) {
    ROS_WARN_STREAM_THROTTLE(5, "Could not solve IK: " << ex.what());
    return false;
  }

  // Copy to the point
  robot_state.copyJointGroupPositions(jmg, point.positions);

  return found_ik;
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

  // Rip through trajectory and plan
  // Note: these waypoints are defined in the screw's (PLANNING) frame
  for (auto& wp : affordance_traj.trajectory) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = wp.time_from_start;
    if (!solveIK(joint_model_group_, wp.pose, ee_name, current_state, point)) {
      return ap_planning::NO_IK_SOLUTION;
    }
    // TODO: check first IK solution is close to start
    if (joint_trajectory.points.size() > 0 &&
        verifyTransition(joint_trajectory.points.back(), point,
                         joint_model_group_,
                         current_state) != ap_planning::SUCCESS) {
      return ap_planning::INVALID_TRANSITION;
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
}  // namespace ap_closed_chain_planning

PLUGINLIB_EXPORT_CLASS(ap_closed_chain_planning::IKSolver,
                       ap_closed_chain_planning::IKSolverBase);
