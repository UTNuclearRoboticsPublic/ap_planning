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

  // Set up planning scene monitor
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_description_name);

  return true;
}

void IKSolver::setUp(APPlanningResponse& res) {
  // Set response to failing case
  res.joint_trajectory.joint_names.clear();
  res.joint_trajectory.points.clear();
  res.percentage_complete = 0.0;
  res.trajectory_is_valid = false;
  res.path_length = -1;

  // Get the new planning scene
  if (!planning_scene_) {
    psm_->requestPlanningSceneState();
    planning_scene_ =
        std::make_shared<planning_scene_monitor::LockedPlanningSceneRO>(psm_);
  }
}

void IKSolver::cleanUp() { planning_scene_.reset(); }

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
  // Set up the validation callback to make sure we don't collide with the
  // environment
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn =
      [this, jmg, robot_state](const geometry_msgs::Pose& pose,
                               const std::vector<double>& joints,
                               moveit_msgs::MoveItErrorCodes& error_code) {
        auto state_cpy = robot_state;
        state_cpy.setJointGroupPositions(jmg, joints);
        collision_detection::CollisionResult::ContactMap contacts;
        (*planning_scene_)->getCollidingPairs(contacts, state_cpy);

        if (contacts.size() == 0) {
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        } else {
          error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        }
      };

  // Solve the IK
  std::vector<double> ik_solution, seed_state;
  robot_state.copyJointGroupPositions(jmg, seed_state);
  moveit_msgs::MoveItErrorCodes err;
  if (!ik_solver_->searchPositionIK(target_pose, seed_state, 0.05, ik_solution,
                                    ik_callback_fn, err)) {
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
    const std::vector<double>& start_state, const std::string& ee_name,
    APPlanningResponse& res) {
  // Only supported input is a starting joint state
  if (start_state.size() != joint_model_group_->getVariableCount()) {
    ROS_WARN_STREAM("Starting joint state was size: "
                    << start_state.size() << ", expected size: "
                    << joint_model_group_->getVariableCount());
    return ap_planning::INVALID_GOAL;
  }

  setUp(res);

  // Make a new robot state and copy the starting state
  moveit::core::RobotStatePtr current_state(
      new moveit::core::RobotState(kinematic_model_));
  current_state->setJointGroupPositions(joint_model_group_, start_state);

  // We will check the first IK solution is close to the starting state
  trajectory_msgs::JointTrajectoryPoint starting_point;
  current_state->copyJointGroupPositions(joint_model_group_,
                                         starting_point.positions);

  // Figure out how many waypoints there are
  const size_t num_waypoints = affordance_traj.trajectory.size();
  const double wp_percent = 1 / double(num_waypoints);

  // Rip through trajectory and plan
  // Note: these waypoints are defined in the screw's (PLANNING) frame
  for (auto& wp : affordance_traj.trajectory) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = wp.time_from_start;
    if (!solveIK(joint_model_group_, wp.pose, ee_name, *current_state, point)) {
      cleanUp();
      return ap_planning::NO_IK_SOLUTION;
    }
    if (res.joint_trajectory.points.size() < 1) {
      if (!checkPointsAreClose(starting_point, point)) {
        ROS_ERROR_STREAM("Points are not close!\n"
                         << starting_point << "\n\n"
                         << point);
        cleanUp();
        return ap_planning::INVALID_TRANSITION;
      }
    } else {
      auto transition_result =
          verifyTransition(res.joint_trajectory.points.back(), point,
                           joint_model_group_, *current_state);
      if (transition_result != ap_planning::SUCCESS) {
        cleanUp();
        return transition_result;
      }
    }
    res.percentage_complete += wp_percent;
    res.joint_trajectory.points.push_back(point);
  }

  // Set up the output
  res.joint_trajectory.header.frame_id = kinematic_model_->getModelFrame();
  res.joint_trajectory.joint_names = joint_model_group_->getVariableNames();
  res.trajectory_is_valid = true;
  res.path_length = -1;  // Not implemented

  cleanUp();
  return ap_planning::SUCCESS;
}

ap_planning::Result IKSolver::plan(const APPlanningRequest& req,
                                   APPlanningResponse& res) {
  if (req.screw_path.size() < 1) {
    ROS_WARN_STREAM("Screw path is empty");
    return INVALID_GOAL;
  }

  setUp(res);

  // Make a new robot state
  moveit::core::RobotStatePtr current_state(
      new moveit::core::RobotState(kinematic_model_));
  std::vector<double> starting_joint_config = req.start_joint_state;

  if (req.start_joint_state.size() != joint_model_group_->getVariableCount()) {
    // Use IK to find the first joint state
    const auto first_pose = req.start_pose.pose;
    trajectory_msgs::JointTrajectoryPoint point;
    bool found_start_config = false;
    for (size_t i = 0; i < 5; ++i) {
      current_state->setToRandomPositions();
      if (solveIK(joint_model_group_, first_pose, req.ee_frame_name,
                  *current_state, point)) {
        found_start_config = true;
        break;
      }
    }
    if (!found_start_config) {
      cleanUp();
      return ap_planning::NO_IK_SOLUTION;
    }
    current_state->copyJointGroupPositions(joint_model_group_,
                                           starting_joint_config);
  } else {
    // Copy the starting position
    current_state->setJointGroupPositions(joint_model_group_,
                                          req.start_joint_state);
  }

  // We will check the first IK solution is close to the starting state
  trajectory_msgs::JointTrajectoryPoint starting_point;
  current_state->copyJointGroupPositions(joint_model_group_,
                                         starting_point.positions);
  current_state->update(true);

  // Create an AP Goal to use internally
  affordance_primitives::AffordancePrimitiveGoal ap_goal;
  ap_goal.moving_frame_source = ap_goal.PROVIDED;
  ap_goal.moving_to_task_frame = tf2::eigenToTransform(
      current_state->getGlobalLinkTransform(req.ee_frame_name).inverse());
  ap_goal.moving_to_task_frame.header.frame_id = req.ee_frame_name;
  ap_goal.moving_to_task_frame.child_frame_id =
      kinematic_model_->getModelFrame();

  // Go through each screw axis and add it to the path
  affordance_primitives::AffordanceTrajectory full_trajectory;
  ros::Duration duration_shift(0);
  for (const auto& segment : req.screw_path) {
    ap_goal.screw = segment.screw_msg;
    ap_goal.screw_distance = segment.theta;

    // TODO: think about getting this in as a param
    ap_goal.theta_dot = 0.1;

    // Check the tf was valid
    std::optional<geometry_msgs::TransformStamped> tfmsg_moving_to_task =
        screw_executor_.getTFInfo(ap_goal);
    if (!tfmsg_moving_to_task.has_value()) {
      cleanUp();
      return ap_planning::INVALID_GOAL;
    }

    // Figure out how many waypoints to do
    const size_t num_waypoints = calculateNumWaypoints(
        segment.screw_msg, *tfmsg_moving_to_task, segment.theta);

    // Generate the affordance trajectory
    std::optional<affordance_primitives::AffordanceTrajectory> ap_trajectory =
        screw_executor_.getTrajectoryCommands(ap_goal, num_waypoints);
    if (!ap_trajectory.has_value()) {
      cleanUp();
      return ap_planning::INVALID_GOAL;
    }

    // Add trajectory
    for (const auto& wp : ap_trajectory->trajectory) {
      full_trajectory.trajectory.push_back(wp);
      full_trajectory.trajectory.back().time_from_start += duration_shift;
    }

    // Set up for next segment
    duration_shift = full_trajectory.trajectory.back().time_from_start;
    duration_shift += ap_trajectory->trajectory.at(1).time_from_start;

    Eigen::Isometry3d last_wp;
    tf2::fromMsg(full_trajectory.trajectory.back().pose, last_wp);
    ap_goal.moving_to_task_frame = tf2::eigenToTransform(last_wp.inverse());
    ap_goal.moving_to_task_frame.header.frame_id = req.ee_frame_name;
    ap_goal.moving_to_task_frame.child_frame_id =
        kinematic_model_->getModelFrame();
  }

  // Do the planning
  return plan(full_trajectory, starting_joint_config, req.ee_frame_name, res);
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
