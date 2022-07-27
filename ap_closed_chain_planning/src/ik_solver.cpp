#include <bio_ik/bio_ik.h>
#include <ap_closed_chain_planning/ik_solver.hpp>

namespace ap_closed_chain_planning {
bool solveIK(const moveit::core::JointModelGroup* jmg,
             const geometry_msgs::Pose& target_pose,
             const std::string& ee_frame, moveit::core::RobotState& robot_state,
             trajectory_msgs::JointTrajectoryPoint& point) {
  // Set up BioIK options
  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.replace = true;
  ik_options.return_approximate_solution = true;

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
  bool found_ik = robot_state.setFromIK(
      jmg, geometry_msgs::Pose(), 0,
      moveit::core::GroupStateValidityCallbackFn(), ik_options);

  // Copy to the point
  robot_state.copyJointGroupPositions(jmg, point.positions);

  return found_ik;
}

void IKSolver::initialize(const ros::NodeHandle& nh) {
  nh_ = nh;

  // Get robot description and load model
  std::string robot_description, move_group_name;
  nh_.param<std::string>("robot_description", robot_description,
                         "robot_description");
  if (!nh_.getParam("move_group_name", move_group_name)) {
    // TODO something
  }

  robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
  kinematic_model_ = robot_model_loader.getModel();
  joint_model_group_ = kinematic_model_->getJointModelGroup(move_group_name);
}

ap_planning::Result IKSolver::plan(
    const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
    const moveit::core::RobotStatePtr& start_state,
    trajectory_msgs::JointTrajectory& joint_trajectory) {
  // Copy the state
  moveit::core::RobotState current_state(*start_state);

  // Trajectory time
  double time_from_start{0.0};
  const double time_inc{0.05};

  // Output trajectory setup
  joint_trajectory.header.frame_id = kinematic_model_->getModelFrame();
  joint_trajectory.joint_names = joint_model_group_->getVariableNames();

  // Rip through trajectory and plan
  // Note: these waypoints are defined in the screw's (PLANNING) frame
  for (auto& wp : affordance_traj.trajectory) {
    trajectory_msgs::JointTrajectoryPoint point;
    if (!solveIK(joint_model_group_, wp.pose, "EE_frame", current_state,
                 point)) {
      return ap_planning::NO_IK_SOLUTION;
    }
    // if (joint_trajectory.points.size() > 0 &&
    //     !verify_step(joint_traj.trajectory.at(0).joint_trajectory.points.back(),
    //                  point)) {
    //   return ap_planning::INVALD_TRANSITION;
    // }
    point.time_from_start = ros::Duration(time_from_start);
    time_from_start += time_inc;
    joint_trajectory.points.push_back(point);
  }

  return ap_planning::SUCCESS;
}
}  // namespace ap_closed_chain_planning
