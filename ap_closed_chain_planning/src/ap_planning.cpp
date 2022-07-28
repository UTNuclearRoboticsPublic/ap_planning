#include <ap_closed_chain_planning/ap_planning.hpp>

namespace ap_planning {
APPlanner::APPlanner(const ros::NodeHandle& nh, const std::string action_name) {
  nh_ = nh;
}

bool APPlanner::initialize() {
  // Read the solver name from the parameter server
  std::string ik_solver_name;
  if (!nh_.getParam(ros::this_node::getName() + "/ik_solver_name",
                    ik_solver_name)) {
    return false;
  }

  // Load solver
  solver_loader_ = std::make_shared<
      pluginlib::ClassLoader<ap_closed_chain_planning::IKSolverBase>>(
      "ap_closed_chain_planning", "ap_closed_chain_planning::IKSolverBase");
  try {
    ik_solver_ = solver_loader_->createInstance(ik_solver_name);
    ik_solver_->initialize(nh_);
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("Solver plugin failed to load, error was: %s", ex.what());
    return false;
  }
  return true;
}

ap_planning::Result APPlanner::plan(
    const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
    const moveit::core::RobotStatePtr& start_state, const std::string& ee_name,
    trajectory_msgs::JointTrajectory& joint_trajectory) {
  return ik_solver_->plan(affordance_traj, start_state, ee_name,
                          joint_trajectory);
}

ap_planning::Result APPlanner::plan(
    const affordance_primitive_msgs::AffordancePrimitiveGoal& ap_goal,
    const moveit::core::RobotStatePtr& start_state,
    trajectory_msgs::JointTrajectory& joint_trajectory) {
  return ik_solver_->plan(ap_goal, start_state, joint_trajectory);
}

}  // namespace ap_planning
