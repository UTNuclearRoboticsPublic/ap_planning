#include <ap_planning/naive_planner.hpp>

namespace ap_planning {
NaivePlanner::NaivePlanner(const ros::NodeHandle& nh) : initialized_(false) {
  nh_ = nh;
}

bool NaivePlanner::initialize() {
  // Read the solver name from the parameter server
  std::string ik_solver_name;
  if (!nh_.getParam(ros::this_node::getName() + "/ik_solver_name",
                    ik_solver_name)) {
    ROS_ERROR_STREAM("Could not load parameter: " << std::string(
                         ros::this_node::getName() + "/ik_solver_name"));
    return false;
  }

  // Load solver
  solver_loader_ =
      std::make_shared<pluginlib::ClassLoader<ap_planning::IKSolverBase>>(
          "ap_planning", "ap_planning::IKSolverBase");
  try {
    ik_solver_ = solver_loader_->createInstance(ik_solver_name);
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("Solver plugin failed to load, error was: %s", ex.what());
    return false;
  }
  initialized_ = ik_solver_->initialize(nh_);
  return initialized_;
}

ap_planning::Result NaivePlanner::plan(
    const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
    const std::vector<double>& start_state, const std::string& ee_name,
    APPlanningResponse& res) {
  // If we haven't already initialized, do so
  if (!initialized_ && !initialize()) {
    return ap_planning::INITIALIZATION_FAIL;
  }

  if (!ik_solver_) {
    return ap_planning::INITIALIZATION_FAIL;
  }
  return ik_solver_->plan(affordance_traj, start_state, ee_name, res);
}

ap_planning::Result NaivePlanner::plan(const APPlanningRequest& req,
                                       APPlanningResponse& res) {
  // If we haven't already initialized, do so
  if (!initialized_ && !initialize()) {
    return ap_planning::INITIALIZATION_FAIL;
  }

  if (!ik_solver_) {
    return ap_planning::INITIALIZATION_FAIL;
  }
  return ik_solver_->plan(req, res);
}

}  // namespace ap_planning
