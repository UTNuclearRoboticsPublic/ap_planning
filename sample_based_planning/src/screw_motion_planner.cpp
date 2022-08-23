#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sample_based_planning/screw_motion_planner.hpp>

namespace ap_planning {
APMotionPlanner::APMotionPlanner() {
  // Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();

  // Get information about the robot
  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      kinematic_model_->getJointModelGroup("panda_arm"));
}

bool APMotionPlanner::plan(const APPlanningRequest& req,
                           APPlanningResponse& res) {
  // Set response to failing case
  res.joint_trajectory.joint_names.clear();
  res.joint_trajectory.points.clear();
  res.percentage_complete = 0.0;
  res.trajectory_is_valid = false;

  // Set up the state space for this plan
  if (!setupStateSpace(req)) {
    return false;
  }

  // Set the parameters for the state space
  if (!setSpaceParameters(req, state_space_)) {
    return false;
  }

  // Set the sampler and lock
  state_space_->setStateSamplerAllocator(ap_planning::allocScrewSampler);
  state_space_->as<ob::CompoundStateSpace>()->lock();

  // Create the SimpleSetup class
  ss_ = std::make_shared<og::SimpleSetup>(state_space_);

  // Set state validity checking
  ss_->setStateValidityChecker(
      std::make_shared<ap_planning::ScrewValidityChecker>(
          ss_->getSpaceInformation()));

  // Set valid state sampler
  ss_->getSpaceInformation()->setValidStateSamplerAllocator(
      ap_planning::allocScrewValidSampler);

  // Create a number start states

  // Create a number goal states

  // Set the start and goal states

  // Set planner

  // Plan
}

bool APMotionPlanner::setupStateSpace(const APPlanningRequest& req) {
  state_space_.reset();

  // construct the state space we are planning in
  auto screw_space = std::make_shared<ob::RealVectorStateSpace>();
  auto joint_space = std::make_shared<ob::RealVectorStateSpace>();

  // Add screw dimension
  // TODO: multi-DoF problems?
  // TODO: make sure the theta is positive
  screw_space->addDimension(0, req.theta);

  // Go through joint group and add bounds for each joint
  for (const moveit::core::JointModel* joint :
       joint_model_group_->getActiveJointModels()) {
    const auto& bounds = joint->getVariableBounds(joint->getName());
    if (bounds.position_bounded_) {
      joint_space->addDimension(bounds.min_position_, bounds.max_position_);
    } else {
      // TODO: Add warning or throw?
      return false;
    }
  }

  // combine the state space
  state_space_ = screw_space + joint_space;
  return true;
}

bool APMotionPlanner::setSpaceParameters(const APPlanningRequest& req,
                                         ompl::base::StateSpacePtr& space) {
  // We need to transform the screw to be in the starting frame
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.frame_id = "panda_link0";
  tf_msg.child_frame_id = "panda_link8";
  tf_msg.transform.rotation = req.start_pose.pose.orientation;
  tf_msg.transform.translation.x = req.start_pose.pose.position.x;
  tf_msg.transform.translation.y = req.start_pose.pose.position.y;
  tf_msg.transform.translation.z = req.start_pose.pose.position.z;
  auto transformed_screw =
      affordance_primitives::transformScrew(req.screw_msg, tf_msg);

  // Add screw param (from starting pose screw)
  auto screw_param = std::make_shared<ap_planning::ScrewParam>("screw_param");
  screw_param->setValue(
      affordance_primitives::screwMsgToStr(transformed_screw));
  space->params().add(screw_param);

  // Add starting pose
  auto pose_param = std::make_shared<ap_planning::PoseParam>("pose_param");
  pose_param->setValue(affordance_primitives::poseToStr(req.start_pose));
  space->params().add(pose_param);

  // Add robot description and move group parameters
  // TODO make these obsolete, parameterized, or anything else
  auto robot_description_param =
      std::make_shared<ap_planning::StringParam>("robot_description");
  robot_description_param->setValue("robot_description");
  auto move_group_param =
      std::make_shared<ap_planning::StringParam>("move_group");
  move_group_param->setValue("panda_arm");
  space->params().add(robot_description_param);
  space->params().add(move_group_param);

  return true;
}
}  // namespace ap_planning
