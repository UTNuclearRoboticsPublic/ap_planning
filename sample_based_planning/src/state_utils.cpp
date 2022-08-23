#include <sample_based_planning/state_utils.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <affordance_primitives/screw_planning/screw_planning.hpp>

namespace ap_planning {
ScrewGoal::ScrewGoal(const ob::SpaceInformationPtr si)
    : GoalStates(si), screw_bounds_(1) {
  ob::CompoundStateSpace *compound_space =
      si_->getStateSpace()->as<ob::CompoundStateSpace>();
  screw_bounds_ = compound_space->getSubspace(0)
                      ->as<ob::RealVectorStateSpace>()
                      ->getBounds();
}

double ScrewGoal::distanceGoal(const ob::State *state) const {
  const ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // TODO make this generic for a vector of screws
  return screw_bounds_.high[0] - screw_state[0];
}

ScrewValidityChecker::ScrewValidityChecker(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si), robot_bounds_(1), screw_bounds_(1) {
  // TODO: robot description and move group name need to be parameters
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup("panda_arm"));

  std::string screw_msg_string, pose_msg_string;
  si->getStateSpace()->params().getParam("screw_param", screw_msg_string);
  si->getStateSpace()->params().getParam("pose_param", pose_msg_string);
  screw_axis_.setScrewAxis(
      *affordance_primitives::strToScrewMsg(screw_msg_string));
  tf2::fromMsg(affordance_primitives::strToPose(pose_msg_string)->pose,
               start_pose_);

  ob::CompoundStateSpace *compound_space =
      si_->getStateSpace()->as<ob::CompoundStateSpace>();
  screw_bounds_ = compound_space->getSubspace(0)
                      ->as<ob::RealVectorStateSpace>()
                      ->getBounds();
  robot_bounds_ = compound_space->getSubspace(1)
                      ->as<ob::RealVectorStateSpace>()
                      ->getBounds();
}

bool ScrewValidityChecker::isValid(const ob::State *state) const {
  const ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    if (screw_state[i] > screw_bounds_.high[i] ||
        screw_state[i] < screw_bounds_.low[i]) {
      return false;
    }
  }

  // std::cout << "Validity state: ";

  std::vector<double> joint_state(robot_bounds_.low.size());
  for (size_t i = 0; i < robot_bounds_.low.size(); ++i) {
    if (robot_state[i] > robot_bounds_.high[i] ||
        robot_state[i] < robot_bounds_.low[i]) {
      return false;
    }
    joint_state[i] = robot_state[i];
    // std::cout << joint_state[i] << ", ";
  }
  // std::cout << "\n";

  kinematic_state_->setJointGroupPositions("panda_arm", joint_state);
  kinematic_state_->update(true);
  auto this_state_pose = kinematic_state_->getFrameTransform("panda_link8");

  // const Eigen::Vector3d lin_distance = this_state_pose.translation() -
  // screw_axis.getQVector(); if (lin_distance.norm() > 0.01) {
  //   std::cout << "Broad face: " << lin_distance.norm() << "\n";
  //   return false;
  // }

  Eigen::VectorXd error(6);
  error.setZero();
  if (!affordance_primitives::constraintFn(this_state_pose, start_pose_,
                                           screw_axis_, screw_bounds_.high[0],
                                           screw_state[0], error)) {
    return false;
  }

  if (error.norm() > 0.001) {
    // std::cout << "Invalid state: " << error.norm() << "\n";
    return false;
  }
  return true;
}

}  // namespace ap_planning
