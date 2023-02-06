#include <moveit/robot_model_loader/robot_model_loader.h>
#include <affordance_primitives/screw_planning/screw_planning.hpp>
#include <ap_planning/state_utils.hpp>

namespace ap_planning {
bool checkDuplicateState(const std::vector<std::vector<double>> &states,
                         const std::vector<double> &new_state) {
  for (const auto &state : states) {
    if (state.size() != new_state.size()) {
      return false;
    }

    Eigen::VectorXd error(new_state.size());
    for (size_t i = 0; i < state.size(); ++i) {
      error[i] = new_state.at(i) - state.at(i);
    }
    if (error.norm() < 1e-3) {
      return false;
    }
  }
  return true;
}

ob::ScopedState<> vectorToState(ompl::base::StateSpacePtr space,
                                const std::vector<double> &screw_state,
                                const std::vector<double> &robot_state) {
  ob::ScopedState<> output(space);

  const size_t n_screw = screw_state.size();
  for (size_t i = 0; i < n_screw; ++i) {
    output[i] = screw_state.at(i);
  }

  for (size_t i = 0; i < robot_state.size(); ++i) {
    output[n_screw + i] = robot_state.at(i);
  }

  return output;
}

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
    : ob::StateValidityChecker(si), robot_bounds_(1) {
  // Get move group parameter
  std::string mg_string;
  si->getStateSpace()->params().getParam("move_group", mg_string);
  si->getStateSpace()->params().getParam("ee_frame_name", ee_frame_name_);

  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model->getJointModelGroup(mg_string));

  ob::CompoundStateSpace *compound_space =
      si_->getStateSpace()->as<ob::CompoundStateSpace>();
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

  // Check screw bounds
  for (size_t i = 0; i < constraints->size(); ++i) {
    if (screw_state[i] > constraints->upperBounds()[i] ||
        screw_state[i] < constraints->lowerBounds()[i]) {
      return false;
    }
  }

  // Check robot bounds
  std::vector<double> joint_state(robot_bounds_.low.size());
  for (size_t i = 0; i < robot_bounds_.low.size(); ++i) {
    if (robot_state[i] > robot_bounds_.high[i] ||
        robot_state[i] < robot_bounds_.low[i]) {
      return false;
    }
    joint_state[i] = robot_state[i];
  }

  // Calculate the EE pose for this robot state
  kinematic_state_->setJointGroupPositions(joint_model_group_.get(),
                                           joint_state);
  kinematic_state_->update(true);
  auto this_state_pose = kinematic_state_->getFrameTransform(ee_frame_name_);

  // Call constraintFn
  affordance_primitives::ScrewConstraintSolution sol;
  if (!constraints->constraintFn(this_state_pose, joint_state, sol)) {
    return false;
  }

  if (sol.error > 0.005) {
    return false;
  }

  // Check for collisions
  collision_detection::CollisionResult::ContactMap contacts;
  const planning_scene::PlanningSceneConstPtr ps(*planning_scene);
  ps->getCollidingPairs(contacts, *kinematic_state_);
  if (contacts.size() > 0) {
    return false;
  }

  return true;
}

}  // namespace ap_planning
