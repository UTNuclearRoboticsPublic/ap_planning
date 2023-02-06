#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ap_planning/state_sampling.hpp>
#include <ap_planning/state_utils.hpp>

namespace ap_planning {

bool ikCallbackFnAdapter(const moveit::core::JointModelGroupPtr jmg,
                         const moveit::core::RobotStatePtr robot_state,
                         const planning_scene_monitor::LockedPlanningSceneRO ps,
                         const std::vector<double> &joints,
                         moveit_msgs::MoveItErrorCodes &error_code) {
  // Copy the IK solution to the robot state
  auto state_cpy = robot_state;
  state_cpy->setJointGroupPositions(jmg.get(), joints);

  // Check for collisions
  collision_detection::CollisionResult::ContactMap contacts;
  ps->getCollidingPairs(contacts, *state_cpy);

  // Set the error code
  if (contacts.size() == 0) {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } else {
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  }
  return true;
}

void increaseStateList(const moveit::core::JointModelGroupPtr jmg,
                       const moveit::core::RobotStatePtr robot_state,
                       const planning_scene_monitor::LockedPlanningSceneRO ps,
                       const kinematics::KinematicsBasePtr ik_solver,
                       const affordance_primitives::Pose &pose,
                       std::vector<std::vector<double>> &state_list) {
  // Set up IK callback
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn =
      [&jmg, &robot_state, &ps](const geometry_msgs::Pose &pose,
                                const std::vector<double> &joints,
                                moveit_msgs::MoveItErrorCodes &error_code) {
        ikCallbackFnAdapter(jmg, robot_state, ps, joints, error_code);
      };

  // Try to solve the IK
  std::vector<double> seed_state, ik_solution;
  moveit_msgs::MoveItErrorCodes err;
  kinematics::KinematicsQueryOptions opts;
  robot_state->copyJointGroupPositions(jmg.get(), seed_state);
  if (!ik_solver->searchPositionIK(pose, seed_state, 0.05, ik_solution,
                                   ik_callback_fn, err, opts)) {
    return;
  }

  // If the solution is valid, add it to the list
  if (checkDuplicateState(state_list, ik_solution)) {
    state_list.push_back(ik_solution);
  }
}

ScrewValidSampler::ScrewValidSampler(const ob::SpaceInformation *si)
    : ValidStateSampler(si) {
  name_ = "screw_valid_sampler";

  // Get move group parameters
  std::string mg_string;
  si->getStateSpace()->params().getParam("move_group", mg_string);

  // Load robot
  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model->getJointModelGroup(mg_string));

  ik_solver_ = joint_model_group_->getSolverInstance();
}

bool ScrewValidSampler::sample(ob::State *state) {
  ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  // Draw a random screw state within bounds
  auto sampled_state = constraints->sampleUniformState();
  for (size_t i = 0; i < sampled_state.size(); ++i) {
    screw_state[i] = sampled_state[i];
  }

  // Get the pose of this theta
  Eigen::Isometry3d current_pose = constraints->getPose(sampled_state);
  geometry_msgs::Pose pose_msg = tf2::toMsg(current_pose);

  // Set up IK callback
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn =
      [this](const geometry_msgs::Pose &pose, const std::vector<double> &joints,
             moveit_msgs::MoveItErrorCodes &error_code) {
        ikCallbackFnAdapter(joint_model_group_, kinematic_state_,
                            *planning_scene, joints, error_code);
      };

  // Calculate IK for the pose
  std::vector<double> ik_solution, seed_state;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            seed_state);
  moveit_msgs::MoveItErrorCodes err;
  kinematics::KinematicsQueryOptions opts;
  // opts.return_approximate_solution = true;
  bool found_ik = ik_solver_->searchPositionIK(
      pose_msg, seed_state, 0.05, ik_solution, ik_callback_fn, err, opts);
  if (!found_ik) {
    return false;
  }

  kinematic_state_->setJointGroupActivePositions(joint_model_group_.get(),
                                                 ik_solution);
  kinematic_state_->update();

  // Convert to robot state
  for (size_t i = 0; i < ik_solution.size(); ++i) {
    robot_state[i] = ik_solution[i];
  }
  return true;
}

ob::ValidStateSamplerPtr allocScrewValidSampler(
    const ob::SpaceInformation *si) {
  return std::make_shared<ScrewValidSampler>(si);
}

ScrewSampler::ScrewSampler(const ob::StateSpace *state_space)
    : StateSampler(state_space) {
  // Get move group parameters
  std::string mg_string;
  state_space->params().getParam("move_group", mg_string);

  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model->getJointModelGroup(mg_string));

  ik_solver_ = joint_model_group_->getSolverInstance();
}

void ScrewSampler::sample(ob::State *state,
                          const std::vector<double> screw_theta) {
  ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  // Set the screw state from input
  for (size_t i = 0; i < screw_theta.size(); ++i) {
    screw_state[i] = screw_theta[i];
  }

  // Get the pose of this theta
  Eigen::Isometry3d current_pose = constraints->getPose(screw_theta);
  geometry_msgs::Pose pose_msg = tf2::toMsg(current_pose);

  // Set up IK callback
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn =
      [this](const geometry_msgs::Pose &pose, const std::vector<double> &joints,
             moveit_msgs::MoveItErrorCodes &error_code) {
        ikCallbackFnAdapter(joint_model_group_, kinematic_state_,
                            *planning_scene, joints, error_code);
      };

  // Solve IK for the pose
  std::vector<double> ik_solution, seed_state;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            seed_state);
  moveit_msgs::MoveItErrorCodes err;
  kinematics::KinematicsQueryOptions opts;
  // opts.return_approximate_solution = true;
  bool found_ik = ik_solver_->searchPositionIK(
      pose_msg, seed_state, 0.05, ik_solution, ik_callback_fn, err, opts);
  if (!found_ik) {
    return;
  }

  kinematic_state_->setJointGroupPositions(joint_model_group_.get(),
                                           ik_solution);
  kinematic_state_->update();

  // // Convert to robot state
  for (size_t i = 0; i < ik_solution.size(); ++i) {
    robot_state[i] = ik_solution[i];
  }
}

void ScrewSampler::sampleUniform(ob::State *state) {
  sample(state, constraints->sampleUniformState());
}

void ScrewSampler::sampleUniformNear(ob::State *state, const ob::State *near,
                                     double distance) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *near->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // Extract the state
  std::vector<double> screw_theta(constraints->size());
  for (size_t i = 0; i < constraints->size(); ++i) {
    screw_theta[i] = screw_state[i];
  }

  sample(state, constraints->sampleUniformStateNear(screw_theta, distance));
}

void ScrewSampler::sampleGaussian(ob::State *state, const ob::State *mean,
                                  double stdDev) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *mean->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // Extract the state
  std::vector<double> screw_theta(constraints->size());
  for (size_t i = 0; i < constraints->size(); ++i) {
    screw_theta[i] = screw_state[i];
  }

  sample(state, constraints->sampleGaussianStateNear(screw_theta, stdDev));
}

ob::StateSamplerPtr allocScrewSampler(const ob::StateSpace *state_space) {
  return std::make_shared<ScrewSampler>(state_space);
}

}  // namespace ap_planning
