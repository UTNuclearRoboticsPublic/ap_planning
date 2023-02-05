#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ap_planning/state_sampling.hpp>

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

ScrewValidSampler::ScrewValidSampler(const ob::SpaceInformation *si)
    : ValidStateSampler(si), screw_bounds_(1) {
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

  std::string screw_msg_string, pose_msg_string;
  si->getStateSpace()->params().getParam("screw_param", screw_msg_string);
  si->getStateSpace()->params().getParam("pose_param", pose_msg_string);
  tf2::fromMsg(affordance_primitives::strToPose(pose_msg_string)->pose,
               start_pose_);

  const auto screw_msgs =
      affordance_primitives::strToScrewMsgVector(screw_msg_string);
  screw_axes_.reserve(screw_msgs.size());
  for (const auto &msg : screw_msgs) {
    affordance_primitives::ScrewAxis axis;
    axis.setScrewAxis(msg);
    screw_axes_.push_back(axis);
  }

  ob::CompoundStateSpace *compound_space =
      si_->getStateSpace()->as<ob::CompoundStateSpace>();
  screw_bounds_ = compound_space->getSubspace(0)
                      ->as<ob::RealVectorStateSpace>()
                      ->getBounds();

  screw_constraints_.screw_axis_set = screw_axes_;
  screw_constraints_.phi_bounds.first.resize(screw_axes_.size());
  screw_constraints_.phi_bounds.second.resize(screw_axes_.size());

  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_constraints_.phi_bounds.first(i) = screw_bounds_.low[i];
    screw_constraints_.phi_bounds.second(i) = screw_bounds_.high[i];
  }
}

bool ScrewValidSampler::sample(ob::State *state) {
  ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  // Draw a random screw state within bounds
  Eigen::VectorXd phi(screw_axes_.size());
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_state[i] =
        rng_.uniformReal(screw_bounds_.low[i], screw_bounds_.high[i]);
    phi(i) = screw_state[i];
  }

  // Get the pose of this theta
  // TODO: multiple screw axis?
  Eigen::Isometry3d current_pose =
      affordance_primitives::getPoseOnPath(screw_constraints_, phi);
  // screw_axis_.getTF(screw_state[0]) * start_pose_;
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
    : StateSampler(state_space), screw_bounds_(state_space->getDimension()) {
  // Get move group parameters
  std::string mg_string;
  state_space->params().getParam("move_group", mg_string);

  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model->getJointModelGroup(mg_string));

  ik_solver_ = joint_model_group_->getSolverInstance();

  auto compound_space = state_space->as<ob::CompoundStateSpace>();
  screw_bounds_ = compound_space->getSubspace(0)
                      ->as<ob::RealVectorStateSpace>()
                      ->getBounds();

  std::string screw_msg_string, pose_msg_string;
  state_space->params().getParam("screw_param", screw_msg_string);
  state_space->params().getParam("pose_param", pose_msg_string);
  tf2::fromMsg(affordance_primitives::strToPose(pose_msg_string)->pose,
               start_pose_);

  const auto screw_msgs =
      affordance_primitives::strToScrewMsgVector(screw_msg_string);
  screw_axes_.reserve(screw_msgs.size());
  for (const auto &msg : screw_msgs) {
    affordance_primitives::ScrewAxis axis;
    axis.setScrewAxis(msg);
    screw_axes_.push_back(axis);
  }

  screw_constraints_.screw_axis_set = screw_axes_;
  screw_constraints_.phi_bounds.first.resize(screw_axes_.size());
  screw_constraints_.phi_bounds.second.resize(screw_axes_.size());

  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_constraints_.phi_bounds.first(i) = screw_bounds_.low[i];
    screw_constraints_.phi_bounds.second(i) = screw_bounds_.high[i];
  }
  lambda_max_ = affordance_primitives::getLambda(
      screw_constraints_.phi_bounds.second, screw_constraints_.phi_bounds);
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
  Eigen::VectorXd phi(screw_axes_.size());
  for (size_t i = 0; i < screw_theta.size(); ++i) {
    screw_state[i] = screw_theta[i];
    phi(i) = screw_state[i];
  }

  // Get the pose of this theta
  // TODO: multiple screw axis?
  Eigen::Isometry3d current_pose =
      affordance_primitives::getPoseOnPath(screw_constraints_, phi);
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
  std::vector<double> screw_theta;
  screw_theta.reserve(screw_bounds_.low.size());

  // Draw a random sample along the chain
  double lambda = rng_.uniformReal(0, lambda_max_);

  auto phi =
      affordance_primitives::getPhi(lambda, screw_constraints_.phi_bounds);

  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_theta.push_back(phi(i));
  }

  sample(state, screw_theta);
}

void ScrewSampler::sampleUniformNear(ob::State *state, const ob::State *near,
                                     double distance) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *near->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // Calculate the lambda of the near state
  Eigen::VectorXd phi(screw_bounds_.low.size());
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    phi(i) = screw_state[i];
  }
  const double lambda_near =
      affordance_primitives::getLambda(phi, screw_constraints_.phi_bounds);

  // Sample near it
  double lambda =
      lambda_near + rng_.uniformReal(-0.5 * distance, 0.5 * distance);

  // Enforce the bounds
  lambda = std::clamp(lambda, 0.0, lambda_max_);

  // Put back into phi-form
  phi = affordance_primitives::getPhi(lambda, screw_constraints_.phi_bounds);

  std::vector<double> screw_theta;
  screw_theta.reserve(screw_bounds_.low.size());
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_theta[i] = phi(i);
  }

  sample(state, screw_theta);
}

void ScrewSampler::sampleGaussian(ob::State *state, const ob::State *mean,
                                  double stdDev) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *mean->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // Calculate the lambda of the near state
  Eigen::VectorXd phi(screw_bounds_.low.size());
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    phi(i) = screw_state[i];
  }
  const double lambda_near =
      affordance_primitives::getLambda(phi, screw_constraints_.phi_bounds);

  // Sample near it
  double lambda = rng_.gaussian(lambda_near, stdDev);

  // Enforce the bounds
  lambda = std::clamp(lambda, 0.0, lambda_max_);

  // Put back into phi-form
  phi = affordance_primitives::getPhi(lambda, screw_constraints_.phi_bounds);

  std::vector<double> screw_theta;
  screw_theta.reserve(screw_bounds_.low.size());
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_theta[i] = phi(i);
  }

  sample(state, screw_theta);
}

ob::StateSamplerPtr allocScrewSampler(const ob::StateSpace *state_space) {
  return std::make_shared<ScrewSampler>(state_space);
}

}  // namespace ap_planning
