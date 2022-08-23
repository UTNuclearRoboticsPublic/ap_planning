#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sample_based_planning/state_sampling.hpp>

namespace ap_planning {

ScrewValidSampler::ScrewValidSampler(const ob::SpaceInformation *si)
    : ValidStateSampler(si), screw_bounds_(1) {
  name_ = "screw_valid_sampler";

  // Get robot description and move group parameters
  std::string rd_string, mg_string;
  si->getStateSpace()->params().getParam("robot_description", rd_string);
  si->getStateSpace()->params().getParam("move_group", mg_string);

  // Load robot
  robot_model_loader::RobotModelLoader robot_model_loader(rd_string);
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup(mg_string));

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
}

bool ScrewValidSampler::sample(ob::State *state) {
  ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  // Draw a random screw state within bounds
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_state[i] =
        rng_.uniformReal(screw_bounds_.low[i], screw_bounds_.high[i]);
  }

  // Get the pose of this theta
  // TODO: multiple screw axis?
  Eigen::Isometry3d current_pose =
      start_pose_ * screw_axis_.getTF(screw_state[0]);
  geometry_msgs::Pose pose_msg = tf2::toMsg(current_pose);

  // Calculate IK for the pose
  bool found_ik =
      kinematic_state_->setFromIK(joint_model_group_.get(), pose_msg);
  if (!found_ik) {
    std::cout << "no IK found\n";
    return false;
  }

  // Convert to robot state
  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            joint_values);
  for (size_t i = 0; i < joint_values.size(); ++i) {
    robot_state[i] = joint_values[i];
  }
  return true;
}

ob::ValidStateSamplerPtr allocScrewValidSampler(
    const ob::SpaceInformation *si) {
  return std::make_shared<ScrewValidSampler>(si);
}

ScrewSampler::ScrewSampler(const ob::StateSpace *state_space)
    : StateSampler(state_space), screw_bounds_(state_space->getDimension()) {
  // Get robot description and move group parameters
  std::string rd_string, mg_string;
  state_space->params().getParam("robot_description", rd_string);
  state_space->params().getParam("move_group", mg_string);

  robot_model_loader::RobotModelLoader robot_model_loader(rd_string);
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup(mg_string));

  auto compound_space = state_space->as<ob::CompoundStateSpace>();
  screw_bounds_ = compound_space->getSubspace(0)
                      ->as<ob::RealVectorStateSpace>()
                      ->getBounds();

  std::string screw_msg_string, pose_msg_string;
  state_space->params().getParam("screw_param", screw_msg_string);
  state_space->params().getParam("pose_param", pose_msg_string);

  screw_axis_.setScrewAxis(
      *affordance_primitives::strToScrewMsg(screw_msg_string));
  tf2::fromMsg(affordance_primitives::strToPose(pose_msg_string)->pose,
               start_pose_);
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
  // TODO: multiple screw axis?
  Eigen::Isometry3d current_pose =
      start_pose_ * screw_axis_.getTF(screw_state[0]);
  geometry_msgs::Pose pose_msg = tf2::toMsg(current_pose);

  // Solve IK for the pose
  bool found_ik =
      kinematic_state_->setFromIK(joint_model_group_.get(), pose_msg);
  if (!found_ik) {
    std::cout << "no IK found\n";
    return;
  }

  // Convert to robot state
  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            joint_values);
  for (size_t i = 0; i < joint_values.size(); ++i) {
    robot_state[i] = joint_values[i];
  }
}

void ScrewSampler::sampleUniform(ob::State *state) {
  std::vector<double> screw_theta;
  screw_theta.reserve(screw_bounds_.low.size());

  // Draw random samples uniformly over the range
  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_theta.push_back(
        rng_.uniformReal(screw_bounds_.low[i], screw_bounds_.high[i]));
  }

  sample(state, screw_theta);
}

void ScrewSampler::sampleUniformNear(ob::State *state, const ob::State *near,
                                     double distance) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *near->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // Calculate a new screw theta at the proper distance and within bounds
  double screw_dist =
      std::max(std::min(screw_state[0] + distance, screw_bounds_.high[0]),
               screw_bounds_.low[0]);

  std::vector<double> screw_theta{screw_dist};
  sample(state, screw_theta);
}

void ScrewSampler::sampleGaussian(ob::State *state, const ob::State *mean,
                                  double stdDev) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *mean->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  // Calculate a new theta based on Gaussian distribution
  double screw_dist = rng_.gaussian(screw_state[0], stdDev);
  screw_dist = std::max(std::min(screw_dist, screw_bounds_.high[0]),
                        screw_bounds_.low[0]);

  std::vector<double> screw_theta{screw_dist};
  sample(state, screw_theta);
}

ob::StateSamplerPtr allocScrewSampler(const ob::StateSpace *state_space) {
  return std::make_shared<ScrewSampler>(state_space);
}

}  // namespace ap_planning
