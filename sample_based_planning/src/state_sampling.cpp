#include <sample_based_planning/state_sampling.hpp>

namespace ap_planning {

ScrewSampler::ScrewSampler(const ob::SpaceInformation *si)
    : ValidStateSampler(si), screw_bounds_(1) {
  name_ = "my sampler";

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
}

bool ScrewSampler::sample(ob::State *state) {
  ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_state[i] =
        rng_.uniformReal(screw_bounds_.low[i], screw_bounds_.high[i]);
  }

  // Get the pose of this theta
  // TODO: multiple screw axis?
  Eigen::Isometry3d current_pose =
      start_pose_ * screw_axis_.getTF(screw_state[0]);

  geometry_msgs::Pose pose_msg = tf2::toMsg(current_pose);

  // auto t1 = std::chrono::high_resolution_clock::now();
  bool found_ik =
      kinematic_state_->setFromIK(joint_model_group_.get(), pose_msg);
  // auto t2 = std::chrono::high_resolution_clock::now();
  // microseconds_ +=
  //     std::chrono::duration_cast<std::chrono::microseconds>(t2 -
  //     t1).count();
  // std::cout << microseconds_ << "\n";

  // Eigen::VectorXd error(6);
  // error.setZero();
  // affordance_primitives::constraintFn(
  //     kinematic_state->getFrameTransform("panda_link8"), start_pose,
  //     screw_axis, screw_bounds.high[0], error);

  // if (error.norm() > 1e-3) {
  //   std::cout << "Sample error is: " << error.norm() << "\n";
  // }

  if (!found_ik) {
    std::cout << "no IK found\n";
    return false;
  }

  // std::cout << screw_state[0] << "\n";

  // std::cout << "Sample state. Error: " << error.norm() << ". Vals: ";

  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            joint_values);
  for (size_t i = 0; i < joint_values.size(); ++i) {
    robot_state[i] = joint_values[i];
    // std::cout << robot_state[i] << ", ";
  }
  // std::cout << "\n";

  // assert(si_->isValid(state));
  return true;
}

ob::ValidStateSamplerPtr allocScrewSampler(const ob::SpaceInformation *si) {
  return std::make_shared<ScrewSampler>(si);
}

MyStateSampler::MyStateSampler(const ob::StateSpace *state_space)
    : StateSampler(state_space), screw_bounds_(state_space->getDimension()) {
  // TODO: robot description and move group name need to be parameters
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup("panda_arm"));

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

void MyStateSampler::sample(ob::State *state,
                            const std::vector<double> screw_theta) {
  ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  // ob::RealVectorBounds screw_bounds = screw_state.getBounds();

  for (size_t i = 0; i < screw_theta.size(); ++i) {
    screw_state[i] = screw_theta[i];
  }

  // Get the pose of this theta
  // TODO: multiple screw axis?
  Eigen::Isometry3d current_pose =
      start_pose_ * screw_axis_.getTF(screw_state[0]);

  geometry_msgs::Pose pose_msg = tf2::toMsg(current_pose);

  // auto t1 = std::chrono::high_resolution_clock::now();
  bool found_ik =
      kinematic_state_->setFromIK(joint_model_group_.get(), pose_msg);
  // auto t2 = std::chrono::high_resolution_clock::now();
  // microseconds_ +=
  //     std::chrono::duration_cast<std::chrono::microseconds>(t2 -
  //     t1).count();
  // std::cout << microseconds_ << "\n";

  // Eigen::VectorXd error(6);
  // error.setZero();
  // affordance_primitives::constraintFn(
  //     kinematic_state->getFrameTransform("panda_link8"), start_pose,
  //     screw_axis, screw_bounds.high[0], error);

  // if (error.norm() > 1e-3) {
  //   std::cout << "Sample error is: " << error.norm() << "\n";
  // }

  if (!found_ik) {
    std::cout << "no IK found\n";
    return;
  }

  // std::cout << screw_state[0] << "\n";

  // std::cout << "Sample state. Error: " << error.norm() << ". Vals: ";

  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            joint_values);
  for (size_t i = 0; i < joint_values.size(); ++i) {
    robot_state[i] = joint_values[i];
    // std::cout << robot_state[i] << ", ";
  }
  // std::cout << "\n";
}

void MyStateSampler::sampleUniform(ob::State *state) {
  std::vector<double> screw_theta;
  screw_theta.reserve(screw_bounds_.low.size());

  for (size_t i = 0; i < screw_bounds_.low.size(); ++i) {
    screw_theta.push_back(
        rng_.uniformReal(screw_bounds_.low[i], screw_bounds_.high[i]));
  }

  // std::cout << "Uniform sample: ";

  sample(state, screw_theta);
}

void MyStateSampler::sampleUniformNear(ob::State *state, const ob::State *near,
                                       double distance) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *near->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  double screw_dist =
      std::max(std::min(screw_state[0] + distance, screw_bounds_.high[0]),
               screw_bounds_.low[0]);

  std::vector<double> screw_theta{screw_dist};
  sample(state, screw_theta);
}

void MyStateSampler::sampleGaussian(ob::State *state, const ob::State *mean,
                                    double stdDev) {
  const ob::CompoundStateSpace::StateType &compound_state =
      *mean->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  double screw_dist = rng_.gaussian(screw_state[0], stdDev);
  screw_dist = std::max(std::min(screw_dist, screw_bounds_.high[0]),
                        screw_bounds_.low[0]);

  std::vector<double> screw_theta{screw_dist};
  sample(state, screw_theta);
}

ob::StateSamplerPtr allocMyStateSampler(const ob::StateSpace *state_space) {
  return std::make_shared<MyStateSampler>(state_space);
}

}  // namespace ap_planning
