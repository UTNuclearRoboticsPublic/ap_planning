#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <iostream>
#include <thread>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

moveit::core::RobotModelPtr kinematic_model;

static const std::string LOGNAME = "ompl_tests";

// TODO: change this to GoalLazySamples
class ScrewGoal : public ob::GoalStates {
 public:
  ScrewGoal(const ob::SpaceInformationPtr si) : GoalStates(si) {}

  double distanceGoal(const ob::State *state) const override {
    const ob::CompoundStateSpace::StateType &compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

    // TODO: make these class members so less redoing the same steps over and
    // over
    ob::CompoundStateSpace *compound_space =
        si_->getStateSpace()->as<ob::CompoundStateSpace>();
    ob::RealVectorBounds screw_bounds = compound_space->getSubspace(0)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();

    // TODO make this generic for a vector of screws
    return screw_bounds.high[0] - screw_state[0];
  }
};

class ScrewSampler : public ob::ValidStateSampler {
 public:
  ScrewSampler(const ob::SpaceInformation *si) : ValidStateSampler(si) {
    name_ = "my sampler";
  }
  bool sample(ob::State *state) override {
    ob::CompoundStateSpace::StateType &compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType &robot_state =
        *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

    // TODO: make these class members so less redoing the same steps over and
    // over
    ob::CompoundStateSpace *compound_space =
        si_->getStateSpace()->as<ob::CompoundStateSpace>();
    ob::RealVectorBounds screw_bounds = compound_space->getSubspace(0)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();

    for (size_t i = 0; i < screw_bounds.low.size(); ++i) {
      screw_state[i] =
          rng_.uniformReal(screw_bounds.low[i], screw_bounds.high[i]);
    }

    robot_state[0] = cos(screw_state[0]);
    robot_state[1] = sin(screw_state[0]);
    robot_state[2] = screw_state[0];

    assert(si_->isValid(state));
    return true;
  }
  // We don't need this in the example below.
  bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/,
                  const double /*distance*/) override {
    throw ompl::Exception("ScrewSampler::sampleNear", "not implemented");
    return false;
  }

 protected:
  ompl::RNG rng_;
};

bool isNear(double a, double b, double tol = 1e-3) {
  return fabs(a - b) < fabs(tol);
}

class ScrewValidityChecker : public ob::StateValidityChecker {
 public:
  ScrewValidityChecker(const ob::SpaceInformationPtr &si)
      : ob::StateValidityChecker(si) {
    kinematic_state =
        std::make_shared<moveit::core::RobotState>(kinematic_model);
    kinematic_state->setToDefaultValues();

    joint_model_group = std::make_shared<moveit::core::JointModelGroup>(
        *kinematic_model->getJointModelGroup("panda_arm"));
  }

  virtual bool isValid(const ob::State *state) const {
    const ob::CompoundStateSpace::StateType &compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &robot_state =
        *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

    // TODO: make these class members so less redoing the same steps over and
    // over
    ob::CompoundStateSpace *compound_space =
        si_->getStateSpace()->as<ob::CompoundStateSpace>();
    ob::RealVectorBounds screw_bounds = compound_space->getSubspace(0)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();
    ob::RealVectorBounds robot_bounds = compound_space->getSubspace(1)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();

    for (size_t i = 0; i < screw_bounds.low.size(); ++i) {
      if (screw_state[i] > screw_bounds.high[i] ||
          screw_state[i] < screw_bounds.low[i]) {
        return false;
      }
    }

    std::vector<double> joint_state(robot_bounds.low.size());
    for (size_t i = 0; i < robot_bounds.low.size(); ++i) {
      if (robot_state[i] > robot_bounds.high[i] ||
          robot_state[i] < robot_bounds.low[i]) {
        return false;
      }
      joint_state[i] = robot_state[i];
    }

    kinematic_state->setJointGroupPositions("panda_arm", joint_state);
    kinematic_state->update(true);
    auto pose_eig = kinematic_state->getFrameTransform("panda_link8");

    // std::cout << "Screw: " << screw_state[0] << "\nTranslation:\n"
    //           << pose_eig.translation() << "\n";

    if (!isNear(pose_eig.translation().x(), 0.5 * cos(screw_state[0]))) {
      return false;
    }

    if (!isNear(pose_eig.translation().y(), 0.5 * sin(screw_state[0]))) {
      return false;
    }

    if (!isNear(pose_eig.translation().z(), 0.3)) {
      return false;
    }
    return true;
  }

 protected:
  moveit::core::RobotStatePtr kinematic_state;
  moveit::core::JointModelGroupPtr joint_model_group;
};

ob::ValidStateSamplerPtr allocOBValidStateSampler(
    const ob::SpaceInformation *si) {
  return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocScrewSampler(const ob::SpaceInformation *si) {
  return std::make_shared<ScrewSampler>(si);
}

void plan(int samplerIndex) {
  // construct the state space we are planning in
  auto screw_space(std::make_shared<ob::RealVectorStateSpace>());
  auto joint_space(std::make_shared<ob::RealVectorStateSpace>());

  screw_space->addDimension(0, 0.5 * M_PI);

  moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group =
      kinematic_model->getJointModelGroup("panda_arm");

  // Go through joint group and add bounds for each joint
  for (const moveit::core::JointModel *joint :
       joint_model_group->getActiveJointModels()) {
    const auto &bounds = joint->getVariableBounds(joint->getName());
    // TODO: add case for non-bounded joint?
    if (bounds.position_bounded_) {
      joint_space->addDimension(bounds.min_position_, bounds.max_position_);
    }
  }

  // define a simple setup class
  ompl::base::StateSpacePtr space = screw_space + joint_space;
  // TODO: lock space? space.lock()
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(
      std::make_shared<ScrewValidityChecker>(ss.getSpaceInformation()));

  // create a start state
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = 0.5;
  pose_msg.position.z = 0.3;
  pose_msg.orientation.x = 1.0;
  pose_msg.orientation.w = 0;

  bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg);
  if (!found_ik) {
    std::cout << "No initial IK found\n";
  }

  ob::ScopedState<> start(space);
  start[0] = 0;
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (size_t i = 0; i < joint_values.size(); ++i) {
    start[i + 1] = joint_values[i];
  }

  // create a number of goal states
  auto goal_obj = std::make_shared<ScrewGoal>(ss.getSpaceInformation());
  geometry_msgs::msg::Pose goal_pose_msg;
  goal_pose_msg.position.y = 0.5;
  goal_pose_msg.position.z = 0.3;
  goal_pose_msg.orientation.x = 0.7071068;
  goal_pose_msg.orientation.y = 0.7071068;
  goal_pose_msg.orientation.z = 0.0;
  goal_pose_msg.orientation.w = 0;

  for (size_t i = 0; i < 2; ++i) {
    found_ik = kinematic_state->setFromIK(joint_model_group, goal_pose_msg);
    if (!found_ik) {
      std::cout << "Goal: NO IK FOUND!\n";
      continue;
    }

    // kinematic_state->update(true);
    // auto pose_eig = kinematic_state->getFrameTransform("panda_link8");
    // std::cout << "\nGoal Translation:\n" << pose_eig.translation() << "\n";

    ob::ScopedState<> goal(space);
    goal[0] = 0.5 * M_PI;

    // std::cout << "Goal joint state:\n";
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (size_t i = 0; i < joint_values.size(); ++i) {
      goal[i + 1] = joint_values[i];
      // std::cout << joint_values[i] << "\n";
    }
    goal_obj->addState(goal);
  }

  // set the start and goal states
  ss.setStartState(start);
  ss.setGoal(goal_obj);

  // set sampler (optional; the default is uniform sampling)
  if (samplerIndex == 1)
    // use obstacle-based sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        allocOBValidStateSampler);
  else if (samplerIndex == 2)
    // use my sampler
    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocScrewSampler);

  // create a planner for the defined space
  auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(10.0);
  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
  } else
    std::cout << "No solution found" << std::endl;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ompl_constrained_planning_demo_node",
                                        node_options);

  const auto &LOGGER = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s",
              kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group =
      kinematic_model->getJointModelGroup("panda_arm");

  const std::vector<std::string> &joint_names =
      joint_model_group->getVariableNames();

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = 0.5;
  pose_msg.position.z = 0.3;
  pose_msg.orientation.x = 1.0;
  pose_msg.orientation.w = 0;

  bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg);

  RCLCPP_INFO(LOGGER, "\n\n\n\n\n\n");

  if (found_ik) {
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(),
                  joint_values[i]);
    }

    RCLCPP_INFO_STREAM(
        LOGGER,
        kinematic_state->getFrameTransform("panda_link8").translation());
  }

  std::vector<double> goal_joint_state_found;
  goal_joint_state_found.push_back(1.83049);
  goal_joint_state_found.push_back(0.0543568);
  goal_joint_state_found.push_back(-0.259276);
  goal_joint_state_found.push_back(-2.37838);
  goal_joint_state_found.push_back(0.0213562);
  goal_joint_state_found.push_back(2.43081);
  goal_joint_state_found.push_back(-0.0154);

  kinematic_state->setJointGroupPositions("panda_arm", goal_joint_state_found);
  kinematic_state->update(true);
  auto pose_eig = kinematic_state->getFrameTransform("panda_link8");
  std::cout << "\n\n\n\n\n\n\nCalculated Goal IK:\n"
            << pose_eig.translation() << "\n";

  rclcpp::sleep_for(std::chrono::seconds(5));

  std::cout << "Using default uniform sampler:" << std::endl;
  plan(0);
  // std::cout << "\nUsing obstacle-based sampler:" << std::endl;
  // plan(1);
  // std::cout << "\nUsing my sampler:" << std::endl;
  // plan(2);

  rclcpp::shutdown();
  return 0;
}