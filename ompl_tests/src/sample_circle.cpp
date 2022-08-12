#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <iostream>
#include <thread>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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
    kinematic_state =
        std::make_shared<moveit::core::RobotState>(kinematic_model);
    kinematic_state->setToDefaultValues();

    joint_model_group = std::make_shared<moveit::core::JointModelGroup>(
        *kinematic_model->getJointModelGroup("panda_arm"));
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

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = 0.5 * cos(screw_state[0]);
    pose_msg.position.y = 0.5 * sin(screw_state[0]);
    pose_msg.position.z = 0.3;
    pose_msg.orientation.x = 1.0;
    pose_msg.orientation.w = 0;

    // auto t1 = std::chrono::high_resolution_clock::now();
    bool found_ik =
        kinematic_state->setFromIK(joint_model_group.get(), pose_msg);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // microseconds_ +=
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 -
    //     t1).count();
    // std::cout << microseconds_ << "\n";

    if (!found_ik) {
      return false;
    }

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group.get(),
                                             joint_values);
    for (size_t i = 0; i < joint_values.size(); ++i) {
      robot_state[i] = joint_values[i];
    }

    assert(si_->isValid(state));
    return true;
  }
  // We don't need this in the example below.
  bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/,
                  const double /*distance*/) override {
    std::this_thread::sleep_for(ompl::time::seconds(.0005));
    std::cout << "TRIED TO SAMPLE NEAR\n";
    std::this_thread::sleep_for(ompl::time::seconds(.0005));
    throw ompl::Exception("ScrewSampler::sampleNear", "not implemented");
    return false;
  }

  double microseconds_;

 protected:
  ompl::RNG rng_;
  moveit::core::RobotStatePtr kinematic_state;
  moveit::core::JointModelGroupPtr joint_model_group;
};

class MyStateSampler : public ob::StateSampler {
 public:
  MyStateSampler(const ob::StateSpace *state_space) : StateSampler(state_space), screw_bounds(state_space->getDimension()) {
    kinematic_state =
        std::make_shared<moveit::core::RobotState>(kinematic_model);
    kinematic_state->setToDefaultValues();

    joint_model_group = std::make_shared<moveit::core::JointModelGroup>(
        *kinematic_model->getJointModelGroup("panda_arm"));

    auto compound_space = state_space->as<ob::CompoundStateSpace>();
    screw_bounds = compound_space->getSubspace(0)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();
  }

  void sample(ob::State *state, const std::vector<double> screw_theta) {
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

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = 0.5 * cos(screw_state[0]);
    pose_msg.position.y = 0.5 * sin(screw_state[0]);
    pose_msg.position.z = 0.3;
    pose_msg.orientation.x = 1.0;
    pose_msg.orientation.w = 0;

    // auto t1 = std::chrono::high_resolution_clock::now();
    bool found_ik =
        kinematic_state->setFromIK(joint_model_group.get(), pose_msg);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // microseconds_ +=
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 -
    //     t1).count();
    // std::cout << microseconds_ << "\n";


    if (!found_ik) {
      std::cout << "no IK found\n";
      return;
    }

    std::cout << screw_state[0] << "\n";

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group.get(),
                                             joint_values);
    for (size_t i = 0; i < joint_values.size(); ++i) {
      robot_state[i] = joint_values[i];
    }
  }

  void sampleUniform(ob::State *state) override {
    std::vector<double> screw_theta;
    screw_theta.reserve(screw_bounds.low.size());

    for (size_t i=0; i<screw_bounds.low.size(); ++i) {
      screw_theta.push_back(rng_.uniformReal(screw_bounds.low[i], screw_bounds.high[i]));
    }

    std::cout << "Uniform sample: ";

    sample(state, screw_theta);
  }
  void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override {
    const ob::CompoundStateSpace::StateType &compound_state =
        *near->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

    double screw_dist = std::max(std::min(screw_state[0]+distance, screw_bounds.high[0]), screw_bounds.low[0]);

    std::cout << "UniformNear sample: ";
    std::vector<double> screw_theta{screw_dist};
    sample(state, screw_theta);
  }
  void sampleGaussian (ob::State *state, const ob::State *mean, double stdDev) override {
    const ob::CompoundStateSpace::StateType &compound_state =
        *mean->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

    double screw_dist = rng_.gaussian(screw_state[0], stdDev);
    screw_dist = std::max(std::min(screw_dist, screw_bounds.high[0]), screw_bounds.low[0]);

    std::cout << "Gaussian sample: ";
    std::vector<double> screw_theta{screw_dist};
    sample(state, screw_theta);
  }

 protected:
  ompl::RNG rng_;
  moveit::core::RobotStatePtr kinematic_state;
  moveit::core::JointModelGroupPtr joint_model_group;
  ob::RealVectorBounds screw_bounds;
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

ob::StateSamplerPtr allocMyStateSampler(
    const ob::StateSpace *state_space) {
  return std::make_shared<MyStateSampler>(state_space);
}

ompl::geometric::PathGeometric plan(int samplerIndex) {
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
  space->setStateSamplerAllocator(allocMyStateSampler);
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
  goal_pose_msg.orientation.x = 1.0;
  goal_pose_msg.orientation.w = 0;

  for (size_t i = 0; i < 10; ++i) {
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

  // create a planner for the defined space
  auto planner(std::make_shared<og::RRT>(ss.getSpaceInformation()));
  planner->setRange(0.5);
  ss.setPlanner(planner);

  // set sampler (optional; the default is uniform sampling)
  if (samplerIndex == 1)
    // use obstacle-based sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        allocOBValidStateSampler);
  else if (samplerIndex == 2)
    // use my sampler
    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocScrewSampler);


  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(10.0);
  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
  } else
    std::cout << "No solution found" << std::endl;

  return ss.getSolutionPath();
}

trajectory_msgs::msg::JointTrajectoryPoint ompl_to_msg(const ob::State *state) {
  trajectory_msgs::msg::JointTrajectoryPoint output;
  output.positions.reserve(7);

  const ob::CompoundStateSpace::StateType &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &robot_state =
      *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

  for (size_t i = 0; i < 7; ++i) {
    output.positions.push_back(robot_state[i]);
  }

  return output;
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

  // std::vector<double> goal_joint_state_found{
  //     2.55424, -0.0783369, -2.4615, -2.35386, 0.0215131, 2.35256, 0.0805782};

  // kinematic_state->setJointGroupPositions("panda_arm",
  // goal_joint_state_found); kinematic_state->update(true); auto pose_eig =
  // kinematic_state->getFrameTransform("panda_link8"); std::cout <<
  // "\n\n\n\n\n\n\nCalculated Goal IK:\n"
  //           << pose_eig.translation() << "\n";

  rclcpp::sleep_for(std::chrono::seconds(2));

  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.setRobotStateTopic("/display_robot_state");
  // visual_tools.publishRobotState(kinematic_state);
  visual_tools.trigger();

  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the demo");

  // std::cout << "Using default uniform sampler:" << std::endl;
  // plan(0);
  // std::cout << "\nUsing obstacle-based sampler:" << std::endl;
  // plan(1);
  std::cout << "\nUsing my sampler:" << std::endl;
  auto solution = plan(2);

  moveit_msgs::msg::DisplayTrajectory joint_traj;
  joint_traj.model_id = "panda";
  joint_traj.trajectory.push_back(moveit_msgs::msg::RobotTrajectory());

  moveit_msgs::msg::RobotState start_msg;
  start_msg.joint_state.name = joint_model_group->getVariableNames();
  auto first_waypoint = ompl_to_msg(solution.getState(0));
  start_msg.joint_state.position = first_waypoint.positions;
  joint_traj.trajectory_start = start_msg;

  joint_traj.trajectory.at(0).joint_trajectory.header.frame_id = "panda_link0";
  joint_traj.trajectory.at(0).joint_trajectory.joint_names =
      joint_model_group->getVariableNames();

  int time = 0;

  size_t num_waypoints = solution.getStateCount();
  for (size_t i = 0; i < num_waypoints; ++i) {
    auto wp = ompl_to_msg(solution.getState(i));
    wp.time_from_start.sec = time;
    joint_traj.trajectory.at(0).joint_trajectory.points.push_back(wp);

    ++time;
  }

  visual_tools.publishTrajectoryPath(joint_traj);

  rclcpp::shutdown();
  return 0;
}
