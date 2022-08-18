#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <queue>
#include <thread>
#include <utility>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>

#include <affordance_primitive_msgs/msg/screw_stamped.hpp>
#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <affordance_primitives/screw_planning/screw_planning.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

moveit::core::RobotModelPtr kinematic_model;

static const std::string LOGNAME = "ompl_tests";

struct APPlanningRequest {
  affordance_primitive_msgs::msg::ScrewStamped screw_msg;
  double theta;
  geometry_msgs::msg::Pose start_pose;
};

affordance_primitive_msgs::msg::ScrewStamped strToScrewMsg(
    const std::string input) {
  affordance_primitive_msgs::msg::ScrewStamped output;

  std::stringstream ss(input);
  std::vector<std::string> lines;
  std::string delimiter = ": ";

  for (std::string line; std::getline(ss, line, '\n');) {
    line.erase(0, line.find(delimiter) + delimiter.length());
    lines.push_back(line);
  }

  output.header.frame_id = lines.at(1);
  output.origin.x = std::stod(lines.at(2));
  output.origin.y = std::stod(lines.at(3));
  output.origin.z = std::stod(lines.at(4));
  output.axis.x = std::stod(lines.at(5));
  output.axis.y = std::stod(lines.at(6));
  output.axis.z = std::stod(lines.at(7));

  output.is_pure_translation = lines.at(8) == "Infinity";
  if (!output.is_pure_translation) {
    output.pitch = std::stod(lines.at(8));
  }

  return output;
}

class ScrewParam : public ob::GenericParam {
 public:
  ScrewParam(std::string name) : GenericParam(name) {}

  std::string getValue() const override {
    return affordance_primitives::screwMsgToStr(screw_msg);
  }

  // TODO: probably move this function to affordance primitives
  bool setValue(const std::string &value) {
    screw_msg = strToScrewMsg(value);

    return true;
  }

  affordance_primitive_msgs::msg::ScrewStamped getScrew() const {
    return screw_msg;
  };

 protected:
  affordance_primitive_msgs::msg::ScrewStamped screw_msg;
};

geometry_msgs::msg::Pose strToPoseMsg(const std::string input) {
  geometry_msgs::msg::Pose output;

  std::stringstream ss(input);
  std::vector<std::string> lines;
  std::string delimiter = ": ";

  for (std::string line; std::getline(ss, line, '\n');) {
    line.erase(0, line.find(delimiter) + delimiter.length());
    lines.push_back(line);
  }

  output.position.x = std::stod(lines.at(0));
  output.position.y = std::stod(lines.at(1));
  output.position.z = std::stod(lines.at(2));
  output.orientation.x = std::stod(lines.at(3));
  output.orientation.y = std::stod(lines.at(4));
  output.orientation.z = std::stod(lines.at(5));
  output.orientation.w = std::stod(lines.at(6));

  return output;
}

std::string poseMsgToStr(const geometry_msgs::msg::Pose &pose_msg) {
  std::stringstream ss;
  ss << "Position X: " << pose_msg.position.x
     << "\nPosition Y: " << pose_msg.position.y
     << "\nPosition Z: " << pose_msg.position.z
     << "\nQuaternion X: " << pose_msg.orientation.x
     << "\nQuaternion Y: " << pose_msg.orientation.y
     << "\nQuaternion Z: " << pose_msg.orientation.z
     << "\nQuaternion W: " << pose_msg.orientation.w;

  return ss.str();
}

class PoseParam : public ob::GenericParam {
 public:
  PoseParam(std::string name) : GenericParam(name) {}

  std::string getValue() const override { return poseMsgToStr(pose_msg); }

  bool setValue(const std::string &value) {
    pose_msg = strToPoseMsg(value);

    return true;
  }

  geometry_msgs::msg::Pose getPose() const { return pose_msg; };

 protected:
  geometry_msgs::msg::Pose pose_msg;
};
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

    std::string screw_msg_string, pose_msg_string;
    si->getStateSpace()->params().getParam("screw_param", screw_msg_string);
    si->getStateSpace()->params().getParam("pose_param", pose_msg_string);

    screw_axis.setScrewAxis(strToScrewMsg(screw_msg_string));
    tf2::fromMsg(strToPoseMsg(pose_msg_string), start_pose);
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

    // Get the pose of this theta
    // TODO: multiple screw axis?
    Eigen::Isometry3d current_pose =
        start_pose * screw_axis.getTF(screw_state[0]);

    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(current_pose);

    // auto t1 = std::chrono::high_resolution_clock::now();
    bool found_ik =
        kinematic_state->setFromIK(joint_model_group.get(), pose_msg);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // microseconds_ +=
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 -
    //     t1).count();
    // std::cout << microseconds_ << "\n";

    Eigen::VectorXd error(6);
    error.setZero();
    affordance_primitives::constraintFn(
        kinematic_state->getFrameTransform("panda_link8"), start_pose,
        screw_axis, screw_bounds.high[0], error);

    if (error.norm() > 1e-3) {
      std::cout << "Sample error is: " << error.norm() << "\n";
    }

    if (!found_ik) {
      std::cout << "no IK found\n";
      return false;
    }

    // std::cout << screw_state[0] << "\n";

    std::cout << "Sample state. Error: " << error.norm() << ". Vals: ";

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group.get(),
                                             joint_values);
    for (size_t i = 0; i < joint_values.size(); ++i) {
      robot_state[i] = joint_values[i];
      std::cout << robot_state[i] << ", ";
    }
    std::cout << "\n";

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
  affordance_primitives::ScrewAxis screw_axis;
  Eigen::Isometry3d start_pose;
};

class MyStateSampler : public ob::StateSampler {
 public:
  MyStateSampler(const ob::StateSpace *state_space)
      : StateSampler(state_space), screw_bounds(state_space->getDimension()) {
    kinematic_state =
        std::make_shared<moveit::core::RobotState>(kinematic_model);
    kinematic_state->setToDefaultValues();

    joint_model_group = std::make_shared<moveit::core::JointModelGroup>(
        *kinematic_model->getJointModelGroup("panda_arm"));

    auto compound_space = state_space->as<ob::CompoundStateSpace>();
    screw_bounds = compound_space->getSubspace(0)
                       ->as<ob::RealVectorStateSpace>()
                       ->getBounds();

    std::string screw_msg_string, pose_msg_string;
    state_space->params().getParam("screw_param", screw_msg_string);
    state_space->params().getParam("pose_param", pose_msg_string);

    screw_axis.setScrewAxis(strToScrewMsg(screw_msg_string));
    tf2::fromMsg(strToPoseMsg(pose_msg_string), start_pose);
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

    // Get the pose of this theta
    // TODO: multiple screw axis?
    Eigen::Isometry3d current_pose =
        start_pose * screw_axis.getTF(screw_state[0]);

    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(current_pose);

    // auto t1 = std::chrono::high_resolution_clock::now();
    bool found_ik =
        kinematic_state->setFromIK(joint_model_group.get(), pose_msg);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // microseconds_ +=
    //     std::chrono::duration_cast<std::chrono::microseconds>(t2 -
    //     t1).count();
    // std::cout << microseconds_ << "\n";

    Eigen::VectorXd error(6);
    error.setZero();
    affordance_primitives::constraintFn(
        kinematic_state->getFrameTransform("panda_link8"), start_pose,
        screw_axis, screw_bounds.high[0], error);

    if (error.norm() > 1e-3) {
      std::cout << "Sample error is: " << error.norm() << "\n";
    }

    if (!found_ik) {
      std::cout << "no IK found\n";
      return;
    }

    // std::cout << screw_state[0] << "\n";

    std::cout << "Sample state. Error: " << error.norm() << ". Vals: ";

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group.get(),
                                             joint_values);
    for (size_t i = 0; i < joint_values.size(); ++i) {
      robot_state[i] = joint_values[i];
      std::cout << robot_state[i] << ", ";
    }
    std::cout << "\n";
  }

  void sampleUniform(ob::State *state) override {
    std::vector<double> screw_theta;
    screw_theta.reserve(screw_bounds.low.size());

    for (size_t i = 0; i < screw_bounds.low.size(); ++i) {
      screw_theta.push_back(
          rng_.uniformReal(screw_bounds.low[i], screw_bounds.high[i]));
    }

    // std::cout << "Uniform sample: ";

    sample(state, screw_theta);
  }
  void sampleUniformNear(ob::State *state, const ob::State *near,
                         double distance) override {
    const ob::CompoundStateSpace::StateType &compound_state =
        *near->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

    double screw_dist =
        std::max(std::min(screw_state[0] + distance, screw_bounds.high[0]),
                 screw_bounds.low[0]);

    std::vector<double> screw_theta{screw_dist};
    sample(state, screw_theta);
  }
  void sampleGaussian(ob::State *state, const ob::State *mean,
                      double stdDev) override {
    const ob::CompoundStateSpace::StateType &compound_state =
        *mean->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

    double screw_dist = rng_.gaussian(screw_state[0], stdDev);
    screw_dist = std::max(std::min(screw_dist, screw_bounds.high[0]),
                          screw_bounds.low[0]);

    std::vector<double> screw_theta{screw_dist};
    sample(state, screw_theta);
  }

 protected:
  ompl::RNG rng_;
  moveit::core::RobotStatePtr kinematic_state;
  moveit::core::JointModelGroupPtr joint_model_group;
  ob::RealVectorBounds screw_bounds;
  affordance_primitives::ScrewAxis screw_axis;
  Eigen::Isometry3d start_pose;
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

    std::string screw_msg_string, pose_msg_string;
    si->getStateSpace()->params().getParam("screw_param", screw_msg_string);
    si->getStateSpace()->params().getParam("pose_param", pose_msg_string);
    screw_axis.setScrewAxis(strToScrewMsg(screw_msg_string));
    tf2::fromMsg(strToPoseMsg(pose_msg_string), start_pose);
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

    // std::cout << "Validity state: ";

    std::vector<double> joint_state(robot_bounds.low.size());
    for (size_t i = 0; i < robot_bounds.low.size(); ++i) {
      if (robot_state[i] > robot_bounds.high[i] ||
          robot_state[i] < robot_bounds.low[i]) {
        return false;
      }
      joint_state[i] = robot_state[i];
      // std::cout << joint_state[i] << ", ";
    }
    // std::cout << "\n";

    kinematic_state->setJointGroupPositions("panda_arm", joint_state);
    kinematic_state->update(true);
    auto this_state_pose = kinematic_state->getFrameTransform("panda_link8");

    // const Eigen::Vector3d lin_distance = this_state_pose.translation() -
    // screw_axis.getQVector(); if (lin_distance.norm() > 0.01) {
    //   std::cout << "Broad face: " << lin_distance.norm() << "\n";
    //   return false;
    // }

    Eigen::VectorXd error(6);
    error.setZero();
    if (!affordance_primitives::constraintFn(this_state_pose, start_pose,
                                             screw_axis, screw_bounds.high[0],
                                             error)) {
      return false;
    }

    if (error.norm() > 0.01) {
      std::cout << "Invalid state: " << error.norm() << "\n";
      return false;
    }
    return true;
  }

 protected:
  moveit::core::RobotStatePtr kinematic_state;
  moveit::core::JointModelGroupPtr joint_model_group;
  affordance_primitives::ScrewAxis screw_axis;
  Eigen::Isometry3d start_pose;
};

ob::ValidStateSamplerPtr allocOBValidStateSampler(
    const ob::SpaceInformation *si) {
  return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocScrewSampler(const ob::SpaceInformation *si) {
  return std::make_shared<ScrewSampler>(si);
}

ob::StateSamplerPtr allocMyStateSampler(const ob::StateSpace *state_space) {
  return std::make_shared<MyStateSampler>(state_space);
}

// TODO: document this: both start pose and the screw axis should be given in
// the planning frame!!
ompl::geometric::PathGeometric plan(const APPlanningRequest &req) {
  // construct the state space we are planning in
  auto screw_space(std::make_shared<ob::RealVectorStateSpace>());
  auto joint_space(std::make_shared<ob::RealVectorStateSpace>());

  screw_space->addDimension(0, req.theta);

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

  // We need to transform the screw to be in the starting frame
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "panda_link0";
  tf_msg.child_frame_id = "panda_link8";
  tf_msg.transform.rotation = req.start_pose.orientation;
  tf_msg.transform.translation.x = req.start_pose.position.x;
  tf_msg.transform.translation.y = req.start_pose.position.y;
  tf_msg.transform.translation.z = req.start_pose.position.z;
  auto transformed_screw =
      affordance_primitives::transformScrew(req.screw_msg, tf_msg);

  // define a simple setup class
  ompl::base::StateSpacePtr space = screw_space + joint_space;

  // Add screw param (from starting pose screw)
  auto screw_param = std::make_shared<ScrewParam>("screw_param");
  screw_param->setValue(
      affordance_primitives::screwMsgToStr(transformed_screw));
  space->params().add(screw_param);

  // Add starting pose
  auto pose_param = std::make_shared<PoseParam>("pose_param");
  pose_param->setValue(poseMsgToStr(req.start_pose));
  space->params().add(pose_param);

  space->setStateSamplerAllocator(allocMyStateSampler);
  // TODO: lock space? space.lock()
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(
      std::make_shared<ScrewValidityChecker>(ss.getSpaceInformation()));

  // create a start state
  bool found_ik = kinematic_state->setFromIK(joint_model_group, req.start_pose);
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

  // Find goal pose (in planning frame)
  affordance_primitives::ScrewAxis screw_axis;
  screw_axis.setScrewAxis(transformed_screw);
  const Eigen::Isometry3d planning_to_start = tf2::transformToEigen(tf_msg);
  Eigen::Isometry3d goal_pose = planning_to_start * screw_axis.getTF(req.theta);

  // create a number of goal states
  auto goal_obj = std::make_shared<ScrewGoal>(ss.getSpaceInformation());
  for (size_t i = 0; i < 10; ++i) {
    found_ik =
        kinematic_state->setFromIK(joint_model_group, tf2::toMsg(goal_pose));
    if (!found_ik) {
      std::cout << "Goal: NO IK FOUND!\n";
      continue;
    }

    ob::ScopedState<> goal(space);
    goal[0] = req.theta;

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
  auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
  // planner->setRange(0.25);
  ss.setPlanner(planner);

  ss.getSpaceInformation()->setValidStateSamplerAllocator(allocScrewSampler);

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(10.0);
  if (solved) {
    ss.simplifySolution(5.);
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
    return ss.getSolutionPath();
  } else {
    std::cout << "No solution found" << std::endl;
  }
  return og::PathGeometric(ss.getSpaceInformation());
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

void show_screw(const affordance_primitives::ScrewStamped &screw_msg,
                moveit_visual_tools::MoveItVisualTools &visual_tools) {
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  Eigen::Vector3d axis, origin;
  tf2::fromMsg(screw_msg.axis, axis);
  tf2::fromMsg(screw_msg.origin, origin);

  Eigen::Vector3d end_point = origin + 0.2 * axis.normalized();
  geometry_msgs::msg::Point end = tf2::toMsg(end_point);

  visual_tools.publishArrow(screw_msg.origin, end);
  visual_tools.trigger();
};

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

  std::queue<APPlanningRequest> planning_queue;
  APPlanningRequest single_request;
  single_request.screw_msg.header.frame_id = "panda_link0";

  // For now, all requests start at same point
  single_request.start_pose.position.x = 0.5;
  single_request.start_pose.position.z = 0.3;
  single_request.start_pose.orientation.x = 1.0;
  single_request.start_pose.orientation.w = 0;

  // Add some test cases
  single_request.theta = 0.25 * M_PI;
  single_request.screw_msg.origin = single_request.start_pose.position;
  single_request.screw_msg.axis.x = 1;
  planning_queue.push(single_request);

  single_request.theta = 0.5 * M_PI;
  single_request.screw_msg.axis.x = -1;
  planning_queue.push(single_request);

  single_request.theta = 0.25 * M_PI;
  single_request.screw_msg.axis.x = 0;
  single_request.screw_msg.axis.z = 1;
  planning_queue.push(single_request);

  single_request.screw_msg.origin.x -= 0.1;
  planning_queue.push(single_request);

  single_request.screw_msg.origin = geometry_msgs::msg::Point();
  planning_queue.push(single_request);

  // Plan each screw request
  while (planning_queue.size() > 0 && rclcpp::ok()) {
    auto req = planning_queue.front();
    planning_queue.pop();
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start the demo");

    show_screw(req.screw_msg, visual_tools);

    std::cout << "\nUsing my sampler:" << std::endl;
    auto solution = plan(req);
    solution.interpolate();

    moveit_msgs::msg::DisplayTrajectory joint_traj;
    joint_traj.model_id = "panda";
    joint_traj.trajectory.push_back(moveit_msgs::msg::RobotTrajectory());

    moveit_msgs::msg::RobotState start_msg;
    start_msg.joint_state.name = joint_model_group->getVariableNames();
    auto first_waypoint = ompl_to_msg(solution.getState(0));
    start_msg.joint_state.position = first_waypoint.positions;
    joint_traj.trajectory_start = start_msg;

    joint_traj.trajectory.at(0).joint_trajectory.header.frame_id =
        "panda_link0";
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
  }

  rclcpp::shutdown();
  return 0;
}
