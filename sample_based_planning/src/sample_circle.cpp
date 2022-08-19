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
#include <ros/ros.h>

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <affordance_primitives/screw_planning/screw_planning.hpp>
#include <sample_based_planning/state_sampling.hpp>
#include <sample_based_planning/state_utils.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ap_planning {

// TODO: probably move this function to affordance primitives
affordance_primitive_msgs::ScrewStamped strToScrewMsg(const std::string input) {
  affordance_primitive_msgs::ScrewStamped output;

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

geometry_msgs::Pose strToPoseMsg(const std::string input) {
  geometry_msgs::Pose output;

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

std::string poseMsgToStr(const geometry_msgs::Pose &pose_msg) {
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

  screw_axis_.setScrewAxis(strToScrewMsg(screw_msg_string));
  tf2::fromMsg(strToPoseMsg(pose_msg_string), start_pose_);

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

  screw_axis_.setScrewAxis(strToScrewMsg(screw_msg_string));
  tf2::fromMsg(strToPoseMsg(pose_msg_string), start_pose_);
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

// bool isNear(double a, double b, double tol = 1e-3) {
//   return fabs(a - b) < fabs(tol);
// }

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
  screw_axis_.setScrewAxis(strToScrewMsg(screw_msg_string));
  tf2::fromMsg(strToPoseMsg(pose_msg_string), start_pose_);

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

}  // namespace ap_planning

// TODO: document this: both start pose and the screw axis should be given in
// the planning frame!!
std::pair<ompl::geometric::PathGeometric, double> plan(
    const ap_planning::APPlanningRequest &req,
    const std::vector<std::vector<double>> &start_configs,
    const std::vector<std::vector<double>> &goal_configs) {
  // construct the state space we are planning in
  auto screw_space(std::make_shared<ob::RealVectorStateSpace>());
  auto joint_space(std::make_shared<ob::RealVectorStateSpace>());

  screw_space->addDimension(0, req.theta);

  // TODO: robot description and move group name need to be parameters
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();

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
  geometry_msgs::TransformStamped tf_msg;
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
  auto screw_param = std::make_shared<ap_planning::ScrewParam>("screw_param");
  screw_param->setValue(
      affordance_primitives::screwMsgToStr(transformed_screw));
  space->params().add(screw_param);

  // Add starting pose
  auto pose_param = std::make_shared<ap_planning::PoseParam>("pose_param");
  pose_param->setValue(ap_planning::poseMsgToStr(req.start_pose));
  space->params().add(pose_param);

  space->setStateSamplerAllocator(ap_planning::allocMyStateSampler);
  // TODO: lock space? space.lock()
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(
      std::make_shared<ap_planning::ScrewValidityChecker>(
          ss.getSpaceInformation()));

  // create a number start states
  for (const auto &start_state : start_configs) {
    ob::ScopedState<> start(space);
    start[0] = 0;
    for (size_t i = 0; i < start_state.size(); ++i) {
      start[i + 1] = start_state[i];
    }
    ss.addStartState(start);
  }

  // // Find goal pose (in planning frame)
  // affordance_primitives::ScrewAxis screw_axis;
  // screw_axis.setScrewAxis(transformed_screw);
  // const Eigen::Isometry3d planning_to_start = tf2::transformToEigen(tf_msg);
  // Eigen::Isometry3d goal_pose = planning_to_start *
  // screw_axis.getTF(req.theta);

  // create a number of goal states
  auto goal_obj =
      std::make_shared<ap_planning::ScrewGoal>(ss.getSpaceInformation());
  for (const auto &goal_state : goal_configs) {
    ob::ScopedState<> goal(space);
    goal[0] = req.theta;
    for (size_t i = 0; i < goal_state.size(); ++i) {
      goal[i + 1] = goal_state[i];
    }
    goal_obj->addState(goal);
  }

  // set the start and goal states
  ss.setGoal(goal_obj);

  // create a planner for the defined space
  auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
  // planner->setRange(0.25);
  ss.setPlanner(planner);

  ss.getSpaceInformation()->setValidStateSamplerAllocator(
      ap_planning::allocScrewSampler);

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(5.0);
  if (solved) {
    ss.simplifySolution(5.);
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
    return std::make_pair(ss.getSolutionPath(),
                          ss.getLastPlanComputationTime());
  } else {
    std::cout << "No solution found" << std::endl;
  }
  return std::make_pair(og::PathGeometric(ss.getSpaceInformation()), 0.0);
}

// TODO: check waypoints for validity
bool solutionIsValid(og::PathGeometric &solution,
                     const ap_planning::APPlanningRequest &req) {
  if (solution.getStateCount() < 1) {
    return false;
  }

  const ob::CompoundStateSpace::StateType &compound_state =
      *solution.getStates().back()->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  const double err = fabs(req.theta - screw_state[0]);
  if (err > 0.01) {
    return false;
  }

  return true;
}

trajectory_msgs::JointTrajectoryPoint ompl_to_msg(const ob::State *state) {
  trajectory_msgs::JointTrajectoryPoint output;
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
  geometry_msgs::Point end = tf2::toMsg(end_point);

  visual_tools.publishArrow(screw_msg.origin, end);
  visual_tools.trigger();
}

bool checkDuplicateState(const std::vector<std::vector<double>> &states,
                         const std::vector<double> &new_state) {
  for (const auto &state : states) {
    if (state.size() != new_state.size()) {
      return false;
    }

    Eigen::VectorXd eig_new_state(new_state.size());
    Eigen::VectorXd eig_old_state(state.size());
    for (size_t i = 0; i < state.size(); ++i) {
      eig_new_state[i] = new_state.at(i);
      eig_old_state[i] = state.at(i);
    }

    const auto error = eig_new_state - eig_old_state;
    if (error.norm() < 1e-3) {
      // std::cout << "Old state:\n"
      //           << eig_old_state << "\nNew State:\n"
      //           << eig_new_state << "\n";
      return false;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_based_planning");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group =
      kinematic_model->getJointModelGroup("panda_arm");

  std::vector<double> default_joint_state{0, -0.78, 0, -2.35, 0, 1.57, 0.785};
  kinematic_state->setJointGroupPositions(joint_model_group,
                                          default_joint_state);

  // const std::vector<std::string> &joint_names =
  //     joint_model_group->getVariableNames();

  // std::vector<double> goal_joint_state_found{
  //     2.55424, -0.0783369, -2.4615, -2.35386, 0.0215131, 2.35256, 0.0805782};

  // kinematic_state->setJointGroupPositions("panda_arm",
  // goal_joint_state_found); kinematic_state->update(true); auto pose_eig =
  // kinematic_state->getFrameTransform("panda_link8"); std::cout <<
  // "\n\n\n\n\n\n\nCalculated Goal IK:\n"
  //           << pose_eig.translation() << "\n";

  ros::Duration(2.0).sleep();

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.setRobotStateTopic("/display_robot_state");
  visual_tools.publishRobotState(kinematic_state);
  visual_tools.trigger();

  std::queue<ap_planning::APPlanningRequest> planning_queue;
  ap_planning::APPlanningRequest single_request;
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
  single_request.screw_msg.pitch = 0.1;
  planning_queue.push(single_request);

  single_request.screw_msg.origin = geometry_msgs::Point();
  planning_queue.push(single_request);

  single_request.screw_msg.origin = single_request.start_pose.position;
  single_request.screw_msg.axis.x = -1;
  single_request.screw_msg.axis.z = 1;
  single_request.screw_msg.is_pure_translation = true;
  planning_queue.push(single_request);

  size_t success_count = 0;

  // Plan each screw request
  while (planning_queue.size() > 0 && ros::ok()) {
    auto req = planning_queue.front();
    planning_queue.pop();
    success_count = 0;
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start the demo");

    show_screw(req.screw_msg, visual_tools);

    // We need to transform the screw to be in the starting frame
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "panda_link0";
    tf_msg.child_frame_id = "panda_link8";
    tf_msg.transform.rotation = req.start_pose.orientation;
    tf_msg.transform.translation.x = req.start_pose.position.x;
    tf_msg.transform.translation.y = req.start_pose.position.y;
    tf_msg.transform.translation.z = req.start_pose.position.z;
    auto transformed_screw =
        affordance_primitives::transformScrew(req.screw_msg, tf_msg);

    // Find goal pose (in planning frame)
    affordance_primitives::ScrewAxis screw_axis;
    screw_axis.setScrewAxis(transformed_screw);
    const Eigen::Isometry3d planning_to_start = tf2::transformToEigen(tf_msg);
    Eigen::Isometry3d goal_pose =
        planning_to_start * screw_axis.getTF(req.theta);

    // generate start configs
    kinematic_state->setToRandomPositions();
    std::vector<std::vector<double>> start_configs, goal_configs;
    bool found_ik;
    for (size_t i = 0; i < 5; ++i) {
      std::vector<double> joint_values;
      found_ik = kinematic_state->setFromIK(joint_model_group, req.start_pose);
      if (found_ik) {
        kinematic_state->copyJointGroupPositions(joint_model_group,
                                                 joint_values);

        if (checkDuplicateState(start_configs, joint_values)) {
          start_configs.push_back(joint_values);
        }
      }

      found_ik =
          kinematic_state->setFromIK(joint_model_group, tf2::toMsg(goal_pose));
      if (found_ik) {
        // check to see if duplicates?
        kinematic_state->copyJointGroupPositions(joint_model_group,
                                                 joint_values);

        if (checkDuplicateState(goal_configs, joint_values)) {
          goal_configs.push_back(joint_values);
        }
      }
      kinematic_state->setToRandomPositions();
    }

    // generate additional goal configs
    for (size_t i = 0; i < 5; ++i) {
      std::vector<double> joint_values;
      found_ik =
          kinematic_state->setFromIK(joint_model_group, tf2::toMsg(goal_pose));
      if (found_ik) {
        // check to see if duplicates?
        kinematic_state->copyJointGroupPositions(joint_model_group,
                                                 joint_values);

        if (checkDuplicateState(goal_configs, joint_values)) {
          goal_configs.push_back(joint_values);
        }
      }
      kinematic_state->setToRandomPositions();
    }

    double total_success_time = 0.0;
    double max_found_time = 0.0;

    std::cout << "\nUsing my sampler:" << std::endl;
    for (size_t i = 0; i < 5; ++i) {
      auto planner_out = plan(req, start_configs, goal_configs);
      auto solution = planner_out.first;
      if (!solutionIsValid(solution, req)) {
        continue;
      }
      total_success_time += planner_out.second;
      max_found_time = std::max(max_found_time, planner_out.second);
      ++success_count;
      solution.interpolate();

      moveit_msgs::DisplayTrajectory joint_traj;
      joint_traj.model_id = "panda";
      joint_traj.trajectory.push_back(moveit_msgs::RobotTrajectory());

      moveit_msgs::RobotState start_msg;
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
    ROS_INFO_STREAM("Num success: "
                    << success_count
                    << "\nWith #start = " << start_configs.size()
                    << "\n#end = " << goal_configs.size()
                    << "\nAvg time = " << total_success_time / success_count
                    << "\nMax found time = " << max_found_time);
  }

  ros::shutdown();
  return 0;
}
