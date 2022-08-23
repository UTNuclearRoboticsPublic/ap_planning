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

// TODO: document this: both start pose and the screw axis should be given in
// the planning frame!!
std::pair<ompl::geometric::PathGeometric,
          std::pair<ob::SpaceInformationPtr, double>>
plan(const ap_planning::APPlanningRequest &req,
     const std::vector<std::vector<double>> &start_configs,
     const std::vector<std::vector<double>> &goal_configs) {
  // construct the state space we are planning in
  auto screw_space(std::make_shared<ob::RealVectorStateSpace>());
  auto joint_space(std::make_shared<ob::RealVectorStateSpace>());

  screw_space->addDimension(0, req.theta);

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
  tf_msg.transform.rotation = req.start_pose.pose.orientation;
  tf_msg.transform.translation.x = req.start_pose.pose.position.x;
  tf_msg.transform.translation.y = req.start_pose.pose.position.y;
  tf_msg.transform.translation.z = req.start_pose.pose.position.z;
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
  pose_param->setValue(affordance_primitives::poseToStr(req.start_pose));
  space->params().add(pose_param);

  // Add robot description and move group parameters
  auto robot_description_param =
      std::make_shared<ap_planning::StringParam>("robot_description");
  robot_description_param->setValue("robot_description");
  auto move_group_param =
      std::make_shared<ap_planning::StringParam>("move_group");
  move_group_param->setValue("panda_arm");
  space->params().add(robot_description_param);
  space->params().add(move_group_param);

  space->setStateSamplerAllocator(ap_planning::allocScrewSampler);
  space->as<ob::CompoundStateSpace>()->lock();
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
      ap_planning::allocScrewValidSampler);

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(5.0);
  if (solved) {
    ss.simplifySolution(5.);
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
    return std::make_pair(ss.getSolutionPath(),
                          std::make_pair(ss.getSpaceInformation(),
                                         ss.getLastPlanComputationTime()));
  } else {
    std::cout << "No solution found" << std::endl;
  }
  return std::make_pair(og::PathGeometric(ss.getSpaceInformation()),
                        std::make_pair(ss.getSpaceInformation(), 0.0));
}

bool solutionIsValid(og::PathGeometric &solution,
                     const ap_planning::APPlanningRequest &req,
                     const ob::SpaceInformationPtr si) {
  if (solution.getStateCount() < 1) {
    return false;
  }

  // Check last waypoint is close to goal
  const ob::CompoundStateSpace::StateType &compound_state =
      *solution.getStates().back()->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType &screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  const double err = fabs(req.theta - screw_state[0]);
  if (err > 0.01) {
    return false;
  }

  // Go through states and make sure each one is valid
  size_t num_waypoints = solution.getStateCount();
  for (size_t i = 0; i < num_waypoints; ++i) {
    if (!si->isValid(solution.getState(i))) {
      return false;
    }
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
  single_request.start_pose.pose.position.x = 0.5;
  single_request.start_pose.pose.position.z = 0.3;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  // Add some test cases
  single_request.theta = 0.25 * M_PI;
  single_request.screw_msg.origin = single_request.start_pose.pose.position;
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

  single_request.screw_msg.origin = single_request.start_pose.pose.position;
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
    tf_msg.transform.rotation = req.start_pose.pose.orientation;
    tf_msg.transform.translation.x = req.start_pose.pose.position.x;
    tf_msg.transform.translation.y = req.start_pose.pose.position.y;
    tf_msg.transform.translation.z = req.start_pose.pose.position.z;
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
      found_ik =
          kinematic_state->setFromIK(joint_model_group, req.start_pose.pose);
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
    size_t num_post_rejections = 0;

    std::cout << "\nUsing my sampler:" << std::endl;
    for (size_t i = 0; i < 5; ++i) {
      auto planner_out = plan(req, start_configs, goal_configs);
      auto solution = planner_out.first;
      if (!solutionIsValid(solution, req, planner_out.second.first)) {
        continue;
      }
      total_success_time += planner_out.second.second;
      max_found_time = std::max(max_found_time, planner_out.second.second);
      ++success_count;
      solution.interpolate();
      if (!solutionIsValid(solution, req, planner_out.second.first)) {
        ++num_post_rejections;
        continue;
      }

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
                    << " (with num post reject = "
                    << num_post_rejections << ")"
                    << "\nWith #start = " << start_configs.size()
                    << "\n#end = " << goal_configs.size()
                    << "\nAvg time = " << total_success_time / success_count
                    << "\nMax found time = " << max_found_time);
  }

  ros::shutdown();
  return 0;
}
