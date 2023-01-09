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

#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
#include <ap_planning/ap_planning.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

void show_trajectory(const trajectory_msgs::JointTrajectory &traj,
                     moveit_visual_tools::MoveItVisualTools &visual_tools) {
  moveit_msgs::DisplayTrajectory joint_traj;
  joint_traj.model_id = "Puma560";
  joint_traj.trajectory.push_back(moveit_msgs::RobotTrajectory());
  joint_traj.trajectory.at(0).joint_trajectory = traj;

  moveit_msgs::RobotState start_msg;
  start_msg.joint_state.name = traj.joint_names;
  auto first_waypoint = traj.points.at(0);
  start_msg.joint_state.position = first_waypoint.positions;
  joint_traj.trajectory_start = start_msg;

  joint_traj.trajectory.at(0).joint_trajectory.header.frame_id = "world";

  int time = 0;

  for (auto &wp : joint_traj.trajectory.at(0).joint_trajectory.points) {
    wp.time_from_start.sec = time;
    ++time;
  }

  visual_tools.publishTrajectoryPath(joint_traj);
}

std::queue<moveit_msgs::CollisionObject> get_collision_objects() {
  // Set up stuff same across all test cases
  std::queue<moveit_msgs::CollisionObject> output;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.operation = collision_object.ADD;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);

  // Screw one
  collision_object.id = "screw1_box1";
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 1.5;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0;

  // Add screw one object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  // Repeat
  collision_object.id = "screw2_box1";
  primitive.dimensions[primitive.BOX_X] = 1;
  primitive.dimensions[primitive.BOX_Y] = 0.75;
  primitive.dimensions[primitive.BOX_Z] = 1;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.25;
  box_pose.position.z = -0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  primitive.dimensions[primitive.BOX_X] = 1;
  primitive.dimensions[primitive.BOX_Y] = 0.75;
  primitive.dimensions[primitive.BOX_Z] = 1;
  box_pose.position.x = 4;
  box_pose.position.y = 0.25;
  box_pose.position.z = -0.5;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);

  primitive.dimensions[primitive.BOX_X] = 0.25;
  primitive.dimensions[primitive.BOX_Y] = 0.25;
  primitive.dimensions[primitive.BOX_Z] = 1;
  box_pose.position.x = 2.2;
  box_pose.position.y = -0.75;
  box_pose.position.z = -0.5;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  collision_object.id = "screw3_box1";
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 1;
  box_pose.position.x = 0.15;
  box_pose.position.y = -0.6;
  box_pose.position.z = -0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  collision_object.id = "screw4_box1";
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 1.5;
  box_pose.position.x = 0.6;
  box_pose.position.y = -1.05;
  box_pose.position.z = 0.1;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 1.5;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.4;
  box_pose.position.z = 0.1;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  return output;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ap_planning");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &kinematic_model =
      robot_model_loader.getModel();

  moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(kinematic_model));
  std::vector<double> default_joint_state{-0.2, 0.2, 0,    0.1, -0.2,
                                          0.5,  0.1, -0.1, 0};
  kinematic_state->setJointGroupPositions("puma_arm_and_base",
                                          default_joint_state);

  int num_sample, num_sps;
  nh.param<int>(ros::this_node::getName() + "/num_sampling", num_sample, 2);
  nh.param<int>(ros::this_node::getName() + "/num_sps", num_sps, 2);

  bool show_trajectories;
  nh.param<bool>(ros::this_node::getName() + "/show_trajectories",
                 show_trajectories, true);

  bool use_obstacles;
  nh.param<bool>(ros::this_node::getName() + "/add_collision_objects",
                 use_obstacles, true);

  // Set planner type
  std::string planner_name;
  nh.param<std::string>(ros::this_node::getName() + "/planner", planner_name,
                        "prm");
  std::transform(planner_name.begin(), planner_name.end(), planner_name.begin(),
                 ::tolower);

  ap_planning::PlannerType planner_type;
  if (planner_name == "prm") {
    planner_type = ap_planning::PlannerType::PRM;
  } else if (planner_name == "prmstar") {
    planner_type = ap_planning::PlannerType::PRMstar;
  } else if (planner_name == "rrt") {
    planner_type = ap_planning::PlannerType::RRT;
  } else if (planner_name == "rrtconnect") {
    planner_type = ap_planning::PlannerType::RRTconnect;
  } else {
    ROS_WARN_STREAM("Unknown planner type: '" << planner_name
                                              << "', using PRM");
    planner_type = ap_planning::PlannerType::PRM;
  }

  // Add collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto collision_objects = get_collision_objects();

  ros::Duration(2.0).sleep();

  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.setRobotStateTopic("/display_robot_state");
  visual_tools.trigger();

  std::queue<ap_planning::APPlanningRequest> planning_queue;
  ap_planning::APPlanningRequest single_request;
  ap_planning::ScrewSegment single_screw;
  single_screw.screw_msg.header.frame_id = "world";
  single_request.screw_path.push_back(single_screw);
  single_request.ee_frame_name = "link7";
  single_request.planning_time = 20;
  single_request.planner = planner_type;

  // For now, all requests start at same point
  single_request.start_pose.pose.position.x = 0.5;
  single_request.start_pose.pose.position.z = 0.3;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  // Add some test cases
  single_request.screw_path.at(0).theta = 0.5 * M_PI;
  single_request.screw_path.at(0).screw_msg.origin =
      single_request.start_pose.pose.position;
  single_request.screw_path.at(0).screw_msg.axis.x = 1;
  planning_queue.push(single_request);

  // A long linear plan
  single_request.screw_path.at(0).theta = 5;
  single_request.screw_path.at(0).screw_msg.is_pure_translation = true;
  single_request.screw_path.at(0).screw_msg.origin.z = 0.1;
  single_request.start_pose.pose.position.z = 0.1;
  planning_queue.push(single_request);

  single_request.screw_path.at(0).screw_msg.is_pure_translation = false;
  single_request.screw_path.at(0).theta = 0.5 * M_PI;
  single_request.screw_path.at(0).screw_msg.axis.x = 0;
  single_request.screw_path.at(0).screw_msg.axis.z = 1;
  single_request.screw_path.at(0).screw_msg.origin.x = 0.3318;
  single_request.screw_path.at(0).screw_msg.origin.y = -0.15;
  single_request.screw_path.at(0).screw_msg.origin.z = 0.16;
  single_request.start_joint_state = default_joint_state;
  planning_queue.push(single_request);

  single_request.start_joint_state.clear();
  single_request.screw_path.at(0).theta = 0.5 * M_PI;
  single_request.screw_path.at(0).screw_msg.axis.x = 0;
  single_request.screw_path.at(0).screw_msg.axis.z = 1;
  single_request.screw_path.at(0).screw_msg.pitch = 0.2;
  planning_queue.push(single_request);

  ap_planning::DSSPlanner ap_planner("puma_arm_and_base");
  ap_planning::SequentialStepPlanner sequential_step_planner(nh);
  if (!sequential_step_planner.initialize()) {
    ROS_ERROR_STREAM("Init failed");
    return EXIT_FAILURE;
  }

  std::stringstream ss_dssp, ss_sps;
  ss_dssp << "Start of output\n";
  ss_sps << "Start of output\n";
  size_t sample = 0;
  ap_planning::APPlanningResponse last_plan;
  bool collision_obj_exists = false;

  // Plan each screw request
  while (planning_queue.size() > 0 && ros::ok()) {
    sample++;
    auto req = planning_queue.front();
    planning_queue.pop();

    if (show_trajectories) {
      visual_tools.prompt(
          "Press 'next' in the RvizVisualToolsGui window to plan next screw");
    }

    if (use_obstacles && !collision_objects.empty()) {
      if (collision_obj_exists) {
        std::vector<std::string> remove_objs;
        remove_objs.push_back(collision_objects.front().id);
        planning_scene_interface.removeCollisionObjects(remove_objs);
        collision_objects.pop();
      }
      std::vector<moveit_msgs::CollisionObject> collision_obj_vec;
      collision_obj_vec.push_back(collision_objects.front());
      planning_scene_interface.addCollisionObjects(collision_obj_vec);
      collision_obj_exists = true;
    }

    show_screw(req.screw_path.at(0).screw_msg, visual_tools);

    for (size_t i = 0; i < num_sample; ++i) {
      std::cout << "Starting i = " << i << "\n";
      ap_planning::APPlanningResponse result;
      auto start = std::chrono::high_resolution_clock::now();
      ap_planning::Result success = ap_planner.plan(req, result);
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      if (success == ap_planning::SUCCESS) {
        std::cout << "\n\n\nDSS planning: Success!!\n\n";
        std::cout << "Trajectory is: " << result.percentage_complete * 100
                  << "% complete, and has length: " << result.path_length
                  << "\n";

        if (show_trajectories) {
          show_trajectory(result.joint_trajectory, visual_tools);
        }
        last_plan = result;
      } else {
        std::cout << "\n\n\nDSS planning: Fail (" << ap_planning::toStr(success)
                  << ")\n\n";
      }

      ss_dssp << sample << ", DSS, " << ap_planning::toStr(success) << ", "
              << result.percentage_complete * 100 << ", " << duration.count()
              << ", " << result.path_length << ",\n";
    }

    // Now move to SPS planner
    if (show_trajectories) {
      visual_tools.prompt(
          "Press 'next' in the RvizVisualToolsGui window to plan again using "
          "SPS planner");
    }

    // Try planning
    for (size_t i = 0; i < num_sps; ++i) {
      ap_planning::APPlanningResponse sps_output;
      auto start = std::chrono::high_resolution_clock::now();
      auto sps_res = sequential_step_planner.plan(req, sps_output);
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      if (sps_res == ap_planning::SUCCESS) {
        std::cout << "\n\n\nSPS planning: Success!!\n\n";
        std::cout << "Trajectory is: " << sps_output.percentage_complete * 100
                  << "% complete, and has length: " << sps_output.path_length
                  << "\n";
        if (show_trajectories) {
          show_trajectory(sps_output.joint_trajectory, visual_tools);
        }
        last_plan = sps_output;
      } else {
        std::cout << "\n\n\nSPS planning: Fail (" << ap_planning::toStr(sps_res)
                  << ")\n\n";
        std::cout << "Trajectory is: " << sps_output.percentage_complete * 100
                  << "% complete, and has length: " << sps_output.path_length
                  << "\n";
      }
      ss_sps << sample << ", SPS, " << ap_planning::toStr(sps_res) << ", "
             << sps_output.percentage_complete * 100 << ", " << duration.count()
             << ",\n";
    }
  }

  if (last_plan.percentage_complete > 0.1) {
    show_trajectory(last_plan.joint_trajectory, visual_tools);
  }

  ss_dssp << "End output\n";
  ss_sps << "End output\n";
  std::cout << ss_dssp.str();
  std::cout << ss_sps.str();

  ros::shutdown();
  return 0;
}
