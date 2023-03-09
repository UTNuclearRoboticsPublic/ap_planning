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
#include <affordance_primitives/screw_planning/chained_screws.hpp>
#include <ap_planning/ap_planning.hpp>

#include <ros/console.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void show_multi_screw(const ap_planning::APPlanningRequest &req,
                      moveit_visual_tools::MoveItVisualTools &visual_tools) {
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Get constraint
  auto constraint = req.toConstraint();

  visual_tools.publishAxis(constraint->referenceFrame());

  // Plot each screw
  const auto viz_screws = constraint->getVisualScrews();
  for (const auto &screw : viz_screws) {
    // Create points for plotting
    Eigen::Vector3d origin, axis;
    tf2::fromMsg(screw.origin, origin);
    tf2::fromMsg(screw.axis, axis);
    Eigen::Vector3d end = origin + 0.2 * axis.normalized();
    geometry_msgs::Point end_point = tf2::toMsg(end);
    geometry_msgs::Point origin_point = tf2::toMsg(origin);

    // Plot
    auto color = screw.is_pure_translation ? rviz_visual_tools::PURPLE
                                           : rviz_visual_tools::ORANGE;
    visual_tools.publishArrow(origin_point, end_point, color,
                              rviz_visual_tools::LARGE);
  }
  visual_tools.trigger();
}

void show_trajectory(const trajectory_msgs::JointTrajectory &traj,
                     moveit_visual_tools::MoveItVisualTools &visual_tools) {
  moveit_msgs::DisplayTrajectory joint_traj;
  joint_traj.model_id = "panda";
  joint_traj.trajectory.push_back(moveit_msgs::RobotTrajectory());
  joint_traj.trajectory.at(0).joint_trajectory = traj;

  moveit_msgs::RobotState start_msg;
  start_msg.joint_state.name = traj.joint_names;
  auto first_waypoint = traj.points.at(0);
  start_msg.joint_state.position = first_waypoint.positions;
  joint_traj.trajectory_start = start_msg;

  joint_traj.trajectory.at(0).joint_trajectory.header.frame_id = "panda_link0";

  int time = 0;

  for (auto &wp : joint_traj.trajectory.at(0).joint_trajectory.points) {
    wp.time_from_start.sec = time;
    ++time;
  }

  visual_tools.publishTrajectoryPath(joint_traj);
}

std::queue<ap_planning::APPlanningRequest> get_planning_queue(
    const std::vector<double> &default_joint_state) {
  std::queue<ap_planning::APPlanningRequest> planning_queue;
  ap_planning::APPlanningRequest single_request;
  ap_planning::ScrewSegment screw_0, screw_1, screw_2, screw_3;
  screw_0.screw_msg.header.frame_id = "panda_link0";
  screw_1.screw_msg.header.frame_id = "panda_link0";
  screw_2.screw_msg.header.frame_id = "panda_link0";

  single_request.ee_frame_name = "panda_link8";
  single_request.planning_time = 10;
  single_request.screw_path_type = ap_planning::ScrewPathType::UNCHAINED;

  // For now, all requests start at same point
  single_request.start_pose.pose.position.x = 0.5;
  single_request.start_pose.pose.position.z = 0.4;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  // Add some test cases
  // Case 1: Linear screws spanning x-y plane
  screw_0.start_theta = 0.0;
  screw_0.end_theta = 0.2;
  screw_0.screw_msg.is_pure_translation = true;
  screw_0.screw_msg.origin = single_request.start_pose.pose.position;
  screw_0.screw_msg.axis.x = 1;

  screw_1.start_theta = 0.0;
  screw_1.end_theta = 0.2;
  screw_1.screw_msg.axis.y = 1;
  screw_1.screw_msg.is_pure_translation = true;
  screw_1.screw_msg.origin = single_request.start_pose.pose.position;

  single_request.screw_path.push_back(screw_0);
  single_request.screw_path.push_back(screw_1);
  planning_queue.push(single_request);

  // Case 2: Linear screws spanning x-z plane
  single_request.screw_path.clear();
  single_request.start_pose.pose.position.x = 0.5;
  single_request.start_pose.pose.position.z = 0.3;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  screw_0.start_theta = 0.0;
  screw_0.end_theta = 0.2;
  screw_0.screw_msg.is_pure_translation = true;
  screw_0.screw_msg.origin = single_request.start_pose.pose.position;
  screw_0.screw_msg.axis.x = 1;

  single_request.start_pose.pose.position.x = 0.5;
  single_request.start_pose.pose.position.z = 0.3;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  screw_1.start_theta = 0.0;
  screw_1.end_theta = 0.2;
  screw_1.screw_msg.axis.y = 0;  // reset the setting from Case 1
  screw_1.screw_msg.axis.z = 1;
  screw_1.screw_msg.is_pure_translation = true;
  screw_1.screw_msg.origin = single_request.start_pose.pose.position;

  single_request.screw_path.push_back(screw_0);
  single_request.screw_path.push_back(screw_1);
  planning_queue.push(single_request);

  // Case 3: Concentric circular screws
  single_request.start_pose.pose.position.x = 0.3;
  single_request.start_pose.pose.position.z = 0.4;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;
  single_request.screw_path.clear();

  screw_0.start_theta = 0.0;
  screw_0.end_theta = 0.5 * M_PI;
  screw_0.screw_msg.is_pure_translation = false;
  screw_0.screw_msg.axis.x = 0;
  screw_0.screw_msg.axis.z = -1;
  screw_0.screw_msg.origin = single_request.start_pose.pose.position;
  screw_0.screw_msg.origin.y += 0.1;

  screw_1.start_theta = 0.0;
  screw_1.end_theta = 0.5 * M_PI;
  screw_1.screw_msg.is_pure_translation = false;
  screw_1.screw_msg.axis.z = -1;
  screw_1.screw_msg.origin = single_request.start_pose.pose.position;
  screw_1.screw_msg.origin.y += 0.1;

  single_request.screw_path.push_back(screw_0);
  single_request.screw_path.push_back(screw_1);
  planning_queue.push(single_request);

  // Case 4: Circular parallel screws separated by a distance
  single_request.start_pose.pose.position.x = 0.4;
  single_request.start_pose.pose.position.z = 0.4;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;
  single_request.screw_path.clear();

  screw_0.start_theta = 0.0;
  screw_0.end_theta = 0.5 * M_PI;
  screw_0.screw_msg.is_pure_translation = false;
  screw_0.screw_msg.axis.x = 0;
  screw_0.screw_msg.axis.z = -1;
  screw_0.screw_msg.origin = single_request.start_pose.pose.position;
  screw_0.screw_msg.origin.y += 0.1;

  single_request.start_pose.pose.position.x = 0.35;
  single_request.start_pose.pose.position.z = 0.4;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  screw_1.start_theta = 0.0;
  screw_1.end_theta = 0.5 * M_PI;
  screw_1.screw_msg.is_pure_translation = false;
  screw_1.screw_msg.axis.z = -1;
  screw_1.screw_msg.origin = single_request.start_pose.pose.position;
  screw_1.screw_msg.origin.y += 0.1;

  single_request.screw_path.push_back(screw_0);
  single_request.screw_path.push_back(screw_1);
  planning_queue.push(single_request);

  // Case 5: Orthogonal circular screws similar to the unchained door-knob
  // door-hinge case
  single_request.start_pose.pose.position.x = 0.3;
  single_request.start_pose.pose.position.z = 0.4;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;
  single_request.screw_path.clear();

  screw_0.start_theta = 0.0;
  screw_0.end_theta = 0.5 * M_PI;
  screw_0.screw_msg.is_pure_translation = false;
  screw_0.screw_msg.axis.x = 0;
  screw_0.screw_msg.axis.y = 1;
  screw_0.screw_msg.axis.z = 0;
  screw_0.screw_msg.origin = single_request.start_pose.pose.position;
  screw_0.screw_msg.origin.y += 0.0;

  single_request.start_pose.pose.position.x = 0.2;
  single_request.start_pose.pose.position.z = 0.4;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  screw_1.start_theta = 0.0;
  screw_1.end_theta = 0.5 * M_PI;
  screw_1.screw_msg.is_pure_translation = false;
  screw_1.screw_msg.axis.z = 1;
  screw_1.screw_msg.origin = single_request.start_pose.pose.position;
  screw_1.screw_msg.origin.y += 0.1;

  single_request.screw_path.push_back(screw_0);
  single_request.screw_path.push_back(screw_1);
  planning_queue.push(single_request);

  return planning_queue;
}

std::queue<moveit_msgs::CollisionObject> get_collision_objects() {
  // Set up stuff same across all test cases
  std::queue<moveit_msgs::CollisionObject> output;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.operation = collision_object.ADD;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);

  // Case 1 obstacle
  collision_object.id = "screw1_box1";
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.4;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  // Case 2 obstacle
  collision_object.id = "screw1_box1";
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.4;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.25;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  // Case 3 obstacle
  collision_object.id = "screw1_box1";
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.4;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.45;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  // Case 4 obstacle
  collision_object.id = "screw1_box1";
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.4;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0.45;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  output.push(collision_object);
  collision_object.primitives.clear();
  collision_object.primitive_poses.clear();

  // Case 5 obstacle
  collision_object.id = "screw1_box1";
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.2;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

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
  std::vector<double> default_joint_state{0, -0.785, 0,    -2.356,
                                          0, 1.571,  0.785};
  kinematic_state->setJointGroupPositions("panda_arm", default_joint_state);

  int num_sample;
  nh.param<int>(ros::this_node::getName() + "/num_sampling", num_sample, 2);

  bool show_trajectories;
  nh.param<bool>(ros::this_node::getName() + "/show_trajectories",
                 show_trajectories, true);

  bool use_obstacles;
  nh.param<bool>(ros::this_node::getName() + "/add_collision_objects",
                 use_obstacles, true);

  // Add collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto collision_objects = get_collision_objects();

  ros::Duration(2.0).sleep();

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.setRobotStateTopic("/display_robot_state");
  visual_tools.trigger();

  auto planning_queue = get_planning_queue(default_joint_state);

  ap_planning::DSSPlanner ap_planner("panda_arm");

  std::stringstream ss_dssp;
  ss_dssp << "Start of output\n";
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

    show_multi_screw(req, visual_tools);

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
  }

  if (last_plan.joint_trajectory.points.size() > 1) {
    show_trajectory(last_plan.joint_trajectory, visual_tools);
  }

  ss_dssp << "End output\n";
  std::cout << ss_dssp.str();

  ros::shutdown();
  return 0;
}
