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

#include <sample_based_planning/screw_motion_planner.hpp>

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_based_planning");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Duration(2.0).sleep();

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.setRobotStateTopic("/display_robot_state");
  visual_tools.trigger();

  std::queue<ap_planning::APPlanningRequest> planning_queue;
  ap_planning::APPlanningRequest single_request;
  single_request.screw_msg.header.frame_id = "panda_link0";
  single_request.ee_frame_name = "panda_link8";

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

  ap_planning::APMotionPlanner ap_planner("panda_arm");

  // Plan each screw request
  while (planning_queue.size() > 0 && ros::ok()) {
    auto req = planning_queue.front();
    planning_queue.pop();
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start the demo");

    show_screw(req.screw_msg, visual_tools);

    ap_planning::APPlanningResponse result;
    if (ap_planner.plan(req, result)) {
      std::cout << "\n\n\nSuccess!!\n\n";
    } else {
      std::cout << "\n\n\nFail!!\n\n";
      continue;
    }

    std::cout << "Trajectory is: " << result.percentage_complete * 100
              << "% complete, and has length: " << result.path_length << "\n";

    show_trajectory(result.joint_trajectory, visual_tools);

    // }
    // ROS_INFO_STREAM("Num success: "
    //                 << success_count
    //                 << " (with num post reject = " << num_post_rejections <<
    //                 ")"
    //                 << "\nWith #start = " << start_configs.size()
    //                 << "\n#end = " << goal_configs.size()
    //                 << "\nAvg time = " << total_success_time / success_count
    //                 << "\nMax found time = " << max_found_time);
  }

  ros::shutdown();
  return 0;
}
