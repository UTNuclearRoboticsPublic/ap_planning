///////////////////////////////////////////////////////////////////////////////
//      Title     : ik_solver_base.hpp
//      Project   : ap_planning
//      Created   : 07/27/2022
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2022. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>

#include <ros/ros.h>

#include <affordance_primitive_msgs/AffordancePrimitiveAction.h>
#include <affordance_primitive_msgs/AffordanceTrajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ap_planning/ap_planning_common.hpp>

namespace ap_planning {
/**
 * Virtual base class for
 */
class IKSolverBase {
 public:
  virtual bool initialize(const ros::NodeHandle& nh,
                          const std::string& move_group_name,
                          const std::string& robot_description_name) = 0;

  /** Solves 1 IK request and updates the passed robot state and trajectory
   * point
   *
   * @param jmg Valid JointModelGroup
   * @param target_pose The pose to solve for
   * @param ee_frame The frame to move to target_pose
   * @param robot_state The robot state. It is updated so the positions match
   * the solution
   * @param point The trajectory point to fill out
   */
  virtual bool solveIK(
      const std::shared_ptr<moveit::core::JointModelGroup>& jmg,
      const geometry_msgs::Pose& target_pose, const std::string& ee_frame,
      moveit::core::RobotState& robot_state,
      trajectory_msgs::JointTrajectoryPoint& point) = 0;

  /** Verifies that the robot transitioning from A to B is valid
   *
   * @param point_a The start point (joint state)
   * @param point_b The end point (joint state)
   * @param jmg Joint model group for checking velocity limits
   * @param state_b The end robot state
   * @return The result
   */
  virtual ap_planning::Result verifyTransition(
      const trajectory_msgs::JointTrajectoryPoint& point_a,
      const trajectory_msgs::JointTrajectoryPoint& point_b,
      const std::shared_ptr<moveit::core::JointModelGroup>& jmg,
      const moveit::core::RobotState& state_b) = 0;

  /** Plans a joint trajectory based on a screw primitive
   *
   * @param req The planning request
   * @param res The planning response
   * @return The result
   */
  virtual ap_planning::Result plan(const APPlanningRequest& req,
                                   APPlanningResponse& res) = 0;

  /** Plans a joint trajectory based on an affordance trajectory
   *
   *
   * @param affordance_traj The Cartesian trajectory to plan for
   * @param start_state The starting state of the robot
   * @param ee_name The name of the EE link
   * @param res The planning response
   * @return The result
   */
  virtual ap_planning::Result plan(
      const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
      const std::vector<double>& start_state, const std::string& ee_name,
      APPlanningResponse& res) = 0;

  virtual ~IKSolverBase(){};

 protected:
  IKSolverBase(){};
  ros::NodeHandle nh_;
  std::shared_ptr<moveit::core::JointModelGroup> joint_model_group_;
  moveit::core::RobotModelPtr kinematic_model_;
};
}  // namespace ap_planning
