///////////////////////////////////////////////////////////////////////////////
//      Title     : ik_solver.hpp
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

#include <ros/ros.h>
#include <ap_closed_chain_planning/ik_solver_base.hpp>

// #include <affordance_primitives/msg_types.hpp>
// #include <optional>

namespace ap_closed_chain_planning {

/**
 * Estimates the state of a task
 */
class IKSolver : public IKSolverBase {
 public:
  IKSolver(){};
  ~IKSolver(){};

  void initialize(const ros::NodeHandle& nh) override;

  bool solveIK(const moveit::core::JointModelGroup* jmg,
               const geometry_msgs::Pose& target_pose,
               const std::string& ee_frame,
               moveit::core::RobotState& robot_state,
               trajectory_msgs::JointTrajectoryPoint& point) override;

  ap_planning::Result verifyTransition(
      const trajectory_msgs::JointTrajectoryPoint& point_a,
      const trajectory_msgs::JointTrajectoryPoint& point_b) override;

  /** Plans a joint trajectory based on an input Screw Axis
   *
   */
  ap_planning::Result plan(
      const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
      const moveit::core::RobotStatePtr& start_state,
      trajectory_msgs::JointTrajectory& joint_trajectory) override;
};
}  // namespace ap_closed_chain_planning
