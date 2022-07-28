
///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_planning.hpp
//      Project   : ap_planning
//      Created   : 07/28/2022
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

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ap_closed_chain_planning/ik_solver_base.hpp>

namespace ap_planning {
/**
 * Handles planning an AP with a screw-based primitive
 * This is mostly just holding an "IKSolver" type object, but handles the plugin loading internally
 *
 */
class APPlanner {
 public:
  /** Constructor
   *
   * @param nh ROS node handle
   */
  APPlanner(const ros::NodeHandle& nh, const std::string action_name);

  ~APPlanner(){};

  bool initialize();

  ap_planning::Result plan(
      const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
      const moveit::core::RobotStatePtr& start_state,
      const std::string& ee_name,
      trajectory_msgs::JointTrajectory& joint_trajectory);

 protected:
  // node handle
  ros::NodeHandle nh_;

  // This is the bread and butter
  std::shared_ptr<
      pluginlib::ClassLoader<ap_closed_chain_planning::IKSolverBase>>
      solver_loader_;
  boost::shared_ptr<ap_closed_chain_planning::IKSolverBase> ik_solver_;
};
}  // namespace ap_planning
