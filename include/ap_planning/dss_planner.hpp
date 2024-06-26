///////////////////////////////////////////////////////////////////////////////
//      Title     : dss_planner.hpp
//      Project   : ap_planning
//      Created   : 08/23/2022
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

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <affordance_primitives/screw_planning/screw_constraint.hpp>
#include <ap_planning/ap_planning_common.hpp>
#include <ap_planning/state_sampling.hpp>
#include <ap_planning/state_utils.hpp>

#include <optional>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ap_planning {
/**
 * Plans a motion following a screw path. Direct Screw Sampling Planner
 */
class DSSPlanner {
 public:
  DSSPlanner(const std::string& move_group_name,
             const std::string& robot_description_name = "robot_description");
  ~DSSPlanner();

  /** Attempts to plan a screw-based trajectory
   *
   * @param req The planning request
   * @param res The planning response
   * @return Returns the result of the underlaying planner. Note it is possible
   * for the planner to return success but not have reached the goal, so check
   * the response
   */
  ap_planning::Result plan(const APPlanningRequest& req,
                           APPlanningResponse& res);

 protected:
  ompl::base::StateSpacePtr state_space_;
  ompl::geometric::SimpleSetupPtr ss_;
  Eigen::Isometry3d start_pose_, goal_pose_;
  std::shared_ptr<affordance_primitives::ScrewConstraint> constraints_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  std::shared_ptr<moveit::core::JointModelGroup> joint_model_group_;
  kinematics::KinematicsBasePtr ik_solver_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::shared_ptr<planning_scene_monitor::LockedPlanningSceneRO>
      planning_scene_;
  bool passed_start_config_;
  std::string robot_description_name_;

  void cleanUp();

  bool setupStateSpace(const APPlanningRequest& req);
  void getStartTF(const APPlanningRequest& req);
  bool setSpaceParameters(const APPlanningRequest& req,
                          ompl::base::StateSpacePtr& space);
  bool setSimpleSetup(const ompl::base::StateSpacePtr& space,
                      const APPlanningRequest& req);

  /** Solves IK for the start and goal configurations
   *
   * @param req The planning request
   * @param num_start Number of starting configurations to generate
   * @param num_goal Number of goal configurations to generate
   * @param start_configs Generated starting configs
   * @param goal_configs Generated goal configs
   * @return True if successful, false otherwise
   */
  bool findStartGoalStates(const APPlanningRequest& req, const size_t num_start,
                           const size_t num_goal,
                           std::vector<std::vector<double>>& start_configs,
                           std::vector<std::vector<double>>& goal_configs);

  /** Sets the start state to the requested and solves IK for goal states
   *
   * @param req The planning request
   * @param num_goal Number of goal configurations to generate
   * @param start_configs Generated starting configs
   * @param goal_configs Generated goal configs
   * @return True if successful, false otherwise
   */
  bool findGoalStates(const APPlanningRequest& req, const size_t num_goal,
                      std::vector<std::vector<double>>& start_configs,
                      std::vector<std::vector<double>>& goal_configs);

  /** Given a solution path, this will fill in the planning response
   *
   * Note: it will interpolate the path, with may invalidate an otherwise valid
   * path
   *
   * @param solution The un-interpolated found path
   * @param req The original planning request
   * @param res The response, which will be filled out
   */
  void populateResponse(ompl::geometric::PathGeometric& solution,
                        const APPlanningRequest& req, APPlanningResponse& res);
};

}  // namespace ap_planning
