///////////////////////////////////////////////////////////////////////////////
//      Title     : screw_motion_planner.hpp
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
#include <sample_based_planning/state_sampling.hpp>
#include <sample_based_planning/state_utils.hpp>

#include <optional>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ap_planning {
/**
 * Plans a screw motion using OMPL's PRM
 */
class APMotionPlanner {
 public:
  APMotionPlanner();

  bool plan(const APPlanningRequest& req, APPlanningResponse& res);

 protected:
  ompl::base::StateSpacePtr state_space_;
  ompl::geometric::SimpleSetupPtr ss_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  std::shared_ptr<moveit::core::JointModelGroup> joint_model_group_;

  bool setupStateSpace(const APPlanningRequest& req);
  bool setSpaceParameters(const APPlanningRequest& req,
                          ompl::base::StateSpacePtr& space);
};

}  // namespace ap_planning
