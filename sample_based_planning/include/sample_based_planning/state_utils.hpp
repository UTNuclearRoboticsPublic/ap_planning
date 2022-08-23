///////////////////////////////////////////////////////////////////////////////
//      Title     : state_utils.hpp
//      Project   : ap_planning
//      Created   : 08/19/2022
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

// TODO: clean these up
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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ap_planning {
struct APPlanningRequest {
  affordance_primitive_msgs::ScrewStamped screw_msg;
  double theta;
  geometry_msgs::PoseStamped start_pose;
};

class ScrewParam : public ob::GenericParam {
 public:
  ScrewParam(std::string name) : GenericParam(name) {}

  std::string getValue() const override {
    return affordance_primitives::screwMsgToStr(screw_msg_);
  }

  bool setValue(const std::string &value) {
    auto decoded = affordance_primitives::strToScrewMsg(value);
    if (decoded) {
      screw_msg_ = *decoded;
      return true;
    }
    return false;
  }

 protected:
  affordance_primitive_msgs::ScrewStamped screw_msg_;
};

class PoseParam : public ob::GenericParam {
 public:
  PoseParam(std::string name) : GenericParam(name) {}

  std::string getValue() const override {
    return affordance_primitives::poseToStr(pose_msg_);
  }

  bool setValue(const std::string &value) {
    auto decoded = affordance_primitives::strToPose(value);
    if (decoded) {
      pose_msg_ = *decoded;
      return true;
    }
    return false;
  }

 protected:
  geometry_msgs::PoseStamped pose_msg_;
};

class ScrewGoal : public ob::GoalStates {
 public:
  ScrewGoal(const ob::SpaceInformationPtr si);

  double distanceGoal(const ob::State *state) const override;

 protected:
  ob::RealVectorBounds screw_bounds_;
};

// TODO: this must be thread safe, is it?
class ScrewValidityChecker : public ob::StateValidityChecker {
 public:
  ScrewValidityChecker(const ob::SpaceInformationPtr &si);

  virtual bool isValid(const ob::State *state) const;

 protected:
  ob::RealVectorBounds robot_bounds_;
  ob::RealVectorBounds screw_bounds_;
  // TODO: make kinematic model static so less reading params
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  moveit::core::JointModelGroupPtr joint_model_group_;
  affordance_primitives::ScrewAxis screw_axis_;
  Eigen::Isometry3d start_pose_;
};

}  // namespace ap_planning
