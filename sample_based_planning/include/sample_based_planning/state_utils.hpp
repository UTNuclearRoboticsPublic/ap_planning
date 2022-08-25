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

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <tf2_eigen/tf2_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <affordance_primitives/screw_model/screw_axis.hpp>

namespace ob = ompl::base;

namespace ap_planning {
/**
 * A struct for holding all the information needed to make an AP planning
 * request
 */
// TODO: document this: both start pose and the screw axis should be given in
// the planning frame!!
struct APPlanningRequest {
  affordance_primitive_msgs::ScrewStamped screw_msg;
  double theta;
  geometry_msgs::PoseStamped start_pose;
  std::string ee_frame_name;
};

struct APPlanningResponse {
  trajectory_msgs::JointTrajectory joint_trajectory;
  double percentage_complete;
  bool trajectory_is_valid;
  double path_length;
};

/** Checks if a state is close to any in a list of others
 *
 * @param states The list of already generated states
 * @param new_state The new state to check
 * @return True if the state can be added to the list (without duplication),
 * false otherwise
 */
bool checkDuplicateState(const std::vector<std::vector<double>> &states,
                         const std::vector<double> &new_state);

/** Takes vectors and makes a screw/robot hybrid state
 *
 * @param space State space
 * @param screw_state Vector for the screw theta's
 * @param robot_state Vector for the robot joint angles
 * @return The OMPL state
 */
ompl::base::ScopedState<> vectorToState(ompl::base::StateSpacePtr space,
                                        const std::vector<double> &screw_state,
                                        const std::vector<double> &robot_state);

/**
 * Allows passing screw parameters into the planning instance
 */
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

/**
 * Allows passing pose parameters into the planning instance
 */
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

/**
 * Allows passing string parameters into the planning instance
 */
class StringParam : public ob::GenericParam {
 public:
  StringParam(std::string name) : GenericParam(name) {}

  std::string getValue() const override { return value_; }

  bool setValue(const std::string &value) {
    value_ = value;
    return true;
  }

 protected:
  std::string value_;
};

/**
 * Holds a number of goal poses and allows checking if a state satisfies the
 * goal
 */
class ScrewGoal : public ob::GoalStates {
 public:
  ScrewGoal(const ob::SpaceInformationPtr si);

  /**
   * Returns the distance of the state to the goal. This is simple, and only
   * uses the screw space (independent of robot joints)
   */
  double distanceGoal(const ob::State *state) const override;

 protected:
  ob::RealVectorBounds screw_bounds_;
};

/**
 * Checks a state to make sure it is valid. That mostly means checking to make
 * sure the robot state places the EE link on the required screw axis
 */
// TODO: this must be thread safe, is it?
class ScrewValidityChecker : public ob::StateValidityChecker {
 public:
  ScrewValidityChecker(const ob::SpaceInformationPtr &si);

  virtual bool isValid(const ob::State *state) const;

  // Holds the kinematic model
  // NOTE: You must set this before creating instances of this class!
  inline static moveit::core::RobotModelPtr kinematic_model;

 protected:
  ob::RealVectorBounds robot_bounds_;
  ob::RealVectorBounds screw_bounds_;
  moveit::core::RobotStatePtr kinematic_state_;
  moveit::core::JointModelGroupPtr joint_model_group_;
  affordance_primitives::ScrewAxis screw_axis_;
  Eigen::Isometry3d start_pose_;
  std::string ee_frame_name_;
};

}  // namespace ap_planning
