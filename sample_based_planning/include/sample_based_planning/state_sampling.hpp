///////////////////////////////////////////////////////////////////////////////
//      Title     : state_sampling.hpp
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
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <affordance_primitives/screw_model/screw_axis.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ap_planning {

class ScrewSampler : public ob::ValidStateSampler {
 public:
  ScrewSampler(const ob::SpaceInformation *si);

  bool sample(ob::State *state) override;
  bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/,
                  const double /*distance*/) override {
    throw ompl::Exception("ScrewSampler::sampleNear", "not implemented");
    return false;
  }

 protected:
  ompl::RNG rng_;
  ob::RealVectorBounds screw_bounds_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  moveit::core::JointModelGroupPtr joint_model_group_;
  affordance_primitives::ScrewAxis screw_axis_;
  Eigen::Isometry3d start_pose_;
};

class MyStateSampler : public ob::StateSampler {
 public:
  MyStateSampler(const ob::StateSpace *state_space);

  void sample(ob::State *state, const std::vector<double> screw_theta);
  void sampleUniform(ob::State *state) override;
  void sampleUniformNear(ob::State *state, const ob::State *near,
                         double distance) override;
  void sampleGaussian(ob::State *state, const ob::State *mean,
                      double stdDev) override;

 protected:
  ompl::RNG rng_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  moveit::core::JointModelGroupPtr joint_model_group_;
  ob::RealVectorBounds screw_bounds_;
  affordance_primitives::ScrewAxis screw_axis_;
  Eigen::Isometry3d start_pose_;
};

}  // namespace ap_planning
