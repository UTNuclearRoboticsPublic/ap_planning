#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ap_planning/dss_planner.hpp>

namespace ap_planning {
DSSPlanner::DSSPlanner(const std::string& move_group_name,
                       const std::string& robot_description_name) {
  // Load the robot model
  robot_description_name_ = robot_description_name;
  robot_model_loader::RobotModelLoader robot_model_loader(
      robot_description_name);
  kinematic_model_ = robot_model_loader.getModel();

  // Get information about the robot
  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup(move_group_name));

  ik_solver_ = joint_model_group_->getSolverInstance();

  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_description_name_);
  psm_->startSceneMonitor();
  psm_->startStateMonitor();

  // Set kinematic model for classes that will need it
  ScrewSampler::kinematic_model = kinematic_model_;
  ScrewValidityChecker::kinematic_model = kinematic_model_;
  ScrewValidSampler::kinematic_model = kinematic_model_;
}
DSSPlanner::~DSSPlanner() { cleanUp(); }

ap_planning::Result DSSPlanner::plan(const APPlanningRequest& req,
                                     APPlanningResponse& res) {
  // Set response to failing case
  res.joint_trajectory.joint_names.clear();
  res.joint_trajectory.points.clear();
  res.percentage_complete = 0.0;
  res.trajectory_is_valid = false;

  kinematic_state_ = std::make_shared<moveit::core::RobotState>(
      *(psm_->getStateMonitor()->getCurrentState()));

  // Get the planning scene
  psm_->requestPlanningSceneState();

  planning_scene_ =
      std::make_shared<planning_scene_monitor::LockedPlanningSceneRO>(psm_);

  // Set planning scene for classes that will need it
  ScrewSampler::planning_scene = planning_scene_;
  ScrewValidSampler::planning_scene = planning_scene_;
  ScrewValidityChecker::planning_scene = planning_scene_;

  // TODO make general for non-chained case
  constraints_ = std::make_shared<affordance_primitives::ChainedScrews>();

  // Set up the state space for this plan
  if (!setupStateSpace(req)) {
    cleanUp();
    return INITIALIZATION_FAIL;
  }

  // Set the parameters for the state space
  if (!setSpaceParameters(req, state_space_)) {
    cleanUp();
    return INITIALIZATION_FAIL;
  }

  // Set the sampler and lock
  state_space_->setStateSamplerAllocator(ap_planning::allocScrewSampler);
  state_space_->as<ob::CompoundStateSpace>()->lock();

  // Set up... the SimpleSetup
  if (!setSimpleSetup(state_space_, req)) {
    cleanUp();
    return INITIALIZATION_FAIL;
  }

  // Set constraints for classes that will need it
  ScrewSampler::constraints = constraints_;
  ScrewValidSampler::constraints = constraints_;
  ScrewValidityChecker::constraints = constraints_;

  // Create start and goal states
  std::vector<std::vector<double>> start_configs, goal_configs;
  if (passed_start_config_) {
    if (!findGoalStates(req, 30, start_configs, goal_configs)) {
      cleanUp();
      return NO_IK_SOLUTION;
    }
  } else {
    if (!findStartGoalStates(req, 20, 30, start_configs, goal_configs)) {
      cleanUp();
      return NO_IK_SOLUTION;
    }
  }

  // Set the start states
  for (const auto& start_state : start_configs) {
    ss_->addStartState(
        vectorToState(state_space_, constraints_->lowerBounds(), start_state));
  }

  // Create and populate the goal object
  auto goal_obj = std::make_shared<ScrewGoal>(ss_->getSpaceInformation());
  for (const auto& goal_state : goal_configs) {
    goal_obj->addState(
        vectorToState(state_space_, constraints_->upperBounds(), goal_state));
  }
  ss_->setGoal(goal_obj);

  // Plan
  ob::PlannerStatus solved = ss_->solve(req.planning_time);
  ap_planning::Result result = PLANNING_FAIL;
  if (solved) {
    ss_->simplifySolution(1.0);

    populateResponse(ss_->getSolutionPath(), req, res);
    result = SUCCESS;
  }
  cleanUp();
  return result;
}

void DSSPlanner::cleanUp() {
  planning_scene_.reset();
  ScrewSampler::planning_scene.reset();
  ScrewValidSampler::planning_scene.reset();
  ScrewValidityChecker::planning_scene.reset();
  ScrewSampler::constraints.reset();
  ScrewValidSampler::constraints.reset();
  ScrewValidityChecker::constraints.reset();
}

bool DSSPlanner::setupStateSpace(const APPlanningRequest& req) {
  state_space_.reset();

  // construct the state space we are planning in
  auto screw_space = std::make_shared<ob::RealVectorStateSpace>();
  auto joint_space = std::make_shared<ob::RealVectorStateSpace>();

  size_t i = 0;
  // Add screw dimensions
  for (const auto& segment : req.screw_path) {
    if (segment.start_theta > segment.end_theta) {
      return false;
    }
    screw_space->addDimension(segment.start_theta, segment.end_theta);
    constraints_->addScrewAxis(segment.screw_msg, segment.start_theta,
                               segment.end_theta);
  }

  // Go through joint group and add bounds for each joint
  for (const moveit::core::JointModel* joint :
       joint_model_group_->getActiveJointModels()) {
    const auto jt = joint->getType();

    if (jt == joint->PLANAR) {
      joint_space->addDimension(-1e3, 1e3);
      joint_space->addDimension(-1e3, 1e3);
      joint_space->addDimension(-M_PI, M_PI);
    } else if (jt == joint->REVOLUTE || jt == joint->PRISMATIC) {
      const auto& bounds = joint->getVariableBounds(joint->getName());
      if (bounds.position_bounded_) {
        joint_space->addDimension(bounds.min_position_, bounds.max_position_);
      } else {
        // TODO: Add warning or throw?
        return false;
      }
    } else {
      // TODO: Add warning or throw?
    }
  }

  // combine the state space
  state_space_ = screw_space + joint_space;
  return true;
}

bool DSSPlanner::setSpaceParameters(const APPlanningRequest& req,
                                    ompl::base::StateSpacePtr& space) {
  // Handle the starting transformation
  getStartTF(req);

  // Solve the goal pose
  goal_pose_ = constraints_->getPose(constraints_->upperBounds());

  // Add EE frame name and move group parameters
  auto ee_name_param =
      std::make_shared<ap_planning::StringParam>("ee_frame_name");
  ee_name_param->setValue(req.ee_frame_name);
  space->params().add(ee_name_param);
  auto move_group_param =
      std::make_shared<ap_planning::StringParam>("move_group");
  move_group_param->setValue(joint_model_group_->getName());
  space->params().add(move_group_param);

  return true;
}

void DSSPlanner::getStartTF(const APPlanningRequest& req) {
  // Check if a starting joint configuration was given
  if (req.start_joint_state.size() == joint_model_group_->getVariableCount()) {
    // Extract the start pose
    kinematic_state_->setJointGroupPositions(joint_model_group_.get(),
                                             req.start_joint_state);
    kinematic_state_->update(true);
    start_pose_ = kinematic_state_->getFrameTransform(req.ee_frame_name);
    passed_start_config_ = true;
    constraints_->setReferenceFrame(start_pose_);
  } else {
    // Set pose directly
    Eigen::Isometry3d tf_m_to_s;
    tf2::fromMsg(req.start_pose.pose, tf_m_to_s);
    constraints_->setReferenceFrame(tf_m_to_s);

    // Solve start pose manually
    start_pose_ = constraints_->getPose(constraints_->lowerBounds());
    passed_start_config_ = false;
  }
}

bool DSSPlanner::setSimpleSetup(const ompl::base::StateSpacePtr& space,
                                const APPlanningRequest& req) {
  // Create the SimpleSetup class
  ss_ = std::make_shared<og::SimpleSetup>(space);

  // Set state validity checking
  ss_->setStateValidityChecker(
      std::make_shared<ScrewValidityChecker>(ss_->getSpaceInformation()));

  // Set valid state sampler
  ss_->getSpaceInformation()->setValidStateSamplerAllocator(
      ap_planning::allocScrewValidSampler);

  // Set planner
  if (req.planner == PlannerType::PRMstar) {
    auto planner = std::make_shared<og::PRMstar>(ss_->getSpaceInformation());
    ss_->setPlanner(planner);
  } else if (req.planner == PlannerType::RRT) {
    auto planner = std::make_shared<og::RRT>(ss_->getSpaceInformation());
    ss_->setPlanner(planner);
  } else if (req.planner == PlannerType::RRTconnect) {
    auto planner = std::make_shared<og::RRTConnect>(ss_->getSpaceInformation());
    ss_->setPlanner(planner);
  } else {
    auto planner = std::make_shared<og::PRM>(ss_->getSpaceInformation());
    ss_->setPlanner(planner);
  }

  return true;
}

bool DSSPlanner::findStartGoalStates(
    const APPlanningRequest& req, const size_t num_start, const size_t num_goal,
    std::vector<std::vector<double>>& start_configs,
    std::vector<std::vector<double>>& goal_configs) {
  if (num_goal < 1 || num_start < 1) {
    return false;
  }

  start_configs.clear();
  start_configs.reserve(num_start);
  goal_configs.clear();
  goal_configs.reserve(num_goal);

  geometry_msgs::Pose goal_pose_msg = tf2::toMsg(goal_pose_);
  geometry_msgs::Pose start_pose_msg = tf2::toMsg(start_pose_);

  // Go through and make configurations
  size_t i = 0;
  while ((start_configs.size() < num_start || goal_configs.size() < num_goal) &&
         i < 2 * (num_goal + num_start)) {
    // Every time, we set to random states to get variety in solutions
    kinematic_state_->setToRandomPositions(joint_model_group_.get());
    i++;

    // Try to add a start configuration
    if (start_configs.size() < num_start) {
      increaseStateList(joint_model_group_, kinematic_state_, *planning_scene_,
                        ik_solver_, start_pose_msg, start_configs);
    }

    // Try to add a goal configuration
    if (goal_configs.size() < num_goal) {
      increaseStateList(joint_model_group_, kinematic_state_, *planning_scene_,
                        ik_solver_, goal_pose_msg, goal_configs);
    }
  }

  return start_configs.size() > 0 && goal_configs.size() > 0;
}

bool DSSPlanner::findGoalStates(
    const APPlanningRequest& req, const size_t num_goal,
    std::vector<std::vector<double>>& start_configs,
    std::vector<std::vector<double>>& goal_configs) {
  start_configs.clear();
  goal_configs.clear();
  goal_configs.reserve(num_goal);

  if (req.start_joint_state.size() != joint_model_group_->getVariableCount()) {
    return false;
  }
  start_configs.push_back(req.start_joint_state);
  kinematic_state_->setJointGroupPositions(joint_model_group_.get(),
                                           req.start_joint_state);

  geometry_msgs::Pose goal_pose_msg = tf2::toMsg(goal_pose_);

  // Go through and make configurations
  size_t i = 0;
  while (goal_configs.size() < num_goal && i < 2 * num_goal) {
    // Try to add a goal configuration
    increaseStateList(joint_model_group_, kinematic_state_, *planning_scene_,
                      ik_solver_, goal_pose_msg, goal_configs);

    // Every time, we set to random states to get variety in solutions
    kinematic_state_->setToRandomPositions(joint_model_group_.get());
    i++;
  }

  return goal_configs.size() > 0;
}

// void DSSPlanner::increaseStateList(
//     const affordance_primitives::Pose& pose,
//     std::vector<std::vector<double>>& state_list) {
//   // Set up IK callback
//   kinematics::KinematicsBase::IKCallbackFn ik_callback_fn =
//       [this](const geometry_msgs::Pose& pose, const std::vector<double>&
//       joints,
//              moveit_msgs::MoveItErrorCodes& error_code) {
//         ikCallbackFnAdapter(joint_model_group_, kinematic_state_,
//                             *planning_scene_, joints, error_code);
//       };

//   // Try to solve the IK
//   std::vector<double> seed_state, ik_solution;
//   moveit_msgs::MoveItErrorCodes err;
//   kinematics::KinematicsQueryOptions opts;
//   kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
//                                             seed_state);
//   if (!ik_solver_->searchPositionIK(pose, seed_state, 0.05, ik_solution,
//                                     ik_callback_fn, err, opts)) {
//     return;
//   }

//   // If the solution is valid, add it to the list
//   if (checkDuplicateState(state_list, ik_solution)) {
//     state_list.push_back(ik_solution);
//   }
// }

void DSSPlanner::populateResponse(ompl::geometric::PathGeometric& solution,
                                  const APPlanningRequest& req,
                                  APPlanningResponse& res) {
  // We can stop if the trajectory doesn't have points
  if (solution.getStateCount() < 2) {
    return;
  }

  // Interpolate the solution path
  solution.interpolate();

  // We will populate the trajectory
  res.joint_trajectory.joint_names = joint_model_group_->getVariableNames();
  const size_t num_joints = res.joint_trajectory.joint_names.size();
  res.joint_trajectory.points.reserve(solution.getStateCount());

  // TODO make general for non-chained case
  affordance_primitives::ChainedScrews& chained_cons =
      dynamic_cast<affordance_primitives::ChainedScrews&>(*constraints_);

  // Go through each point and check it for validity
  for (const auto& state : solution.getStates()) {
    // Extract the state info
    const ob::CompoundStateSpace::StateType& compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType& screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType& robot_state =
        *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

    // First check for validity
    if (!ss_->getSpaceInformation()->isValid(state)) {
      // If a state is invalid, we don't want to continue the trajectory
      res.trajectory_is_valid = false;

      // Calculate the percent through the trajectory we made it
      std::vector<double> phi(constraints_->size());
      for (size_t i = 0; i < constraints_->size(); ++i) {
        phi[i] = screw_state[i];
      }
      const double lambda = chained_cons.getLambda(phi);

      res.percentage_complete = lambda / chained_cons.lambdaMax();
      return;
    }

    // Now add this state to the trajectory
    trajectory_msgs::JointTrajectoryPoint output;
    output.positions.reserve(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
      output.positions.push_back(robot_state[i]);
    }

    // TODO: figure out what to do about vel / time
    res.joint_trajectory.points.push_back(output);
  }

  // Finally, we check the last point to make sure it is at the goal
  const ob::CompoundStateSpace::StateType& compound_state =
      *solution.getStates().back()->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType& screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();

  std::vector<double> phi(constraints_->size());
  for (size_t i = 0; i < constraints_->size(); ++i) {
    phi[i] = screw_state[i];
  }

  const double lambda = chained_cons.getLambda(phi);
  const double err = chained_cons.lambdaMax() - lambda;
  if (err > 0.01) {
    res.trajectory_is_valid = false;
  } else {
    res.trajectory_is_valid = true;
  }
  res.percentage_complete = lambda / chained_cons.lambdaMax();
  res.path_length = solution.length();
}
}  // namespace ap_planning
