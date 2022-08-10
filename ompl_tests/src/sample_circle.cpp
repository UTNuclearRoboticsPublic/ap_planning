#include <ompl/base/SpaceInformation.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ScrewSampler : public ob::ValidStateSampler {
 public:
  ScrewSampler(const ob::SpaceInformation *si) : ValidStateSampler(si) {
    name_ = "my sampler";
  }
  bool sample(ob::State *state) override {
    ob::CompoundStateSpace::StateType &compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
    ob::RealVectorStateSpace::StateType &robot_state =
        *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

    // TODO: make these class members so less redoing the same steps over and
    // over
    ob::CompoundStateSpace *compound_space =
        si_->getStateSpace()->as<ob::CompoundStateSpace>();
    ob::RealVectorBounds screw_bounds = compound_space->getSubspace(0)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();

    for (size_t i = 0; i < screw_bounds.low.size(); ++i) {
      screw_state[i] =
          rng_.uniformReal(screw_bounds.low[i], screw_bounds.high[i]);
    }

    robot_state[0] = cos(screw_state[0]);
    robot_state[1] = sin(screw_state[0]);
    robot_state[2] = screw_state[0];

    assert(si_->isValid(state));
    return true;
  }
  // We don't need this in the example below.
  bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/,
                  const double /*distance*/) override {
    throw ompl::Exception("ScrewSampler::sampleNear", "not implemented");
    return false;
  }

 protected:
  ompl::RNG rng_;
};

bool isNear(double a, double b, double tol = 1e-3) {
  return fabs(a - b) < fabs(tol);
}

class ScrewValidityChecker : public ob::StateValidityChecker {
 public:
  ScrewValidityChecker(const ob::SpaceInformationPtr &si)
      : ob::StateValidityChecker(si) {}

  virtual bool isValid(const ob::State *state) const {
    const ob::CompoundStateSpace::StateType &compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType &robot_state =
        *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

    // TODO: make these class members so less redoing the same steps over and
    // over
    ob::CompoundStateSpace *compound_space =
        si_->getStateSpace()->as<ob::CompoundStateSpace>();
    ob::RealVectorBounds screw_bounds = compound_space->getSubspace(0)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();
    ob::RealVectorBounds robot_bounds = compound_space->getSubspace(1)
                                            ->as<ob::RealVectorStateSpace>()
                                            ->getBounds();

    for (size_t i = 0; i < screw_bounds.low.size(); ++i) {
      if (screw_state[i] > screw_bounds.high[i] ||
          screw_state[i] < screw_bounds.low[i]) {
        return false;
      }
    }

    for (size_t i = 0; i < robot_bounds.low.size(); ++i) {
      if (robot_state[i] > robot_bounds.high[i] ||
          robot_state[i] < robot_bounds.low[i]) {
        return false;
      }
    }

    if (!isNear(robot_state[0], cos(robot_state[2]), 0.001)) {
      return false;
    }

    if (!isNear(robot_state[1], sin(robot_state[2]), 0.001)) {
      return false;
    }

    if (!isNear(robot_state[2], screw_state[0], 0.001)) {
      return false;
    }
    return true;
  }
};

ob::ValidStateSamplerPtr allocOBValidStateSampler(
    const ob::SpaceInformation *si) {
  return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocScrewSampler(const ob::SpaceInformation *si) {
  return std::make_shared<ScrewSampler>(si);
}

void plan(int samplerIndex) {
  // construct the state space we are planning in
  auto screw_space(std::make_shared<ob::RealVectorStateSpace>());
  auto joint_space(std::make_shared<ob::RealVectorStateSpace>());

  screw_space->addDimension(0, 0.5 * M_PI);
  joint_space->addDimension(-2, 2);
  joint_space->addDimension(-2, 2);
  joint_space->addDimension(-2, 2);

  // define a simple setup class
  ompl::base::StateSpacePtr space = screw_space + joint_space;
  // TODO: lock space? space.lock()
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(
      std::make_shared<ScrewValidityChecker>(ss.getSpaceInformation()));

  // create a start state
  ob::ScopedState<> start(space);
  start[0] = 0;
  start[1] = 1;
  start[2] = 0;
  start[3] = 0;

  // create a goal state
  ob::ScopedState<> goal(space);
  goal[0] = 0.5 * M_PI;
  goal[1] = 0;
  goal[2] = 1;
  goal[3] = 0.5 * M_PI;

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  // set sampler (optional; the default is uniform sampling)
  if (samplerIndex == 1)
    // use obstacle-based sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        allocOBValidStateSampler);
  else if (samplerIndex == 2)
    // use my sampler
    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocScrewSampler);

  // create a planner for the defined space
  auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(10.0);
  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
  } else
    std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/) {
  std::cout << "Using default uniform sampler:" << std::endl;
  plan(0);
  std::cout << "\nUsing obstacle-based sampler:" << std::endl;
  plan(1);
  std::cout << "\nUsing my sampler:" << std::endl;
  plan(2);

  return 0;
}
