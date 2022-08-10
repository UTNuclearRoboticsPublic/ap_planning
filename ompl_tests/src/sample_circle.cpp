#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
class MyValidStateSampler : public ob::ValidStateSampler
{
public:
  MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
  {
    name_ = "my sampler";
  }
  // Generate a sample in the valid part of the R^3 state space
  // Valid states satisfy the following constraints:
  // -1<= x,y,z <=1
  // if .25 <= z <= .5, then |x|>.8 and |y|>.8
  bool sample(ob::State *state) override
  {
    double *val = static_cast<ob::RealVectorStateSpace::StateType *>(state)->values;
    double z = rng_.uniformReal(0, 0.5*M_PI);

    val[0] = cos(z);
    val[1] = sin(z);
    val[2] = z;
    assert(si_->isValid(state));
    return true;
  }
  // We don't need this in the example below.
  bool sampleNear(ob::State * /*state*/, const ob::State * /*near*/, const double /*distance*/) override
  {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
  }

protected:
  ompl::RNG rng_;
};

bool isNear(double a, double b, double tol) {
  return fabs(a-b) < fabs(tol);
}

// this function is needed, even when we can write a sampler like the one
// above, because we need to check path segments for validity
bool isStateValid(const ob::State *state)
{
  const ob::RealVectorStateSpace::StateType &pos = *state->as<ob::RealVectorStateSpace::StateType>();
  
  if (pos[2] > 0.5*M_PI || pos[2] < 0) {
    return false;
  }

  if (!isNear(pos[0], cos(pos[2]), 0.001)) {
    return false;
  }

  if (!isNear(pos[1], sin(pos[2]), 0.001)) {
    return false;
  }
  return true;
}

// return an obstacle-based sampler
ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
  // we can perform any additional setup / configuration of a sampler here,
  // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
  return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

// return an instance of my sampler
ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
  return std::make_shared<MyValidStateSampler>(si);
}

void plan(int samplerIndex)
{
  // construct the state space we are planning in
  auto space(std::make_shared<ob::RealVectorStateSpace>(3));

  // set the bounds
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-2);
  bounds.setHigh(2);
  space->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(isStateValid);

  // create a start state
  ob::ScopedState<> start(space);
  start[0] = 1;
  start[1] = 0;
  start[2] = 0;

  // create a goal state
  ob::ScopedState<> goal(space);
  goal[0] = 0;
  goal[1] = 1;
  goal[2] = 0.5*M_PI;

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  // set sampler (optional; the default is uniform sampling)
  if (samplerIndex == 1)
    // use obstacle-based sampling
    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
  else if (samplerIndex == 2)
    // use my sampler
    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMyValidStateSampler);

  // create a planner for the defined space
  auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // attempt to solve the problem within ten seconds of planning time
  ob::PlannerStatus solved = ss.solve(10.0);
  if (solved)
  {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.getSolutionPath().print(std::cout);
  }
  else
    std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
  std::cout << "Using default uniform sampler:" << std::endl;
  plan(0);
  std::cout << "\nUsing obstacle-based sampler:" << std::endl;
  plan(1);
  std::cout << "\nUsing my sampler:" << std::endl;
  plan(2);

  return 0;
}
