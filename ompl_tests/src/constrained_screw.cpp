#include "ompl_tests/ConstrainedPlanningCommon.h"
#include <affordance_primitives/screw_planning/screw_planning.hpp>

class ScrewConstraint : public ob::Constraint
{
public:
  // Space: [x, y, theta]
  ScrewConstraint() : ob::Constraint(2, 1)
  {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.orientation.w = 1;
    pose_msg.position.y = 5;

    Eigen::Vector3d axis(0, 0, 1);
    screw_axis_.setScrewAxis(pose_msg, axis);

    start_pose_.setIdentity();
  }

  void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    // Create tf from state
    Eigen::Isometry3d current_state;
    current_state.setIdentity();
    current_state.translation().x() = x[0];
    current_state.translation().y() = x[1];
    Eigen::AngleAxisd aa(x[3], Eigen::Vector3d(0, 0, 1));
    current_state.linear() = aa.toRotationMatrix();

    // Find TF q -> starting
    const auto tf_q_to_starting = current_state.inverse() * start_pose_;

    // Find the closest point on the path
    const auto closest_pt = affordance_primitives::findClosestPoint(tf_q_to_starting, theta_0_, theta_max_, screw_axis_);

    // Use closest point to calculate error
    auto error = affordance_primitives::calcError(closest_pt.second);
    std::cout << closest_pt.first << "\n";

    Eigen::VectorXd output(3);
    output[0] = error[0];
    output[1] = error[1];
    // output[2] = error[5];

    out = output;
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    Eigen::Vector3d err;
    function(x, err);

    out = err.transpose().normalized();
  }

private:
  affordance_primitives::ScrewAxis screw_axis_;
  Eigen::Isometry3d start_pose_;
  double theta_max_ = 0.5 * M_PI;
  double theta_0_ = 0.5 * theta_max_;
};

class ScrewProjection : public ob::ProjectionEvaluator
{
public:
  ScrewProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
  {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.orientation.w = 1;
    pose_msg.position.y = 5;

    Eigen::Vector3d axis(0, 0, 1);
    screw_axis_.setScrewAxis(pose_msg, axis);
  }

  unsigned int getDimension() const override
  {
    return 1;
  }

  void defaultCellSizes() override
  {
    cellSizes_.resize(1);
    cellSizes_[0] = 0.02;
    // cellSizes_[1] = 0.1;
  }

  void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
  {
    auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
    // projection(0) = x[0];
    // projection(1) = x[1];
    // Create tf from state
    Eigen::Isometry3d current_state;
    current_state.setIdentity();
    current_state.translation().x() = x[0];
    current_state.translation().y() = x[1];
    Eigen::AngleAxisd aa(x[3], Eigen::Vector3d(0, 0, 1));
    current_state.linear() = aa.toRotationMatrix();

    // Find TF q -> starting
    const auto tf_q_to_starting = current_state.inverse() * start_pose_;

    // Find the closest point on the path
    const auto closest_pt = affordance_primitives::findClosestPoint(tf_q_to_starting, theta_0_, theta_max_, screw_axis_);
    projection(0) = closest_pt.first;
  }

private:
  affordance_primitives::ScrewAxis screw_axis_;
  Eigen::Isometry3d start_pose_;
  double theta_max_ = 0.5 * M_PI;
  double theta_0_ = 0.5 * theta_max_;
};

bool obstacles(const ob::State *state)
{
  // auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

  // if (-0.80 < x[2] && x[2] < -0.6)
  // {
  //   if (-0.05 < x[1] && x[1] < 0.05)
  //     return x[0] > 0;
  //   return false;
  // }
  // else if (-0.1 < x[2] && x[2] < 0.1)
  // {
  //   if (-0.05 < x[0] && x[0] < 0.05)
  //     return x[1] < 0;
  //   return false;
  // }
  // else if (0.6 < x[2] && x[2] < 0.80)
  // {
  //   if (-0.05 < x[1] && x[1] < 0.05)
  //     return x[0] < 0;
  //   return false;
  // }

  return true;
}

bool screwPlanningOnce(ConstrainedProblem &cp, enum PLANNER_TYPE planner, bool output)
{
  cp.setPlanner(planner, "screw");

  // Solve the problem
  ob::PlannerStatus stat = cp.solveOnce(output, "screw");

  // if (output)
  // {
  OMPL_INFORM("Dumping problem information to `screw_info.txt`.");
  std::ofstream infofile("screw_info.txt");
  infofile << cp.type << std::endl;
  infofile.close();
  // }

  cp.atlasStats();

  if (output)
    cp.dumpGraph("screw");

  return stat;
}

bool screwPlanningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
  cp.setupBenchmark(planners, "screw");
  cp.runBenchmark();
  return false;
}

bool screwPlanning(bool output, enum SPACE_TYPE space, std::vector<enum PLANNER_TYPE> &planners,
                   struct ConstrainedOptions &c_opt, struct AtlasOptions &a_opt, bool bench)
{
  // Create the ambient space state space for the problem.
  auto rvss = std::make_shared<ob::RealVectorStateSpace>(2);

  ob::RealVectorBounds bounds(2);
  bounds.setLow(-12);
  bounds.setHigh(12);

  rvss->setBounds(bounds);

  // Create a shared pointer to our constraint.
  auto constraint = std::make_shared<ScrewConstraint>();

  ConstrainedProblem cp(space, rvss, constraint);
  cp.setConstrainedOptions(c_opt);
  cp.setAtlasOptions(a_opt);

  cp.css->registerProjection("screw", std::make_shared<ScrewProjection>(cp.css));

  Eigen::VectorXd start(2), goal(2);
  start << 0, 0;
  goal << 0, 10;

  cp.setStartAndGoalStates(start, goal);
  cp.ss->setStateValidityChecker(obstacles);

  if (!bench)
    return screwPlanningOnce(cp, planners[0], output);
  else
    return screwPlanningBench(cp, planners);
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
                  "`screw_path.txt` and `screw_graph.graphml` respectively.";
auto bench_msg = "Do benchmarking on provided planner list.";

int main(int argc, char **argv)
{
  bool output, bench;
  enum SPACE_TYPE space = PJ;
  std::vector<enum PLANNER_TYPE> planners = {PRM};

  struct ConstrainedOptions c_opt;
  struct AtlasOptions a_opt;

  po::options_description desc("Options");
  desc.add_options()("help,h", help_msg);
  desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
  desc.add_options()("bench", po::bool_switch(&bench)->default_value(false), bench_msg);

  addSpaceOption(desc, &space);
  addPlannerOption(desc, &planners);
  addConstrainedOptions(desc, &c_opt);
  addAtlasOptions(desc, &a_opt);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help") != 0u)
  {
    std::cout << desc << std::endl;
    return 1;
  }

  return screwPlanning(output, space, planners, c_opt, a_opt, bench);
}
