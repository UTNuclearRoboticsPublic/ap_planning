#include <tf2_eigen/tf2_eigen.h>
#include <ap_planning/ap_planning_common.hpp>

namespace ap_planning {

std::unique_ptr<affordance_primitives::ScrewConstraint>
APPlanningRequest::toConstraint() const {
  std::unique_ptr<affordance_primitives::ScrewConstraint> output;
  if (screw_path_type == UNCHAINED) {
    output = std::make_unique<affordance_primitives::UnchainedScrews>();
  } else {
    output = std::make_unique<affordance_primitives::ChainedScrews>();
  }

  // Go through screw path and add each one
  for (const auto& segment : screw_path) {
    if (segment.lower_bound.has_value() && segment.upper_bound.has_value()) {
      output->addScrewAxis(segment.screw_msg, segment.start_theta,
                           segment.end_theta, segment.lower_bound.value(),
                           segment.upper_bound.value());
    } else {
      output->addScrewAxis(segment.screw_msg, segment.start_theta,
                           segment.end_theta);
    }
  }

  // If it looks like the start state wasn't set, then use the passed pose
  if (start_joint_state.size() < 1) {
    Eigen::Isometry3d tf_m_to_s;
    tf2::fromMsg(start_pose.pose, tf_m_to_s);
    output->setReferenceFrame(tf_m_to_s);
  }

  return output;
}

}  // namespace ap_planning
