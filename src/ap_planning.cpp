#include <tf2_eigen/tf2_eigen.h>
#include <ap_planning/ap_planning_common.hpp>

namespace ap_planning {

affordance_primitives::ChainedScrews APPlanningRequest::toConstraint() const {
  affordance_primitives::ChainedScrews output;

  // Go through screw path and add each one
  for (const auto& segment : screw_path) {
    output.addScrewAxis(segment.screw_msg, segment.start_theta,
                        segment.end_theta);
  }

  // If it looks like the start state wasn't set, then use the passed pose
  if (start_joint_state.size() < 1) {
    Eigen::Isometry3d tf_m_to_s;
    tf2::fromMsg(start_pose.pose, tf_m_to_s);
    output.setReferenceFrame(tf_m_to_s);
  }

  return output;
}

}  // namespace ap_planning
