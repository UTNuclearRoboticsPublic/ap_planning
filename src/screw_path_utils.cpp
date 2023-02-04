#include <tf2_eigen/tf2_eigen.h>
#include <ap_planning/ap_planning_common.hpp>

namespace ap_planning {
std::vector<affordance_primitive_msgs::ScrewStamped> getScrewVisuals(
    const std::vector<ScrewSegment>& path, const Eigen::Isometry3d& tf_m_to_s) {
  std::vector<affordance_primitive_msgs::ScrewStamped> output;
  output.reserve(path.size());

  APPlanningRequest ap_req;
  ap_req.screw_path = path;
  ap_req.start_pose.pose = tf2::toMsg(tf_m_to_s);

  auto constraints = getConstraintInfo(ap_req);
  auto tf_s_to_m = tf_m_to_s.inverse();

  auto phi = constraints.phi_bounds.first;
  Eigen::Isometry3d tf_start_of_this_axis =
      affordance_primitives::getPoseOnPath(constraints, phi);
  for (size_t i = 0; i < path.size(); ++i) {
    // Extract screw info (now in affordance frame)
    Eigen::Vector3d origin, axis;
    tf2::fromMsg(path.at(i).screw_msg.origin, origin);
    tf2::fromMsg(path.at(i).screw_msg.axis, axis);

    // Convert screw to be in frame {S}
    origin = tf_s_to_m.linear() * origin + tf_s_to_m.translation();
    axis = tf_s_to_m.linear() * axis;

    // Find the current start location w.r.t. {S}
    auto tf_s_to_i = tf_s_to_m * tf_start_of_this_axis;

    // Move screw info by that ^ difference
    origin += tf_s_to_i.translation();
    axis = tf_s_to_i.linear() * axis;

    // Put screw info back into frame {M}
    origin = tf_m_to_s.linear() * origin + tf_m_to_s.translation();
    axis = tf_m_to_s.linear() * axis;

    // Put back into a message
    affordance_primitive_msgs::ScrewStamped screw_msg = path.at(i).screw_msg;
    tf2::toMsg(axis, screw_msg.axis);
    screw_msg.origin = tf2::toMsg(origin);
    output.push_back(screw_msg);

    // Update for next step
    phi(i) = path.at(i).end_theta;
    tf_start_of_this_axis =
        affordance_primitives::getPoseOnPath(constraints, phi);
  }

  return output;
}

affordance_primitives::ScrewConstraintInfo getConstraintInfo(
    const APPlanningRequest& req) {
  affordance_primitives::ScrewConstraintInfo output;
  affordance_primitives::ScrewAxis screw_axis;

  // Set start pose
  tf2::fromMsg(req.start_pose.pose, output.tf_m_to_s);

  // Go through the path and input data
  const size_t m = req.screw_path.size();
  output.phi_bounds.first.resize(m);
  output.phi_bounds.second.resize(m);
  output.screw_axis_set.reserve(m);

  for (size_t i = 0; i < m; ++i) {
    const ScrewSegment& segment = req.screw_path.at(i);

    screw_axis.setScrewAxis(segment.screw_msg);
    output.screw_axis_set.push_back(screw_axis);
    output.phi_bounds.first(i) = segment.start_theta;
    output.phi_bounds.second(i) = segment.end_theta;
  }

  return output;
}

}  // namespace ap_planning
