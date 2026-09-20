#include <trac_ik/urdf.hpp>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace TRAC_IK {
void loadURDFChain(const std::string& xml, const std::string& base,
                   const std::string& tip, KDL::Chain& chain,
                   KDL::JntArray& lower, KDL::JntArray& upper) {
  const auto model = urdf::parseURDF(xml);
  if (!model || !model->getLink(base) || !model->getLink(tip))
    throw std::invalid_argument("Invalid URDF or missing base/tip link");
  std::vector<urdf::JointConstSharedPtr> joints;
  auto link = model->getLink(tip);
  while (link->name != base) {
    if (!link->parent_joint) throw std::invalid_argument("Base is not an ancestor of tip");
    joints.push_back(link->parent_joint);
    link = model->getLink(link->parent_joint->parent_link_name);
  }
  std::reverse(joints.begin(), joints.end());
  KDL::Chain result;
  std::vector<double> lo, hi;
  for (const auto& joint : joints) {
    const auto& origin = joint->parent_to_joint_origin_transform;
    KDL::Frame frame(KDL::Rotation::Quaternion(origin.rotation.x, origin.rotation.y,
                    origin.rotation.z, origin.rotation.w),
                    KDL::Vector(origin.position.x, origin.position.y, origin.position.z));
    KDL::Joint kdl_joint(joint->name, KDL::Joint::Fixed);
    if (joint->type != urdf::Joint::FIXED) {
      if (joint->mimic) throw std::invalid_argument("Mimic joints are unsupported");
      if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS && joint->type != urdf::Joint::PRISMATIC)
        throw std::invalid_argument("Unsupported joint type");
      KDL::Vector axis(joint->axis.x, joint->axis.y, joint->axis.z);
      if (axis.Norm() < 1e-12) throw std::invalid_argument("Zero joint axis");
      kdl_joint = KDL::Joint(joint->name, frame.p, frame.M * axis,
          joint->type == urdf::Joint::PRISMATIC ? KDL::Joint::TransAxis : KDL::Joint::RotAxis);
      if (joint->type == urdf::Joint::CONTINUOUS) {
        lo.push_back(std::numeric_limits<float>::lowest());
        hi.push_back(std::numeric_limits<float>::max());
      } else {
        if (!joint->limits) throw std::invalid_argument("Missing joint limits");
        lo.push_back(joint->limits->lower);
        hi.push_back(joint->limits->upper);
        if (joint->safety) {
          lo.back() = std::max(lo.back(), joint->safety->soft_lower_limit);
          hi.back() = std::min(hi.back(), joint->safety->soft_upper_limit);
        }
        if (lo.back() > hi.back()) throw std::invalid_argument("Inverted joint limits");
      }
    }
    result.addSegment(KDL::Segment(joint->child_link_name, kdl_joint, frame));
  }
  if (lo.empty()) throw std::invalid_argument("Chain has no movable joints");
  chain = result;
  lower.resize(lo.size()); upper.resize(hi.size());
  for (unsigned int i = 0; i < lo.size(); ++i) { lower(i) = lo[i]; upper(i) = hi[i]; }
}
}
