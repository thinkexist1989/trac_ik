#include <pin_ik/urdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>
#include <algorithm>

namespace PIN_IK {

void loadURDFModel(const std::string& urdf_xml,
                   const std::string& base,
                   const std::string& tip,
                   Model& model,
                   Eigen::VectorXd& lower,
                   Eigen::VectorXd& upper,
                   FrameIndex& tip_frame_id) {
  // Build the full model from URDF
  pinocchio::urdf::buildModelFromXML(urdf_xml, model);
  
  // Find base and tip frames (look for BODY frames which represent links)
  if (!model.existFrame(base, pinocchio::BODY))
    throw std::invalid_argument("Base link '" + base + "' not found in URDF");
  if (!model.existFrame(tip, pinocchio::BODY))
    throw std::invalid_argument("Tip link '" + tip + "' not found in URDF");
  
  FrameIndex base_frame_id = model.getFrameId(base, pinocchio::BODY);
  tip_frame_id = model.getFrameId(tip, pinocchio::BODY);

  // Get joint IDs between base and tip
  std::vector<JointIndex> joint_ids;
  JointIndex current_joint = model.frames[tip_frame_id].parentJoint;
  JointIndex base_joint = model.frames[base_frame_id].parentJoint;

  // Traverse up the kinematic tree from tip to base
  while (current_joint > 0) {
    joint_ids.push_back(current_joint);
    // Stop if we reached the base joint
    if (current_joint == base_joint)
      break;
    current_joint = model.parents[current_joint];
  }

  // If base is the root (base_joint == 0), we should have collected all joints from tip to root
  // If base is not the root, the last joint should be base_joint
  if (joint_ids.empty() || (base_joint != 0 && joint_ids.back() != base_joint))
    throw std::invalid_argument("Base is not an ancestor of tip");

  std::reverse(joint_ids.begin(), joint_ids.end());

  // Extract joint limits for the chain
  // Note: We're using the full model, so we need to extract limits only for chain joints
  int chain_dof = 0;
  for (JointIndex jid : joint_ids) {
    chain_dof += model.joints[jid].nq();
  }

  lower.resize(chain_dof);
  upper.resize(chain_dof);

  int dof_idx = 0;
  for (JointIndex jid : joint_ids) {
    int joint_nq = model.joints[jid].nq();
    int joint_idx_q = model.joints[jid].idx_q();
    
    for (int i = 0; i < joint_nq; ++i) {
      lower(dof_idx) = model.lowerPositionLimit(joint_idx_q + i);
      upper(dof_idx) = model.upperPositionLimit(joint_idx_q + i);
      
      // Handle unbounded joints (continuous, etc.)
      if (lower(dof_idx) <= std::numeric_limits<float>::lowest())
        lower(dof_idx) = -std::numeric_limits<double>::infinity();
      if (upper(dof_idx) >= std::numeric_limits<float>::max())
        upper(dof_idx) = std::numeric_limits<double>::infinity();
      
      dof_idx++;
    }
  }
}

} // namespace PIN_IK
