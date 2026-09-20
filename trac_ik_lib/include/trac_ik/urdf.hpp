#pragma once
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <string>

namespace TRAC_IK {
// Parse an ancestor-to-descendant chain from URDF XML. Throws invalid_argument
// for malformed models, unsupported joints, or invalid limits.
void loadURDFChain(const std::string& xml, const std::string& base,
                   const std::string& tip, KDL::Chain& chain,
                   KDL::JntArray& lower, KDL::JntArray& upper);
}
