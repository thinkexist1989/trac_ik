/********************************************************************************
Copyright (c) 2024, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef PINOCCHIO_TYPES_HPP
#define PINOCCHIO_TYPES_HPP

#include <pinocchio/multibody.hpp>
#include <pinocchio/spatial.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace PIN_IK {

// Joint type enumeration for limit handling
enum BasicJointType {
  RotJoint = 0,
  TransJoint = 1,
  Continuous = 2
};

// Type aliases for Pinocchio 4.x
// Default instantiation: double scalar, column-major (Options=0), default joint collection
using Model = pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
using Data = pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
using SE3 = pinocchio::SE3Tpl<double, 0>;
using Motion = pinocchio::MotionTpl<double, 0>;
using Frame = pinocchio::FrameTpl<double, 0>;
using FrameIndex = pinocchio::FrameIndex;
using JointIndex = pinocchio::JointIndex;

// Helper function to compute pose difference for IK
inline Motion diffRelative(const SE3& current, const SE3& target) {
  // Compute the relative transformation error
  SE3 error = target.actInv(current);
  // Convert to motion (log map)
  Motion motion_error = pinocchio::log6(error);
  return motion_error;
}

// Helper function to check if two SE3 transforms are approximately equal
inline bool isApprox(const SE3& a, const SE3& b, double prec = 1e-6) {
  return a.isApprox(b, prec);
}

// Helper function to check if a Motion is approximately zero
inline bool isMotionZero(const Motion& m, double eps = 1e-6) {
  return m.linear().norm() < eps && m.angular().norm() < eps;
}

} // namespace PIN_IK

#endif // PINOCCHIO_TYPES_HPP
