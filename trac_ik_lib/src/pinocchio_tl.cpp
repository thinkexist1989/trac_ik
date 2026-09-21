/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
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

#include <trac_ik/pinocchio_tl.hpp>
#include <cfloat>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace TRAC_IK
{

ChainIkSolverPos_TL::ChainIkSolverPos_TL(const pinocchio::Model& _model,
                                          const Eigen::VectorXd& _q_min,
                                          const Eigen::VectorXd& _q_max,
                                          pinocchio::FrameIndex _tip_frame_id,
                                          double _maxtime, double _eps,
                                          bool _random_restart, bool _try_jl_wrap):
  model(_model), tip_frame_id(_tip_frame_id), q_min(_q_min), q_max(_q_max),
  delta_q(_model.nv), maxtime(_maxtime), eps(_eps), rr(_random_restart), wrap(_try_jl_wrap)
{
  assert(model.nq == _q_min.size());
  assert(model.nq == _q_max.size());

  data.reset(new pinocchio::Data(model));
  reset();

  // Determine joint types
  for (pinocchio::JointIndex i = 1; i < model.joints.size(); i++)
  {
    const auto& joint = model.joints[i];
    if (joint.nq() == 0) continue; // Skip fixed joints

    int idx = joint.idx_q();
    if (joint.shortname() == "JointModelRX" || joint.shortname() == "JointModelRY" ||
        joint.shortname() == "JointModelRZ" || joint.shortname() == "JointModelRUBX" ||
        joint.shortname() == "JointModelRUBY" || joint.shortname() == "JointModelRUBZ")
    {
      if (q_max(idx) >= std::numeric_limits<float>::max() &&
          q_min(idx) <= std::numeric_limits<float>::lowest())
        types.push_back(Continuous);
      else
        types.push_back(RotJoint);
    }
    else if (joint.shortname() == "JointModelPX" || joint.shortname() == "JointModelPY" ||
             joint.shortname() == "JointModelPZ")
      types.push_back(TransJoint);
    else
      types.push_back(RotJoint); // Default to rotational
  }

  assert(types.size() == static_cast<size_t>(_q_max.size()));
}

int ChainIkSolverPos_TL::CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                                    Eigen::VectorXd &q_out, const pinocchio::Motion _bounds)
{
  if (aborted)
    return -3;

  auto start_time = std::chrono::steady_clock::now();
  q_out = q_init;
  bounds = _bounds;

  double time_left;

  do
  {
    // Forward kinematics
    pinocchio::forwardKinematics(model, *data, q_out);
    pinocchio::updateFramePlacements(model, *data);
    f = data->oMf[tip_frame_id];

    delta_twist = diffRelative(p_in, f);

    // Apply bounds
    if (std::abs(delta_twist.linear()[0]) <= std::abs(bounds.linear()[0]))
      delta_twist.linear()[0] = 0;
    if (std::abs(delta_twist.linear()[1]) <= std::abs(bounds.linear()[1]))
      delta_twist.linear()[1] = 0;
    if (std::abs(delta_twist.linear()[2]) <= std::abs(bounds.linear()[2]))
      delta_twist.linear()[2] = 0;
    if (std::abs(delta_twist.angular()[0]) <= std::abs(bounds.angular()[0]))
      delta_twist.angular()[0] = 0;
    if (std::abs(delta_twist.angular()[1]) <= std::abs(bounds.angular()[1]))
      delta_twist.angular()[1] = 0;
    if (std::abs(delta_twist.angular()[2]) <= std::abs(bounds.angular()[2]))
      delta_twist.angular()[2] = 0;

    if (isMotionZero(delta_twist, eps))
      return 1;

    // Compute delta_twist in the world frame
    pinocchio::SE3 diff = f.actInv(p_in);
    delta_twist = pinocchio::log6(diff);

    // Compute Jacobian
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, *data, q_out, tip_frame_id,
                                     pinocchio::LOCAL_WORLD_ALIGNED, J);

    // Solve for delta_q using pseudo-inverse
    Eigen::VectorXd twist_vec(6);
    twist_vec << delta_twist.linear(), delta_twist.angular();

    delta_q = J.completeOrthogonalDecomposition().solve(twist_vec);

    // Use pinocchio::integrate for proper manifold update
    Eigen::VectorXd q_curr(model.nq);
    pinocchio::integrate(model, q_out, delta_q, q_curr);

    // Apply joint limits
    for (int j = 0; j < q_min.size(); j++)
    {
      if (types[j] == Continuous)
        continue;

      if (q_curr(j) < q_min(j))
      {
        if (!wrap || types[j] == TransJoint)
          q_curr(j) = q_min(j);
        else
        {
          double diffangle = fmod(q_min(j) - q_curr(j), 2 * M_PI);
          double curr_angle = q_min(j) - diffangle + 2 * M_PI;
          if (curr_angle > q_max(j))
            q_curr(j) = q_min(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    for (int j = 0; j < q_max.size(); j++)
    {
      if (types[j] == Continuous)
        continue;

      if (q_curr(j) > q_max(j))
      {
        if (!wrap || types[j] == TransJoint)
          q_curr(j) = q_max(j);
        else
        {
          double diffangle = fmod(q_curr(j) - q_max(j), 2 * M_PI);
          double curr_angle = q_max(j) + diffangle - 2 * M_PI;
          if (curr_angle < q_min(j))
            q_curr(j) = q_max(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    Eigen::VectorXd diff_q = q_out - q_curr;

    if (diff_q.isZero(FLT_EPSILON))
    {
      if (rr)
      {
        for (int j = 0; j < q_out.size(); j++)
          if (types[j] == Continuous)
            q_curr(j) = fRand(q_curr(j) - 2 * M_PI, q_curr(j) + 2 * M_PI);
          else
            q_curr(j) = fRand(q_min(j), q_max(j));
      }
    }

    q_out = q_curr;

    auto timediff = std::chrono::steady_clock::now() - start_time;
    time_left = maxtime - std::chrono::duration<double>(timediff).count();
  }
  while (time_left > 0 && !aborted);

  return -3;
}

ChainIkSolverPos_TL::~ChainIkSolverPos_TL()
{
}

}
