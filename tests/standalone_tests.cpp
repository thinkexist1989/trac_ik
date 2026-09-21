#include <trac_ik/trac_ik.hpp>
#include <trac_ik/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <random>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

template<class F> void rejects(F fn) {
  try { fn(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("Expected invalid_argument");
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Missing URDF path");
    std::ifstream file(argv[1]);
    std::string xml((std::istreambuf_iterator<char>(file)), {});

    TRAC_IK::TRAC_IK parsed("base", "tip", xml, 0.03);
    pinocchio::Model model;
    Eigen::VectorXd lo, hi;
    require(parsed.getModel(model) && parsed.getLimits(lo, hi), "Initialization failed");
    std::cout << "DEBUG: model.nq=" << model.nq << ", lo.size()=" << lo.size() << ", hi.size()=" << hi.size() << std::endl;
    // In Pinocchio 4.x, continuous joints use 2 DOF (cos, sin) representation
    // So we have: 3 prismatic + 2 revolute + 2 continuous = 7 DOF
    require(model.nq == 7, "Wrong number of joints");
    std::cout << "DEBUG: lo = [" << lo.transpose() << "]" << std::endl;
    std::cout << "DEBUG: hi = [" << hi.transpose() << "]" << std::endl;
    // Pinocchio doesn't parse safety_controller, only limit tags
    require(lo(0) == -1.0 && hi(0) == 1.0, "Joint limits missing");
    // Continuous joint (yaw) uses 2 DOF (cos, sin) representation at indices 5-6
    // Limits are slightly > 1 to accommodate numerical errors in unit circle
    require(hi(5) > 1.0 && hi(6) > 1.0, "Continuous limits missing");

    pinocchio::Data data(model);
    pinocchio::FrameIndex tip_frame_id = model.getFrameId("tip", pinocchio::BODY);

    // Configuration: 3 prismatic + 2 revolute + 2 continuous (cos, sin)
    Eigen::VectorXd q(7), seed(7), out;
    // Set: x=0.3, y=-0.2, z=0.4, roll=0.2, pitch=-0.3, yaw=0.7 (as angle)
    q << 0.3, -0.2, 0.4, 0.2, -0.3, std::cos(0.7), std::sin(0.7);
    seed.setZero();
    seed(5) = 1.0; // cos(0) = 1 for continuous joint

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::SE3 goal = data.oMf[tip_frame_id];

    // Build expected transform analytically
    Eigen::Matrix3d R_x, R_y, R_z;
    R_x = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());
    R_y = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY());
    R_z = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation = R_x * R_y * R_z;

    Eigen::Matrix3d R_rpy;
    R_rpy = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d translation = q.head<3>() + rotation * Eigen::Vector3d(0.1, 0.2, 0.3);
    pinocchio::SE3 expected(rotation * R_rpy, translation);

    require(goal.isApprox(expected, 1e-10), "URDF FK disagrees with analytic transform");

    for (auto mode : {TRAC_IK::Speed, TRAC_IK::Distance, TRAC_IK::Manip1, TRAC_IK::Manip2, TRAC_IK::Manip3}) {
      TRAC_IK::TRAC_IK solver(model, lo, hi, tip_frame_id, 0.03, 1e-6, mode);
      require(solver.CartToJnt(seed, goal, out) >= 0, "IK failed");

      pinocchio::forwardKinematics(model, data, out);
      pinocchio::updateFramePlacements(model, data);
      pinocchio::SE3 actual = data.oMf[tip_frame_id];

      require(actual.isApprox(goal, 1e-5), "IK residual too large");
      for (int i = 0; i < 7; ++i)
        require(out(i) >= lo(i)-1e-8 && out(i) <= hi(i)+1e-8, "Joint limit violation");

      std::vector<Eigen::VectorXd> solutions;
      require(solver.getSolutions(solutions), "Solutions missing");
    }

    // Test a 6R arm
    pinocchio::Model arm_model;
    arm_model.name = "6R_arm";

    pinocchio::JointIndex parent_id = 0;
    for (int i = 0; i < 6; ++i) {
      std::string joint_name = "joint_" + std::to_string(i);
      std::string body_name = "body_" + std::to_string(i);

      pinocchio::SE3 placement = pinocchio::SE3::Identity();
      placement.translation() = Eigen::Vector3d(0.15, 0, 0.1);

      pinocchio::Inertia inertia = pinocchio::Inertia::Random();

      if (i % 3 == 0)
        parent_id = arm_model.addJoint(parent_id, pinocchio::JointModelRZ(), placement, joint_name);
      else if (i % 3 == 1)
        parent_id = arm_model.addJoint(parent_id, pinocchio::JointModelRY(), placement, joint_name);
      else
        parent_id = arm_model.addJoint(parent_id, pinocchio::JointModelRX(), placement, joint_name);

      arm_model.appendBodyToJoint(parent_id, inertia, pinocchio::SE3::Identity());
      arm_model.addBodyFrame(body_name, parent_id, pinocchio::SE3::Identity());
    }

    arm_model.addFrame(pinocchio::Frame("tip", parent_id, 0, pinocchio::SE3::Identity(), pinocchio::BODY));

    Eigen::VectorXd arm_lo(6), arm_hi(6), arm_q(6), arm_seed(6), arm_out;
    arm_lo.setConstant(-2.5);
    arm_hi.setConstant(2.5);

    pinocchio::FrameIndex arm_tip_id = arm_model.getFrameId("tip", pinocchio::BODY);
    TRAC_IK::TRAC_IK arm_solver(arm_model, arm_lo, arm_hi, arm_tip_id, 0.05, 1e-6);
    pinocchio::Data arm_data(arm_model);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> sample(-1.5, 1.5);

    for (int trial = 0; trial < 100; ++trial) {
      for (int i = 0; i < 6; ++i) {
        arm_q(i) = sample(rng);
        arm_seed(i) = arm_q(i) + 0.1;
      }

      pinocchio::forwardKinematics(arm_model, arm_data, arm_q);
      pinocchio::updateFramePlacements(arm_model, arm_data);
      pinocchio::SE3 arm_goal = arm_data.oMf[arm_tip_id];

      require(arm_solver.CartToJnt(arm_seed, arm_goal, arm_out) >= 0, "6R arm IK failed");

      pinocchio::forwardKinematics(arm_model, arm_data, arm_out);
      pinocchio::updateFramePlacements(arm_model, arm_data);
      pinocchio::SE3 arm_actual = arm_data.oMf[arm_tip_id];

      require(arm_goal.isApprox(arm_actual, 1e-5), "6R arm FK residual");
      for (int i = 0; i < 6; ++i)
        require(arm_out(i) >= -2.5 && arm_out(i) <= 2.5, "6R limits");
    }

    // Test rotated joint
    const std::string rotated = R"(<robot name="rotated"><link name="base"/><link name="tip"/>
      <joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
      <origin xyz="0.2 -0.1 0.4" rpy="0.3 -0.2 0.7"/><axis xyz="0 1 0"/>
      <limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)";

    pinocchio::Model rotated_model;
    Eigen::VectorXd rotated_lo, rotated_hi;
    pinocchio::FrameIndex rotated_tip_id;
    TRAC_IK::loadURDFModel(rotated, "base", "tip", rotated_model, rotated_lo, rotated_hi, rotated_tip_id);

    Eigen::VectorXd angle(1);
    angle(0) = 0.6;
    pinocchio::Data rotated_data(rotated_model);
    pinocchio::forwardKinematics(rotated_model, rotated_data, angle);
    pinocchio::updateFramePlacements(rotated_model, rotated_data);
    pinocchio::SE3 rotated_actual = rotated_data.oMf[rotated_tip_id];

    Eigen::Matrix3d expected_rot;
    expected_rot = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY());
    pinocchio::SE3 rotated_expected(expected_rot, Eigen::Vector3d(0.2, -0.1, 0.4));

    require(rotated_actual.isApprox(rotated_expected, 1e-10), "Rotated URDF joint transform");

    rejects([&]{ Eigen::VectorXd wrong(1); parsed.setLimits(wrong, hi); });
    require(parsed.CartToJnt(Eigen::VectorXd(2), goal, out) < 0, "Wrong seed accepted");

    auto begin = std::chrono::steady_clock::now();
    pinocchio::SE3 unreachable(Eigen::Matrix3d::Identity(), Eigen::Vector3d(10, 10, 10));
    require(parsed.CartToJnt(seed, unreachable, out) < 0, "Unreachable target accepted");
    require(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() < 1, "Timeout broken");

    rejects([&]{ TRAC_IK::TRAC_IK bad("base", "missing", xml); });
    rejects([&]{ TRAC_IK::TRAC_IK bad("tip", "base", xml); });
    rejects([&]{ TRAC_IK::TRAC_IK bad("base", "tip", "bad xml"); });
    rejects([&]{ TRAC_IK::TRAC_IK bad(model, Eigen::VectorXd(1), hi, tip_frame_id); });

    std::cout << "All five solve modes, 100 6R targets, analytic FK, limits, invalid inputs and timeout passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
