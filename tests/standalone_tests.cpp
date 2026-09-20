#include <trac_ik/trac_ik.hpp>
#include <trac_ik/urdf.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <random>

void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
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
    KDL::Chain chain; KDL::JntArray lo, hi;
    require(parsed.getKDLChain(chain) && parsed.getKDLLimits(lo, hi), "Initialization failed");
    require(chain.getNrOfJoints() == 6 && chain.getNrOfSegments() == 7, "Wrong chain size");
    require(lo(0) == -0.8 && hi(0) == 0.8, "Safety limits missing");
    require(hi(5) == std::numeric_limits<float>::max(), "Continuous limits missing");
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::JntArray q(6), seed(6), out;
    q(0)=0.3; q(1)=-0.2; q(2)=0.4; q(3)=0.2; q(4)=-0.3; q(5)=0.7;
    KDL::Frame goal, actual;
    fk.JntToCart(q, goal);
    const KDL::Rotation rotation = KDL::Rotation::RotX(q(3))*KDL::Rotation::RotY(q(4))*KDL::Rotation::RotZ(q(5));
    const KDL::Frame expected(rotation*KDL::Rotation::RPY(0.1,0.2,0.3), KDL::Vector(q(0),q(1),q(2))+rotation*KDL::Vector(0.1,0.2,0.3));
    require(KDL::Equal(goal, expected, 1e-12), "URDF FK disagrees with analytic transform");
    for (auto mode : {TRAC_IK::Speed, TRAC_IK::Distance, TRAC_IK::Manip1, TRAC_IK::Manip2, TRAC_IK::Manip3}) {
      TRAC_IK::TRAC_IK solver(chain, lo, hi, 0.03, 1e-6, mode);
      require(solver.CartToJnt(seed, goal, out) >= 0, "IK failed");
      fk.JntToCart(out, actual);
      require(KDL::Equal(actual, goal, 1e-5), "IK residual too large");
      for (unsigned i=0; i<6; ++i) require(out(i)>=lo(i)-1e-8 && out(i)<=hi(i)+1e-8, "Joint limit violation");
      std::vector<KDL::JntArray> solutions;
      require(solver.getSolutions(solutions), "Solutions missing");
    }
    // A serial six-revolute-joint arm exercises coupled translational/rotational IK.
    KDL::Chain arm;
    const KDL::Joint::JointType axes[] = {KDL::Joint::RotZ, KDL::Joint::RotY,
      KDL::Joint::RotY, KDL::Joint::RotX, KDL::Joint::RotY, KDL::Joint::RotX};
    for (unsigned i=0; i<6; ++i)
      arm.addSegment(KDL::Segment(KDL::Joint(axes[i]), KDL::Frame(KDL::Vector(0.15,0,0.1))));
    KDL::JntArray arm_lo(6), arm_hi(6), arm_q(6), arm_seed(6), arm_out;
    for (unsigned i=0; i<6; ++i) { arm_lo(i)=-2.5; arm_hi(i)=2.5; }
    TRAC_IK::TRAC_IK arm_solver(arm, arm_lo, arm_hi, 0.05, 1e-6);
    KDL::ChainFkSolverPos_recursive arm_fk(arm);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> sample(-1.5,1.5);
    for (unsigned trial=0; trial<100; ++trial) {
      for (unsigned i=0; i<6; ++i) { arm_q(i)=sample(rng); arm_seed(i)=arm_q(i)+0.1; }
      arm_fk.JntToCart(arm_q, goal);
      require(arm_solver.CartToJnt(arm_seed, goal, arm_out)>=0, "6R arm IK failed");
      arm_fk.JntToCart(arm_out, actual);
      require(KDL::Equal(goal,actual,1e-5), "6R arm FK residual");
      for (unsigned i=0; i<6; ++i) require(arm_out(i)>=-2.5 && arm_out(i)<=2.5, "6R limits");
    }
    // Check URDF joint origin rotation and offset independently of the parser.
    const std::string rotated = R"(<robot name="rotated"><link name="base"/><link name="tip"/>
      <joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
      <origin xyz="0.2 -0.1 0.4" rpy="0.3 -0.2 0.7"/><axis xyz="0 1 0"/>
      <limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)";
    KDL::Chain rotated_chain; KDL::JntArray rotated_lo, rotated_hi, angle(1);
    TRAC_IK::loadURDFChain(rotated,"base","tip",rotated_chain,rotated_lo,rotated_hi);
    angle(0)=0.6;
    KDL::ChainFkSolverPos_recursive(rotated_chain).JntToCart(angle,actual);
    require(KDL::Equal(actual,KDL::Frame(KDL::Rotation::RPY(0.3,-0.2,0.7),KDL::Vector(0.2,-0.1,0.4))*
      KDL::Frame(KDL::Rotation::RotY(0.6)),1e-12), "Rotated URDF joint transform");
    rejects([&]{ KDL::JntArray wrong(1); parsed.setKDLLimits(wrong,hi); });
    require(parsed.CartToJnt(KDL::JntArray(2), goal, out) < 0, "Wrong seed accepted");
    auto begin = std::chrono::steady_clock::now();
    require(parsed.CartToJnt(seed, KDL::Frame(KDL::Vector(10,10,10)), out) < 0, "Unreachable target accepted");
    require(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<1, "Timeout broken");
    rejects([&]{ TRAC_IK::TRAC_IK bad("base","missing",xml); });
    rejects([&]{ TRAC_IK::TRAC_IK bad("tip","base",xml); });
    rejects([&]{ TRAC_IK::TRAC_IK bad("base","tip","bad xml"); });
    rejects([&]{ TRAC_IK::TRAC_IK bad(chain,KDL::JntArray(1),hi); });
    std::cout << "All five solve modes, 100 6R targets, analytic FK, limits, invalid inputs and timeout passed\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
