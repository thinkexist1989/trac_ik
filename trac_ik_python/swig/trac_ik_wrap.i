 /* trac_ik_wrap.i */
 %module trac_ik_wrap

// Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
// Updated to use Pinocchio instead of KDL

 %{
 /* Includes the header in the wrapper code */
 #include <trac_ik/trac_ik.hpp>
 #include <pinocchio/spatial/se3.hpp>
 #include <limits>
 %}

 // We need this or we will get on runtime
 // NotImplementedError: Wrong number or type of arguments for overloaded function
 %include <std_string.i>
 %include <std_vector.i>

// From http://stackoverflow.com/a/8752983
// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}

// USEFUL DOCS: http://www.swig.org/Doc1.3/SWIG.html


%include <exception.i>
%exception {
  try { $action }
  catch (const std::invalid_argument& error) { SWIG_exception(SWIG_ValueError, error.what()); }
  catch (const std::exception& error) { SWIG_exception(SWIG_RuntimeError, error.what()); }
}
namespace TRAC_IK {
class TRAC_IK {
public:
  ~TRAC_IK();
};
}

// Create a more Python friendly constructor
%extend TRAC_IK::TRAC_IK {
    // Delegate URDF parsing and initialization to the standalone C++ constructor.
    TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& urdf_string,
      double timeout, double epsilon, const std::string& solve_type="Speed"){

      TRAC_IK::SolveType solvetype;

      if (solve_type == "Manipulation1")
        solvetype = TRAC_IK::Manip1;
      else if (solve_type == "Manipulation2")
        solvetype = TRAC_IK::Manip2;
      else if (solve_type == "Manipulation3")
        solvetype = TRAC_IK::Manip3;
      else if (solve_type == "Distance")
        solvetype = TRAC_IK::Distance;
      else {
          if (solve_type != "Speed") {
              throw std::invalid_argument("Unknown solve_type: " + solve_type);
          }
          solvetype = TRAC_IK::Speed;
      }
          TRAC_IK::TRAC_IK* newX = new TRAC_IK::TRAC_IK(base_link, tip_link, urdf_string, timeout, epsilon, solvetype);
          return newX;
    }


    // original call:
    // int CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in, Eigen::VectorXd &q_out, const pinocchio::Motion& bounds=pinocchio::Motion::Zero());

    // note that as a comment here https://bitbucket.org/traclabs/trac_ik/issues/18/possible-bug-with-quaternions-in-carttojnt
    // explains, the pose is in reference to the base
    // of the chain...
    std::vector<double> CartToJnt(const std::vector<double> q_init,
     const double x, const double y, const double z,
     const double rx, const double ry, const double rz, const double rw,
     // bounds x y z
     const double boundx=0.0, const double boundy=0.0, const double boundz=0.0,
     // bounds on rotation x y z
     const double boundrx=0.0, const double boundry=0.0, const double boundrz=0.0)
    {

      const double norm = std::sqrt(rx*rx + ry*ry + rz*rz + rw*rw);
      if (!std::isfinite(norm) || norm < 1e-12) throw std::invalid_argument("Invalid quaternion");

      // Create SE3 transform from position and quaternion
      Eigen::Quaterniond quat(rw/norm, rx/norm, ry/norm, rz/norm);
      Eigen::Vector3d pos(x, y, z);
      pinocchio::SE3 frame(quat.toRotationMatrix(), pos);

      pinocchio::Model model;
      $self->getModel(model);
      if (q_init.size() != model.nq) throw std::invalid_argument("Wrong seed size");

      Eigen::VectorXd in(q_init.size()), out(q_init.size());

      for (uint z=0; z < q_init.size(); z++)
          in(z) = q_init[z];

      pinocchio::Motion bounds = pinocchio::Motion::Zero();
      bounds.linear()[0] = boundx;
      bounds.linear()[1] = boundy;
      bounds.linear()[2] = boundz;
      bounds.angular()[0] = boundrx;
      bounds.angular()[1] = boundry;
      bounds.angular()[2] = boundrz;

      int rc = $self->CartToJnt(in, frame, out, bounds);
      std::vector<double> vout;
      // If no solution, return empty vector which acts as None
      if (rc < 0)
          return vout;

      for (uint z=0; z < q_init.size(); z++)
          vout.push_back(out(z));

      return vout;
    }

    // Convenience method to check that calls to IK have the correct
    // number of qinit elements
    int getNrOfJointsInChain(){
      pinocchio::Model model;
      $self->getModel(model);
      return (int) model.nq;
    }

    // Convenience method to get the list of joint names as used internally
    std::vector<std::string> getJointNamesInChain(const std::string& urdf_string){
      pinocchio::Model model;
      $self->getModel(model);

      std::vector<std::string> joint_names_;
      for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
        if (model.joints[i].nq() > 0) {
          joint_names_.push_back(model.names[i]);
        }
      }
      return joint_names_;
    }


    // Convenience method to get the list of link names as used internally
    std::vector<std::string> getLinkNamesInChain(){
      pinocchio::Model model;
      $self->getModel(model);

      std::vector<std::string> link_names_;
      for (pinocchio::FrameIndex i = 0; i < model.frames.size(); ++i) {
        if (model.frames[i].type == pinocchio::BODY) {
          link_names_.push_back(model.frames[i].name);
        }
      }
      return link_names_;
    }


    // Get limits
    std::vector<double> getLowerBoundLimits(){
      Eigen::VectorXd lb_;
      Eigen::VectorXd ub_;
      std::vector<double> lb;
      $self->getLimits(lb_, ub_);
      for(int i=0; i < lb_.size(); i++){
        lb.push_back(lb_(i));
      }
      return lb;
    }

    std::vector<double> getUpperBoundLimits(){
      Eigen::VectorXd lb_;
      Eigen::VectorXd ub_;
      std::vector<double> ub;
      $self->getLimits(lb_, ub_);
      for(int i=0; i < ub_.size(); i++){
        ub.push_back(ub_(i));
      }
      return ub;
    }


    // Set limits, Python takes care of checking number of limits
    void setKDLLimits(const std::vector<double> lb, const std::vector<double> ub) {
      Eigen::VectorXd lb_;
      Eigen::VectorXd ub_;
      lb_.resize(lb.size());
      for(unsigned int i=0; i < lb.size(); i++){
        lb_(i) = lb[i];
      }
      ub_.resize(ub.size());
      for(unsigned int i=0; i < ub.size(); i++){
        ub_(i) = ub[i];
      }
      $self->setLimits(lb_, ub_);
    }

};
