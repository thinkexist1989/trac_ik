 /* trac_ik_wrap.i */
 %module trac_ik_wrap

// Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>

 %{
 /* Includes the header in the wrapper code */
 #include <trac_ik/trac_ik.hpp>
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
    // int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());
        
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
      KDL::Frame frame(KDL::Rotation::Quaternion(rx/norm, ry/norm, rz/norm, rw/norm), KDL::Vector(x,y,z));
      KDL::Chain chain;
      $self->getKDLChain(chain);
      if (q_init.size() != chain.getNrOfJoints()) throw std::invalid_argument("Wrong seed size");

      KDL::JntArray in(q_init.size()), out(q_init.size());

      for (uint z=0; z < q_init.size(); z++)
          in(z) = q_init[z];

      KDL::Twist bounds = KDL::Twist::Zero();
      bounds.vel.x(boundx);
      bounds.vel.y(boundy);
      bounds.vel.z(boundz);
      bounds.rot.x(boundrx);
      bounds.rot.y(boundry);
      bounds.rot.z(boundrz);

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
      KDL::Chain chain;
      $self->getKDLChain(chain);
      return (int) chain.getNrOfJoints();
    }

    // Convenience method to get the list of joint names as used internally
    std::vector<std::string> getJointNamesInChain(const std::string& urdf_string){
      KDL::Chain chain;
      $self->getKDLChain(chain);
      std::vector<KDL::Segment> chain_segs = chain.segments;

      std::vector<std::string> joint_names_;
      std::vector<std::string> link_names_;
      for (const auto& segment : chain_segs)
        if (segment.getJoint().getType() != KDL::Joint::Fixed)
          joint_names_.push_back(segment.getJoint().getName());
      return joint_names_;
    }


    // Convenience method to get the list of link names as used internally
    std::vector<std::string> getLinkNamesInChain(){
      KDL::Chain chain;
      $self->getKDLChain(chain);
      std::vector<KDL::Segment> chain_segs = chain.segments;
      std::vector<std::string> link_names_;
      for(unsigned int i = 0; i < chain_segs.size(); ++i) {
        link_names_.push_back(chain_segs[i].getName());
      }
      return link_names_;
    }


    // Get KDL limits
    std::vector<double> getLowerBoundLimits(){
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      std::vector<double> lb;
      $self->getKDLLimits(lb_, ub_);
      for(unsigned int i=0; i < lb_.rows(); i++){
        lb.push_back(lb_(i));
      }
      return lb;
    }

    std::vector<double> getUpperBoundLimits(){
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      std::vector<double> ub;
      $self->getKDLLimits(lb_, ub_);
      for(unsigned int i=0; i < ub_.rows(); i++){
        ub.push_back(ub_(i));
      }
      return ub;
    }


    // Set KDL limits, Python takes care of checking number of limits
    void setKDLLimits(const std::vector<double> lb, const std::vector<double> ub) {
      KDL::JntArray lb_;
      KDL::JntArray ub_;
      lb_.resize(lb.size());
      for(unsigned int i=0; i < lb.size(); i++){
        lb_(i) = lb[i];
      }
      ub_.resize(ub.size());
      for(unsigned int i=0; i < ub.size(); i++){
        ub_(i) = ub[i];
      }
      $self->setKDLLimits(lb_, ub_);
    }

};

