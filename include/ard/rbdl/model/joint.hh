// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the ard-rbdl.
//
// ard-rbdl is free software: you can redistribute it and/or modify it
// under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// ard-rbdl is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with ard-rbdl.  If not, see
// <http://www.gnu.org/licenses/>.

/// \brief Declaration of Joint class.

#ifndef ARD_RBDL_MODEL_JOINT_HH
# define ARD_RBDL_MODEL_JOINT_HH

# include <boost/enable_shared_from_this.hpp>

# include <rbdl/Joint.h>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <ard/rbdl/model/body.hh>
# include <abstract-robot-dynamics/joint.hh>

namespace ard
{
  namespace rbdl
  {
    class Joint :
      public CjrlJoint,
      public boost::enable_shared_from_this<Joint>
    {
    public:
      /// \brief Default Constructor.
      explicit Joint ();

      /// \brief Create a joint with a given initial position.
      ///
      /// \param initialPosition initial transformation in absolute
      /// frame.
      explicit Joint (const matrix4d& initialPosition);

      /// \brief Copy Constructor.
      Joint (Joint& joint);

      /// \brief Destructor
      virtual ~Joint ();

      /// \name Joint name
      /// \{

      /// \brief Get joint name.
      virtual const std::string& getName () const;

      /// \brief Set joint name.
      virtual void setName (const std::string& name);

      /// \}

      /// \name Joint hierarchy
      /// \{

      /// \brief Get a pointer to the parent joint (if any).
      /// \warning A shared pointer is created then deleted in this
      /// getter. There is therefore no guarantee that the joint will
      /// still exist once the shared pointer has been deleted.
      virtual CjrlJoint* parentJoint () const;

      /// \brief Add a child joint.
      virtual bool addChildJoint (CjrlJoint& joint);

      /// \brief Get the number of children.
      virtual unsigned int countChildJoints () const;

      /// \brief Return the child joint at the given rank.
      virtual CjrlJoint* childJoint (unsigned int jointRank) const;

      /// \brief Get underlying rbdl joint.
      virtual rbdlJoint_t rbdlJoint () const;

      /// \brief Get a vector containing references of the joints
      /// between the rootJoint and this joint. The root Joint and
      /// this Joint are included in the vector.
      virtual std::vector<CjrlJoint*> jointsFromRootToThis () const;

      /// \brief Get the rank of this joint in the robot
      /// configuration vector.
      virtual unsigned int rankInConfiguration () const;

      /// \}

      /// \name Joint kinematics
      /// \{

      /// \brief Get the initial position of the joint.

      /// The initial position of the joint is the position of the
      /// local frame of the joint.
      virtual const matrix4d& initialPosition () const;

      /// \brief Update this joint's transformation according to
      /// degree of freedom value from argument robot
      /// configuration. This does not update the transformations of
      /// child joints.
      /// \param inDofVector is a robot configuration vector.
      /// \return false if argument vector's size is not equal to
      /// the robot's number of degrees of freedom
      virtual bool updateTransformation (const vectorN &inDofVector);

      /// \brief Get the current transformation of the joint.
      ///
      /// The current transformation of the joint is the
      /// transformation moving the joint from the position in
      /// initial configuration to the current position.
      ///
      /// The current transformation is determined by the
      /// configuration \f${\bf q}\f$ of the robot.
      virtual const matrix4d& currentTransformation () const;

      /// \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.
      ///
      /// The velocity is determined by the configuration of the
      /// robot and its time derivative: \f$({\bf q},{\bf
      /// \dot{q}})\f$.
      ///
      /// \return the linear velocity \f${\bf v}\f$ of the origin of
      /// the joint frame and the angular velocity \f${\bf
      /// \omega}\f$ of the joint frame.
      virtual CjrlRigidVelocity jointVelocity () const;

      ///  \brief Get the acceleration of the joint.
      ///
      /// The acceleratoin is determined by the configuration of the
      /// robot and its first and second time derivative: \f$({\bf
      /// q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
      virtual CjrlRigidAcceleration jointAcceleration () const;

      /// \brief Get the number of degrees of freedom of the joint.
      virtual unsigned int numberDof () const;

      /// \}

      /// \name Bounds of the degrees of freedom
      /// \{

      /// \brief Get the lower bound of a given degree of freedom of
      /// the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      virtual double lowerBound (unsigned int inDofRank) const;

      /// \brief Get the upper bound of a given degree of freedom of
      /// the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      virtual double upperBound (unsigned int inDofRank) const;

      /// \brief Set the lower bound of a given degree of freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint \param
      /// inLowerBound lower bound
      virtual void lowerBound (unsigned int inDofRank, double inLowerBound);

      /// \brief Set the upper bound of a given degree of freedom of
      /// the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      /// \param inUpperBound Upper bound.
      virtual void upperBound (unsigned int inDofRank, double inUpperBound);

      /// \brief Get the lower velocity bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      virtual double lowerVelocityBound (unsigned int inDofRank) const;

      /// \brief Get the upper veocity bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      virtual double upperVelocityBound (unsigned int inDofRank) const;

      /// \brief Set the lower velocity bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      /// \param inLowerBound lower bound
      virtual void lowerVelocityBound (unsigned int inDofRank, double inLowerBound);

      /// \brief Set the upper velocity bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      /// \param inUpperBound Upper bound.
      virtual void upperVelocityBound (unsigned int inDofRank, double inUpperBound);

      /// \brief Get the lower torque bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      virtual double lowerTorqueBound (unsigned int inDofRank) const;

      /// \brief Get the upper veocity bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      virtual double upperTorqueBound (unsigned int inDofRank) const;

      /// \brief Set the lower torque bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      /// \param inLowerBound lower bound
      virtual void lowerTorqueBound (unsigned int inDofRank, double inLowerBound);

      /// \brief Set the upper torque bound of a given degree of
      /// freedom of the joint.
      ///
      /// \param inDofRank Id of the dof in the joint
      /// \param inUpperBound Upper bound.
      virtual void upperTorqueBound (unsigned int inDofRank, double inUpperBound);

      /// \}

      /// \name Jacobian functions wrt configuration.
      /// \{

      /// \brief Get the Jacobian matrix of the joint position and
      /// orientation wrt the robot configuration.
      ///
      /// Kinematic constraints from interaction with the
      /// environment are not taken into account for this
      /// computation.
      ///
      /// The corresponding computation can be done by the robot for
      /// each of its joints or by the joint.
      ///
      /// \return a matrix \f$J \in {\bf R}^{6\times n_{dof}}\f$ defined by
      /// \f[
      /// J = \left(\begin{array}{llll}
      /// {\bf v_1} & {\bf v_2} & \cdots & {\bf v_{n_{dof}}} \\
      /// {\bf \omega_1} & {\bf \omega_2} & \cdots & {\bf \omega_{n_{dof}}}
      /// \end{array}\right)
      /// \f]
      /// where \f${\bf v_i}\f$ and \f${\bf \omega_i}\f$ are
      /// respectively the linear and angular velocities of the
      /// joint implied by the variation of degree of freedom
      /// \f$q_i\f$. The velocity of the joint returned by
      /// CjrlJoint::jointVelocity can thus be obtained through the
      /// following formula:
      /// \f[
      /// \left(\begin{array}{l} {\bf v} \\ {\bf
      /// \omega}\end{array}\right) = J {\bf \dot{q}}
      /// \f]
      virtual const matrixNxP& jacobianJointWrtConfig () const;

      /// \brief Compute the joint's jacobian wrt the robot configuration.
      virtual void computeJacobianJointWrtConfig ();

      /// \brief Get the jacobian of the point specified in local
      /// frame by inPointJointFrame.
      /// The output matrix outjacobian is automatically resized if necessary
      virtual void getJacobianPointWrtConfig
      (const vector3d& inPointJointFrame, matrixNxP& outjacobian) const;

      /// \}

      /// \name Body linked to the joint
      /// \{

      /// \brief Get a pointer to the linked body (if any).
      virtual CjrlBody* linkedBody () const;

      /// \brief Link a body to the joint.
      virtual void setLinkedBody (CjrlBody& inBody);

    protected:
      /// \brief Set the parent joint.
      virtual void setParentJoint (jointShPtr_t joint);

      /// \brief Add a child joint of type joint_t.
      virtual bool addChildJoint (joint_t& joint);
      
      /// \brief Link a body to a joint.
      void setLinkedBody (body_t& body);

    private:
      /// \brief Joint name attribute.
      std::string name_;
      /// \brief Parent joint attribute.
      jointWkPtr_t parentJoint_;
      /// \brief Child joints vector attribute.
      jointShPtrs_t childJoints_;
      /// \brief Corresponding rbdl joint attribute.
      rbdlJoint_t rbdlJoint_;
      /// \brief Rank in configuration attribute.
      unsigned int rankInConfiguration_;
      /// \brief Joint transformation in world frame at construction.
      matrix4d initialPosition_;
      /// \brief Joint current transformation attribute.
      matrix4d currentTransformation_;
      /// \brief Joint velocity attribute.
      ardVelocity_t velocity_;
      /// \brief Joint acceleration attribute.
      ardAcceleration_t acceleration_;
      /// \brief Number of dofs attribute.
      unsigned int numberDof_;
      /// \brief Lower bounds attribute.
      std::vector<double> lowerBound_;
      /// \brief Upper bounds attribute.
      std::vector<double> upperBound_;
      /// \brief Lower velocity bounds attribute.
      std::vector<double> lowerVelocityBound_;
      /// \brief Upper velocity bounds attribute.
      std::vector<double> upperVelocityBound_;
      /// \brief Lower torque bounds attribute.
      std::vector<double> lowerTorqueBound_;
      /// \brief Upper torque bounds attribute.
      std::vector<double> upperTorqueBound_;
      /// brief Joint transformation jacobian with respect to
      /// configuration.
      matrixNxP jacobian_;
      /// \brief Linked body attribute.
      bodyShPtr_t linkedBody_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_JOINT_HH
