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

/// \brief Declaration of DynamicRobot class.

#ifndef ARD_RBDL_MODEL_DYNAMICROBOT_HH
# define ARD_RBDL_MODEL_DYNAMICROBOT_HH

# include <boost/enable_shared_from_this.hpp>

# include <rbdl/Model.h>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <ard/rbdl/model/joint.hh>
# include <abstract-robot-dynamics/dynamic-robot.hh>

namespace ard
{
  namespace rbdl
  {
    class DynamicRobot
      : public CjrlDynamicRobot,
	public boost::enable_shared_from_this<DynamicRobot>
    {
    public:
      /// \name Initialization
      /// \{

      /// \brief Default constructor.
      explicit DynamicRobot ();

      /// \brief Copy constructor.
      DynamicRobot (const DynamicRobot& robot);

      /// \brief Assignment operator.
      DynamicRobot operator= (const DynamicRobot& robot);

      /// \brief Destructor
      virtual ~DynamicRobot ();

      /// \brief Initialize data-structure necessary to dynamic computations
      /// This function should be called after building the tree of joints.
      virtual bool initialize ();

      /// \}

      /// \name Kinematic chain
      /// \{

      /// \brief Set the root joint of the robot.
      virtual void rootJoint (CjrlJoint& inJoint);

      /// \brief Get the root joint of the robot.
      virtual to_pointer<CjrlJoint>::type rootJoint () const;

      /// \brief Get a vector containing all the joints.
      virtual std::vector<to_pointer<CjrlJoint>::type > jointVector ();

      /// \brief Get underlying rbdl model.
      virtual rbdlModel_t rbdlModel () const;

      /// \brief Get gravity vector. It is set to a default value on
      /// initialization.
      virtual vector3d gravity () const;

      /// \brief Set gravity vector.
      virtual void gravity (const vector3d& gravity);

      /// \brief Get the chain of joints between two joints
      /// \param inStartJoint First joint.
      /// \param inEndJoint Second joint.
      virtual std::vector<to_pointer<CjrlJoint>::type >
      jointsBetween (const CjrlJoint& inStartJoint,
		     const CjrlJoint& inEndJoint) const;

      /// \brief Get the upper bound for ith dof.
      virtual double upperBoundDof (unsigned int inRankInConfiguration);

      /// \brief Get the lower bound for ith dof.
      virtual double lowerBoundDof (unsigned int inRankInConfiguration);

      /// \brief Compute the upper bound for ith dof using other
      /// configuration values if possible.
      virtual double upperBoundDof (unsigned int inRankInConfiguration,
				    const vectorN& inConfig);

      /// \brief Compute the lower bound for ith dof using other
      /// configuration values if possible.
      virtual double lowerBoundDof (unsigned int inRankInConfiguration,
				    const vectorN& inConfig);

      /// \brief Get the number of degrees of freedom of the robot.
      virtual unsigned int numberDof () const;

      /// \brief Set the joint ordering in the configuration vector
      ///
      /// \param inJointVector Vector of the robot joints
      ///
      /// Specifies the order of the joints in the configuration vector.
      /// The vector should contain all the joints of the current robot.
      virtual void
      setJointOrderInConfig
      (std::vector<to_pointer<CjrlJoint>::type > inJointVector);

      /// \}

      /// \name Configuration, velocity and acceleration

      /// \brief Set the current configuration of the robot.
      ///
      /// \param inConfig the configuration vector \f${\bf q}\f$.
      ///
      /// \return true if success, false if failure (the dimension of the
      /// input vector does not fit the number of degrees of freedom of the
      /// robot).
      virtual bool currentConfiguration (const vectorN& inConfig);

      /// \brief Get the current configuration of the robot.
      ///
      /// \return the configuration vector \f${\bf q}\f$.
      virtual const vectorN& currentConfiguration () const;

      /// \brief Set the current velocity of the robot.
      ///
      /// \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.
      ///
      /// \return true if success, false if failure (the dimension of the
      /// input vector does not fit the number of degrees of freedom of the
      /// robot).
      virtual bool currentVelocity (const vectorN& inVelocity);

      /// \brief Get the current velocity of the robot.
      ///
      /// \return the velocity vector \f${\bf \dot{q}}\f$.
      virtual const vectorN& currentVelocity () const;

      /// \brief Set the current acceleration of the robot.
      ///
      /// \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.
      ///
      /// \return true if success, false if failure (the dimension of the
      /// input vector does not fit the number of degrees of freedom of the
      /// robot).
      virtual bool currentAcceleration (const vectorN& inAcceleration);

      /// \brief Get the current acceleration of the robot.
      ///
      /// \return the acceleration vector \f${\bf \ddot{q}}\f$.
      virtual const vectorN& currentAcceleration () const;

      /// \brief Get the current forces of the robot.
      ///
      /// \return the force vector \f${\bf f}\f$.
      virtual const matrixNxP& currentForces () const;

      /// \brief Get the current torques of the robot.
      ///
      /// \return the torque vector \f${\bf \tau }\f$.
      virtual const matrixNxP& currentTorques () const;

      /// \brief Get the current joint torques of the robot.
      ///
      /// The torques are computed by internal calls to the
      /// direct dynamic computations. Dynamics is computed in free-floating
      /// mode, supposing no contact with the environments, and knowing
      /// given position, velocity and acceleration. This accessor only give
      /// a reference on the already-computed values.
      /// \return the torque vector \f${\bf \tau }\f$.
      virtual const vectorN& currentJointTorques () const;

      /// \}

      /// \name Forward kinematics and dynamics

      /// \brief Compute forward kinematics.
      ///
      /// Update the position, velocity and accelerations of each
      /// joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.
      virtual bool computeForwardKinematics ();

      /// \brief Compute the dynamics of the center of mass.
      ///
      /// Compute the linear and angular momentum and their time
      /// derivatives, at the center of mass.
      virtual bool computeCenterOfMassDynamics ();

      /// \brief Get the position of the center of mass.
      virtual const vector3d& positionCenterOfMass () const;

      /// \brief Get the velocity of the center of mass.
      virtual const vector3d& velocityCenterOfMass ();

      /// \brief Get the acceleration of the center of mass.
      virtual const vector3d& accelerationCenterOfMass ();

      /// \brief Get the linear momentum of the robot.
      virtual const vector3d& linearMomentumRobot ();

      /// \brief Get the time-derivative of the linear momentum.
      virtual const vector3d& derivativeLinearMomentum ();

      /// \brief Get the angular momentum of the robot at the center of mass.
      virtual const vector3d& angularMomentumRobot ();

      /// \brief Get the time-derivative of the angular momentum at the
      /// center of mass.
      virtual const vector3d& derivativeAngularMomentum ();

      /// \brief Get the total mass of the robot
      virtual double mass () const;

      /// \}

      /// \name Control of the implementation
      /// \{

      /// \brief Whether the specified property in implemented.
      virtual bool isSupported (const std::string &);

      /// \brief Get property corresponding to command name.
      ///
      /// \param inProperty name of the property.
      /// \retval outValue value of the property if implemented.
      ///
      /// \note The returned string needs to be cast into the right type
      /// (double, int,...).
      virtual bool getProperty (const std::string &,
				std::string&) const;

      /// \brief Set property corresponding to command name.
      ///
      /// \param inProperty name of the property.
      /// \param inValue value of the property.
      ///
      /// \note The value string is obtained by writing the
      /// corresponding value in a string (operator<<).
      virtual bool setProperty (std::string &,
				const std::string&);

      /// \}


      /// \brief Compute and get position and orientation jacobian
      ///
      /// \param inStartJoint First joint in the chain of joints
      /// influencing the jacobian.
      /// \param inEndJoint Joint where the control frame is located.
      /// \param inFrameLocalPosition Position of the control frame in
      /// inEndJoint local frame.
      /// \retval outjacobian computed jacobian matrix.
      /// \param offset is the rank of the column from where the
      /// jacobian is written.
      /// \param inIncludeStartFreeFlyer Option to include the
      /// contribution of a fictive freeflyer superposed with
      /// inStartJoint
      ///
      /// \return false if matrix has inadequate size. Number of
      /// columns in matrix outJacobian must be at least numberDof()
      /// if inIncludeStartFreeFlyer = true.
      /// It must be at least numberDof()-6 otherwise.
      /// 
      virtual bool getJacobian(const CjrlJoint& inStartJoint,
			       const CjrlJoint& inEndJoint,
			       const vector3d& inFrameLocalPosition,
			       matrixNxP& outjacobian,
			       unsigned int offset = 0,
			       bool inIncludeStartFreeFlyer = true);

      virtual bool getPositionJacobian(const CjrlJoint& inStartJoint,
				       const CjrlJoint& inEndJoint,
				       const vector3d& inFrameLocalPosition,
				       matrixNxP& outjacobian,
				       unsigned int offset = 0,
				       bool inIncludeStartFreeFlyer = true);

      virtual bool getOrientationJacobian(const CjrlJoint& inStartJoint,
					  const CjrlJoint& inEndJoint,
					  matrixNxP& outjacobian,
					  unsigned int offset = 0,
					  bool inIncludeStartFreeFlyer = true);

      virtual bool getJacobianCenterOfMass(const CjrlJoint& inStartJoint,
					   matrixNxP& outjacobian,
					   unsigned int offset = 0,
					   bool inIncludeStartFreeFlyer = true);

      ///\name Inertia matrix related methods
      /// \{

      /// \brief Compute the inertia matrix of the robot according wrt
      /// \f${\bf q}\f$.
      virtual void computeInertiaMatrix ();

      /// \brief Get the inertia matrix of the robot according wrt \f${\bf q}\f$.
      virtual const matrixNxP& inertiaMatrix () const;
      /// \}

      /// \name Actuated joints related methods.
      /// \{

      /// \brief Returns the list of actuated joints.
      virtual const
      std::vector<to_pointer<CjrlJoint>::type >& getActuatedJoints() const;

      /// \brief Specifies the list of actuated joints.
      virtual void
      setActuatedJoints
      (std::vector<to_pointer<CjrlJoint>::type >& lActuatedJoints);

      /// \}

    protected:
      /// \brief Build joint vector by exploring tree starting from
      /// a specified joint (usually the root joint).
      ///
      /// Result can be retrieved with associated getter. It contains
      /// all joints, including fixed ones.
      virtual bool buildJointVector (const jointShPtr_t& joint);

      /// \brief Build rbdl model attribute.
      ///
      /// This method builds the rbdl model from the joint vector.
      ///
      /// \warning Make sure joint vector is built before calling this
      /// method.
      ///
      /// \retval true if success, false if failure
      virtual bool buildRbdlModel ();

      /// \brief Get the root joint of the robot.
      virtual void rootJoint (jointShPtr_t& joint) const;

      /// \brief Set the root joint of the robot.
      virtual void rootJoint (joint_t& joint);

      /// \brief Get a vector containing share pointers to all joints.
      virtual void jointVector (jointShPtrs_t& jointVector) const;

      /// \brief Get a vector containing weak pointers to all joints.
      virtual void jointVector (jointWkPtrs_t& jointVector) const;

      /// \brief Get the velocity of the center of mass.
      virtual const vector3d& velocityCenterOfMass () const;

      /// \brief Get the acceleration of the center of mass.
      virtual const vector3d& accelerationCenterOfMass () const;

      /// \brief Get the linear momentum of the robot.
      virtual const vector3d& linearMomentumRobot () const;

      /// \brief Get the time-derivative of the linear momentum.
      virtual const vector3d& derivativeLinearMomentum () const;

      /// \brief Get the angular momentum of the robot at the center of mass.
      virtual const vector3d& angularMomentumRobot () const;

      /// \brief Get the time-derivative of the angular momentum at the
      /// center of mass.
      virtual const vector3d& derivativeAngularMomentum () const;

      /// \brief Get the vector of actuated joints.
      void actuatedJoints (jointWkPtrs_t& actuatedJoints) const;

    private:
      /// \brief Root joint pointer attribute.
      jointShPtr_t rootJoint_;
      /// \brief Joint vector attribute.
      jointWkPtrs_t jointVector_;
      /// \brief rbdl model attribute.
      rbdlModel_t rbdlModel_;
      /// \brief configuration attribute.
      vectorN configuration_;
      /// \brief velocity attribute.
      vectorN velocity_;
      /// \brief acceleration attribute.
      vectorN acceleration_;
      /// \brief forces attribute.
      matrixNxP forces_;
      /// \brief torques attribute.
      matrixNxP torques_;
      /// \brief jointTorques attribute.
      vectorN jointTorques_;
      /// \brief center of mass position attribute.
      vector3d positionCenterOfMass_;
      /// \brief center of mass velocity attribute.
      vector3d velocityCenterOfMass_;
      /// \brief center of mass acceleration attribute.
      vector3d accelerationCenterOfMass_;
      /// \brief linear momentum attribute.
      vector3d linearMomentumRobot_;
      /// \brief linear momentum derivative attribute.
      vector3d derivativeLinearMomentum_;
      /// \brief angular momentum attribute.
      vector3d angularMomentumRobot_;
      /// \brief angular momentum derivative attribute.
      vector3d derivativeAngularMomentum_;
      /// \brief mass attribute.
      double mass_;
      /// \brief inertia matrix attribute.
      matrixNxP inertiaMatrix_;
      /// \brief actuated joints attribute.
      jointWkPtrs_t actuatedJoints_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_DYNAMICROBOT_HH
