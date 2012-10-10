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

/// \brief Declaration of HumanoidDynamicRobot class.

#ifndef ARD_RBDL_MODEL_HUMANOID_DYNAMIC_ROBOT_HH
# define ARD_RBDL_MODEL_HUMANOID_DYNAMIC_ROBOT_HH

# include <boost/enable_shared_from_this.hpp>

# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <ard/rbdl/model/dynamic-robot.hh>
# include <ard/rbdl/model/hand.hh>
# include <ard/rbdl/model/foot.hh>

namespace ard
{
  namespace rbdl
  {
    class HumanoidDynamicRobot :
      public CjrlHumanoidDynamicRobot,
      public boost::enable_shared_from_this<HumanoidDynamicRobot>
    {
    public:
      /// \name Initialization
      /// \{

      /// \brief Default construcor.
      HumanoidDynamicRobot ();

      /// \brief Destructor.
      virtual ~HumanoidDynamicRobot();

      /// \brief Initialize data-structure necessary to dynamic computations
      /// This function should be called after building the tree of joints.
      virtual bool initialize ();

      /// \}

      /// \name Kinematic chain
      /// \{

      /// \brief Set the root joint of the robot.
      virtual void rootJoint (CjrlJoint& inJoint);

      /// \brief Get the root joint of the robot.
      virtual CjrlJoint* rootJoint () const;

      /// \brief Get a vector containing all the joints.
      virtual std::vector<CjrlJoint*> jointVector ();

      /// \brief Get the chain of joints between two joints
      /// \param inStartJoint First joint.
      /// \param inEndJoint Second joint.
      virtual std::vector<CjrlJoint*>
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
      virtual void setJointOrderInConfig (std::vector<CjrlJoint*> inJointVector);

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
      virtual const std::vector<CjrlJoint*>& getActuatedJoints() const;

      /// \brief Specifies the list of actuated joints.
      virtual void setActuatedJoints(std::vector<CjrlJoint*>& lActuatedJoints);

      /// \}

      /// \name Joints specific to humanoid robots
      /// \{

      /// \brief Set the pointer to the waist.
      virtual void waist (CjrlJoint* inWaist);

      /// \brief Get a pointer to the waist.
      virtual CjrlJoint* waist () const;

      /// \brief Set the pointer to the chest.
      ///
      /// \note for some humanoid robots, the waist and the chest are
      /// the same joints.
      virtual void chest (CjrlJoint* inChest);

      /// \brief Get a pointer to the chest.
      ///
      ///\note for some humanoid robots, the waist and the chest are
      ///the same joints.
      virtual CjrlJoint* chest () const;

      /// \brief Set the pointer to the left wrist joint.
      virtual void leftWrist (CjrlJoint* inLefWrist);

      /// \brief Get a pointer to the left wrist.
      virtual CjrlJoint* leftWrist () const;

      /// \brief Set the pointer to the right wrist joint.
      virtual void rightWrist (CjrlJoint* inRightWrist);

      /// \brief Get a pointer to the right wrist.
      virtual CjrlJoint* rightWrist () const;

      /// \brief Set the pointer to the right hand.
      virtual void rightHand (CjrlHand* inRightHand);

      /// \brief Get a pointer to the right hand.
      virtual CjrlHand* rightHand () const;

      /// \brief Set the pointer to the left hand.
      virtual void leftHand (CjrlHand* inLeftHand);

      /// \brief Get a pointer to the left hand.
      virtual CjrlHand* leftHand () const;

      /// \brief Get the hand clench value.
      /// This is a scalar value ranging between 0 and 1 which
      /// describes the hand clench (0 for open and 1 for closed hand)
      virtual double getHandClench (CjrlHand* inHand);

      /// \brief Set the hand clench value. This is a scalar value
      /// ranging between 0 and 1 which describes the hand clench
      /// (0 for open and 1 for closed hand)
      /// \return false if parameter 2 is out of range
      virtual bool setHandClench (CjrlHand* inHand, double inClenchingValue);

      /// \brief Set the pointer to the left ankle joint.
      virtual void leftAnkle (CjrlJoint* inLefAnkle);

      /// \brief Get a pointer to the left ankle.
      virtual CjrlJoint* leftAnkle () const;

      /// \brief Set the pointer to the right ankle joint.
      virtual void rightAnkle (CjrlJoint* inRightAnkle);

      /// \brief Get a pointer to the right ankle.
      virtual CjrlJoint* rightAnkle () const;

      /// \brief Set the pointer to the left foot joint.
      virtual void leftFoot (CjrlFoot* inLeftFoot);

      /// \brief Get a pointer to the left foot.
      virtual CjrlFoot* leftFoot () const;

      /// \brief Set the pointer to the right foot joint.
      virtual void rightFoot (CjrlFoot* inRightFoot);

      /// \brief Get a pointer to the right foot.
      virtual CjrlFoot* rightFoot () const;

      /// \brief Set gaze joint.
      ///
      /// \note For most humanoid robots, the gaze joint is the head.
      virtual void gazeJoint (CjrlJoint* inGazeJoint);

      /// \brief Get gaze joint.
      virtual CjrlJoint* gazeJoint () const;

      /// \brief Set the gaze orientation and position in the local frame
      /// of the gaze joint.
      /// \return inOrigin a point on the gaze straight line,
      /// \return inDirection the direction of the gaze joint.
      virtual void gaze (const vector3d& inDirection, const vector3d& inOrigin);

      /// \brief Get a point on the gaze straight line.
      virtual const vector3d& gazeOrigin () const;

      /// \brief Get the direction of gaze
      virtual const vector3d& gazeDirection () const;

      /// \}

      /// \name Zero momentum point
      /// \{

      /// \brief return the coordinates of the Zero Momentum Point.
      virtual const vector3d& zeroMomentumPoint () const;

      /// \}

    private:
      /// \brief Dynamic robot attribute.
      dynamicRobotShPtr_t dynamicRobot_;
      /// \brief Waist joint attribute.
      jointWkPtr_t waist_;
      /// \brief Chest joint attribute.
      jointWkPtr_t chest_;
      /// \brief Left wrist joint attribute.
      jointWkPtr_t leftWrist_;
      /// \brief Right wrist joint attribute.
      jointWkPtr_t rightWrist_;
      /// \brief Left hand joint attribute.
      handShPtr_t leftHand_;
      /// \brief Right hand joint attribute.
      handShPtr_t rightHand_;
      /// \brief Left hand clench attribute.
      double leftHandClench_;
      /// \brief Right hand clench attribute.
      double rightHandClench_;
      /// \brief Left ankle joint attribute.
      jointWkPtr_t leftAnkle_;
      /// \brief Right ankle joint attribute.
      jointWkPtr_t rightAnkle_;
      /// \brief Left foot joint attribute.
      footShPtr_t leftFoot_;
      /// \brief Right foot joint attribute.
      footShPtr_t rightFoot_;
      /// \brief Gaze joint attribute.
      jointWkPtr_t gazeJoint_;
      /// \brief Gaze origin attribute.
      vector3d gazeOrigin_;
      /// \brief Gaze direction attribute.
      vector3d gazeDirection_;
      /// \brief Zero moment point attribute.
      vector3d zeroMomentPoint_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_HUMANOID_DYNAMIC_ROBOT_HH
