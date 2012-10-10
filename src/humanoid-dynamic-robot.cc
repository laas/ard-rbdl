// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the ard-rbdl.
//
// ard-rbdl is free software: you can redistribute it and/or modify it
// under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// ard-rbdl is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with ard-rbdl.  If not, see
// <http://www.gnu.org/licenses/>.


/// \file src/model/humanoid-dynamic-robot.cc
/// \brief Implementation of HumanoidDynamicRobot.

#include <stdexcept>

#include <boost/format.hpp>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/humanoid-dynamic-robot.hh>

namespace ard
{
  namespace rbdl
  {
    HumanoidDynamicRobot::HumanoidDynamicRobot () :
      boost::enable_shared_from_this<HumanoidDynamicRobot> (),
      dynamicRobot_ (),
      waist_ (),
      chest_ (),
      leftWrist_ (),
      rightWrist_ (),
      leftHand_ (),
      rightHand_ (),
      leftHandClench_ (),
      rightHandClench_ (),
      leftAnkle_ (),
      rightAnkle_ (),
      leftFoot_ (),
      rightFoot_ (),
      gazeJoint_ (),
      gazeOrigin_ (),
      gazeDirection_ (),
      zeroMomentPoint_ ()
    {
    }

    HumanoidDynamicRobot::~HumanoidDynamicRobot ()
    {
    }

    bool HumanoidDynamicRobot::initialize ()
    {
      return dynamicRobot_->initialize ();
    }

    void HumanoidDynamicRobot::rootJoint (CjrlJoint& joint)
    {
      return dynamicRobot_->rootJoint (joint);
    }

    CjrlJoint* HumanoidDynamicRobot::rootJoint () const
    {
      return dynamicRobot_->rootJoint ();
    }

    std::vector<CjrlJoint*> HumanoidDynamicRobot::jointVector ()
    {
      return dynamicRobot_->jointVector ();
    }

    std::vector<CjrlJoint*>
    HumanoidDynamicRobot::jointsBetween (const CjrlJoint& startJoint,
					 const CjrlJoint& endJoint) const
    {
      return dynamicRobot_->jointsBetween (startJoint, endJoint);
    }

    double
    HumanoidDynamicRobot::upperBoundDof (unsigned int rankInConfiguration)
    {
      return dynamicRobot_->upperBoundDof (rankInConfiguration);
    }

    double
    HumanoidDynamicRobot::lowerBoundDof (unsigned int rankInConfiguration)
    {
      return dynamicRobot_->lowerBoundDof (rankInConfiguration);
    }

    double
    HumanoidDynamicRobot::upperBoundDof (unsigned int rankInConfiguration,
					 const vectorN& config)
    {
      return dynamicRobot_->upperBoundDof (rankInConfiguration, config);
    }

    double
    HumanoidDynamicRobot::lowerBoundDof (unsigned int rankInConfiguration,
					 const vectorN& config)
    {
      return dynamicRobot_->lowerBoundDof (rankInConfiguration, config);
    }

    unsigned int HumanoidDynamicRobot::numberDof () const
    {
      return dynamicRobot_->numberDof ();
    }

    void
    HumanoidDynamicRobot::setJointOrderInConfig (std::vector<CjrlJoint*> jointVector)
    {
      return dynamicRobot_->setJointOrderInConfig (jointVector);
    }

    bool HumanoidDynamicRobot::currentConfiguration (const vectorN& config)
    {
      return dynamicRobot_->currentConfiguration (config);
    }

    const vectorN& HumanoidDynamicRobot::currentConfiguration () const
    {
      return dynamicRobot_->currentConfiguration ();
    }

    bool HumanoidDynamicRobot::currentVelocity (const vectorN& velocity)
    {
      return dynamicRobot_->currentVelocity (velocity);
    }

    const vectorN& HumanoidDynamicRobot::currentVelocity () const
    {
      return dynamicRobot_->currentVelocity ();
    }

    bool HumanoidDynamicRobot::currentAcceleration (const vectorN& acceleration)
    {
      return dynamicRobot_->currentAcceleration (acceleration);
    }

    const vectorN& HumanoidDynamicRobot::currentAcceleration () const
    {
      return dynamicRobot_->currentAcceleration ();
    }

    const matrixNxP& HumanoidDynamicRobot::currentForces () const
    {
      return dynamicRobot_->currentForces ();
    }

    const matrixNxP& HumanoidDynamicRobot::currentTorques () const
    {
      return dynamicRobot_->currentTorques ();
    }

    const vectorN& HumanoidDynamicRobot::currentJointTorques () const
    {
      return dynamicRobot_->currentJointTorques ();
    }

    bool HumanoidDynamicRobot::computeForwardKinematics ()
    {
      return dynamicRobot_->computeForwardKinematics ();
      // FIXME Add Zmp computation.
    }

    bool HumanoidDynamicRobot::computeCenterOfMassDynamics ()
    {
      return dynamicRobot_->computeCenterOfMassDynamics ();
    }

    const vector3d& HumanoidDynamicRobot::positionCenterOfMass () const
    {
      return dynamicRobot_->positionCenterOfMass ();
    }

    const vector3d& HumanoidDynamicRobot::velocityCenterOfMass ()
    {
      return dynamicRobot_->velocityCenterOfMass ();
    }

    const vector3d& HumanoidDynamicRobot::accelerationCenterOfMass ()
    {
      return dynamicRobot_->accelerationCenterOfMass ();
    }

    const vector3d& HumanoidDynamicRobot::linearMomentumRobot ()
    {
      return dynamicRobot_->linearMomentumRobot ();
    }

    const vector3d& HumanoidDynamicRobot::derivativeLinearMomentum ()
    {
      return dynamicRobot_->derivativeLinearMomentum ();
    }

    const vector3d& HumanoidDynamicRobot::angularMomentumRobot ()
    {
      return dynamicRobot_->angularMomentumRobot ();
    }

    const vector3d& HumanoidDynamicRobot::derivativeAngularMomentum ()
    {
      return dynamicRobot_->derivativeAngularMomentum ();
    }

    double HumanoidDynamicRobot::mass () const
    {
      return dynamicRobot_->mass ();
    }

    bool HumanoidDynamicRobot::isSupported (const std::string& key)
    {
      return dynamicRobot_->isSupported (key);
    }

    bool HumanoidDynamicRobot::getProperty (const std::string & key,
					    std::string& value) const
    {
      return dynamicRobot_->getProperty (key, value);
    }

    bool HumanoidDynamicRobot::setProperty (std::string& key,
					    const std::string& value)
    {
      return dynamicRobot_->setProperty (key, value);
    }

    bool
    HumanoidDynamicRobot::getJacobian (const CjrlJoint& inStartJoint,
				       const CjrlJoint& inEndJoint,
				       const vector3d& inFrameLocalPosition,
				       matrixNxP& outjacobian,
				       unsigned int offset,
				       bool inIncludeStartFreeFlyer)
    {
      return dynamicRobot_->getJacobian (inStartJoint,
				  inEndJoint,
				  inFrameLocalPosition,
				  outjacobian,
				  offset,
				  inIncludeStartFreeFlyer);
    }

    bool
    HumanoidDynamicRobot::getPositionJacobian
    (const CjrlJoint& inStartJoint,
     const CjrlJoint& inEndJoint,
     const vector3d& inFrameLocalPosition,
     matrixNxP& outjacobian,
     unsigned int offset,
     bool inIncludeStartFreeFlyer)
    {
      return dynamicRobot_->getPositionJacobian (inStartJoint,
					  inEndJoint,
					  inFrameLocalPosition,
					  outjacobian,
					  offset,
					  inIncludeStartFreeFlyer);
    }

    bool
    HumanoidDynamicRobot::
    getOrientationJacobian (const CjrlJoint& inStartJoint,
			    const CjrlJoint& inEndJoint,
			    matrixNxP& outjacobian,
			    unsigned int offset,
			    bool inIncludeStartFreeFlyer)
    {
      return dynamicRobot_->getOrientationJacobian (inStartJoint,
					     inEndJoint,
					     outjacobian,
					     offset,
					     inIncludeStartFreeFlyer);
    }

    bool HumanoidDynamicRobot::
    getJacobianCenterOfMass (const CjrlJoint& inStartJoint,
			     matrixNxP& outjacobian,
			     unsigned int offset,
			     bool inIncludeStartFreeFlyer)
    {
      return dynamicRobot_->getJacobianCenterOfMass (inStartJoint,
					      outjacobian,
					      offset,
					      inIncludeStartFreeFlyer);
    }

    void HumanoidDynamicRobot::computeInertiaMatrix ()
    {
      return dynamicRobot_->computeInertiaMatrix ();
    }

    const matrixNxP& HumanoidDynamicRobot::inertiaMatrix () const
    {
      return dynamicRobot_->inertiaMatrix ();
    }

    const std::vector<CjrlJoint*>& HumanoidDynamicRobot::getActuatedJoints () const
    {
      return dynamicRobot_->getActuatedJoints ();
    }
 
    void
    HumanoidDynamicRobot::setActuatedJoints (std::vector<CjrlJoint*>& actuatedJoints)
    {
      return dynamicRobot_->setActuatedJoints (actuatedJoints);
    }

    void HumanoidDynamicRobot::waist (CjrlJoint* waist)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (waist);
      if (jointPtr)
	waist_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::waist () const
    {
      return getUnsafePointer (waist_);
    }

    void HumanoidDynamicRobot::chest (CjrlJoint* chest)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (chest);
      if (jointPtr)
	chest_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::chest () const
    {
      return getUnsafePointer (chest_);
    }

    void HumanoidDynamicRobot::leftWrist (CjrlJoint* leftWrist)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (leftWrist);
      if (jointPtr)
	leftWrist_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::leftWrist () const
    {
      return getUnsafePointer (leftWrist_);
    }

    void HumanoidDynamicRobot::rightWrist (CjrlJoint* rightWrist)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (rightWrist);
      if (jointPtr)
	rightWrist_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::rightWrist () const
    {
      return getUnsafePointer (rightWrist_);
    }

    void HumanoidDynamicRobot::rightHand (CjrlHand* rightHand)
    {
      handPtr_t handPtr = dynamic_cast<handPtr_t> (rightHand);
      if (handPtr)
	rightHand_ = handPtr->shared_from_this ();
      else
	throw std::runtime_error ("Null pointer to hand.");
    }

    CjrlHand* HumanoidDynamicRobot::rightHand () const
    {
      return getUnsafePointer (rightHand_);
    }

    void HumanoidDynamicRobot::leftHand (CjrlHand* leftHand)
    {
      handPtr_t handPtr = dynamic_cast<handPtr_t> (leftHand);
      if (handPtr)
	leftHand_ = handPtr->shared_from_this ();
      else
	throw std::runtime_error ("Null pointer to hand.");
    }

    CjrlHand* HumanoidDynamicRobot::leftHand () const
    {
      return getUnsafePointer (leftHand_);
    }

    double HumanoidDynamicRobot::getHandClench (CjrlHand* hand)
    {
      ardHandPtr_t rightHandPtr = getUnsafePointer (rightHand_);
      ardHandPtr_t leftHandPtr = getUnsafePointer (leftHand_);
      if (rightHandPtr == hand)
	return rightHandClench_;
      else if (leftHandPtr == hand)
	return leftHandClench_;
      else
	throw std::runtime_error ("Hand pointer not found in robot.");
    }

    bool
    HumanoidDynamicRobot::setHandClench (CjrlHand* hand, double clenchingValue)
    {
      if (clenchingValue >=0 && clenchingValue <=1)
	{
	  ardHandPtr_t rightHandPtr = getUnsafePointer (rightHand_);
	  ardHandPtr_t leftHandPtr = getUnsafePointer (leftHand_);
	  if (rightHandPtr == hand)
	    return rightHandClench_;
	  else if (leftHandPtr == hand)
	    return leftHandClench_;
	  else
	    throw std::runtime_error ("Hand pointer not found in robot.");
	}
      else
	{
	  boost::format fmt
	    ("Wrong clenching value, expected between 0 and 1, got %1%");
	  fmt % clenchingValue;
	  throw std::runtime_error (fmt.str ());
	}
    }

    void HumanoidDynamicRobot::rightAnkle (CjrlJoint* rightAnkle)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (rightAnkle);
      if (jointPtr)
	rightAnkle_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::rightAnkle () const
    {
      return getUnsafePointer (rightAnkle_);
    }

    void HumanoidDynamicRobot::leftAnkle (CjrlJoint* leftAnkle)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (leftAnkle);
      if (jointPtr)
	leftAnkle_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::leftAnkle () const
    {
      return getUnsafePointer (leftAnkle_);
    }

    void HumanoidDynamicRobot::rightFoot (CjrlFoot* rightFoot)
    {
      footPtr_t footPtr = dynamic_cast<footPtr_t> (rightFoot);
      if (footPtr)
	rightFoot_ = footPtr->shared_from_this ();
      else
	throw std::runtime_error ("Null pointer to foot.");
    }

    CjrlFoot* HumanoidDynamicRobot::rightFoot () const
    {
      return getUnsafePointer (rightFoot_);
    }

    void HumanoidDynamicRobot::leftFoot (CjrlFoot* leftFoot)
    {
      footPtr_t footPtr = dynamic_cast<footPtr_t> (leftFoot);
      if (footPtr)
	leftFoot_ = footPtr->shared_from_this ();
      else
	throw std::runtime_error ("Null pointer to foot.");
    }

    CjrlFoot* HumanoidDynamicRobot::leftFoot () const
    {
      return getUnsafePointer (leftFoot_);
    }

    void HumanoidDynamicRobot::gazeJoint (CjrlJoint* gazeJoint)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (gazeJoint);
      if (jointPtr)
	gazeJoint_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    CjrlJoint* HumanoidDynamicRobot::gazeJoint () const
    {
      return getUnsafePointer (gazeJoint_);
    }

    void
    HumanoidDynamicRobot::gaze (const vector3d& direction,
				const vector3d& origin)
    {
      gazeDirection_ = direction;
      gazeOrigin_ = origin;
    }

    const vector3d& HumanoidDynamicRobot::gazeOrigin () const
    {
      return gazeOrigin_;
    }

    const vector3d& HumanoidDynamicRobot::gazeDirection () const
    {
      return gazeDirection_;
    }

    const vector3d& HumanoidDynamicRobot::zeroMomentumPoint () const
    {
      return zeroMomentPoint_;
    }
  
  } // end of namespace rbdl.
} // end of namespace ard.
