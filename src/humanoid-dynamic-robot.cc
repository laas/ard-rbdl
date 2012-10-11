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

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::rootJoint () const
    {
      return dynamicRobot_->rootJoint ();
    }

    std::vector<to_pointer<CjrlJoint>::type >
    HumanoidDynamicRobot::jointVector ()
    {
      return dynamicRobot_->jointVector ();
    }

    std::vector<to_pointer<CjrlJoint>::type >
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
    HumanoidDynamicRobot::setJointOrderInConfig
    (std::vector<to_pointer<CjrlJoint>::type > jointVector)
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

    const std::vector<to_pointer<CjrlJoint>::type >&
    HumanoidDynamicRobot::getActuatedJoints () const
    {
      return dynamicRobot_->getActuatedJoints ();
    }
 
    void
    HumanoidDynamicRobot::setActuatedJoints
    (std::vector<to_pointer<CjrlJoint>::type >& actuatedJoints)
    {
      return dynamicRobot_->setActuatedJoints (actuatedJoints);
    }

    void HumanoidDynamicRobot::waist (to_pointer<CjrlJoint>::type waist)
    {
      return getPtrFromBase (waist_, waist);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::waist () const
    {
      return getSharedPointer (waist_);
    }

    void HumanoidDynamicRobot::chest (to_pointer<CjrlJoint>::type chest)
    {
      return getPtrFromBase (chest_, chest);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::chest () const
    {
      return getSharedPointer (chest_);
    }

    void HumanoidDynamicRobot::leftWrist (to_pointer<CjrlJoint>::type leftWrist)
    {
      return getPtrFromBase (leftWrist_, leftWrist);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::leftWrist () const
    {
      return getSharedPointer (leftWrist_);
    }

    void HumanoidDynamicRobot::rightWrist
    (to_pointer<CjrlJoint>::type rightWrist)
    { 
      return getPtrFromBase (rightWrist_, rightWrist);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::rightWrist () const
    {
      return getSharedPointer (rightWrist_);
    }

    void HumanoidDynamicRobot::rightHand (to_pointer<CjrlHand>::type rightHand)
    {
      return getPtrFromBase (rightHand_, rightHand);
    }

    to_pointer<CjrlHand>::type HumanoidDynamicRobot::rightHand () const
    {
      return rightHand_;
    }

    void HumanoidDynamicRobot::leftHand (to_pointer<CjrlHand>::type leftHand)
    {
      return getPtrFromBase (leftHand_, leftHand);
    }

    to_pointer<CjrlHand>::type HumanoidDynamicRobot::leftHand () const
    {
      return leftHand_;
    }

    double HumanoidDynamicRobot::getHandClench (to_pointer<CjrlHand>::type hand)
    {
      ardHandPtr_t rightHandPtr = getUnsafePointer (rightHand_);
      ardHandPtr_t leftHandPtr = getUnsafePointer (leftHand_);
      ardHandPtr_t handPtr = getUnsafePointer (hand);
      if (rightHandPtr == handPtr)
	return rightHandClench_;
      else if (leftHandPtr == handPtr)
	return leftHandClench_;
      else
	throw std::runtime_error ("Hand pointer not found in robot.");
    }

    bool
    HumanoidDynamicRobot::setHandClench
    (to_pointer<CjrlHand>::type hand, double clenchingValue)
    {
      if (clenchingValue >=0 && clenchingValue <=1)
	{
	  ardHandPtr_t rightHandPtr = getUnsafePointer (rightHand_);
	  ardHandPtr_t leftHandPtr = getUnsafePointer (leftHand_);
	  ardHandPtr_t handPtr = getUnsafePointer (hand);
	  if (rightHandPtr == handPtr)
	    return rightHandClench_;
	  else if (leftHandPtr == handPtr)
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

    void HumanoidDynamicRobot::rightAnkle (to_pointer<CjrlJoint>::type rightAnkle)
    {
      getPtrFromBase (rightAnkle_, rightAnkle);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::rightAnkle () const
    {
      return getSharedPointer (rightAnkle_);
    }

    void HumanoidDynamicRobot::leftAnkle (to_pointer<CjrlJoint>::type leftAnkle)
    {
      getPtrFromBase (leftAnkle_, leftAnkle);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::leftAnkle () const
    {
      return getSharedPointer (leftAnkle_);
    }

    void HumanoidDynamicRobot::rightFoot (to_pointer<CjrlFoot>::type rightFoot)
    {
      getPtrFromBase (rightFoot_, rightFoot);
    }

    to_pointer<CjrlFoot>::type HumanoidDynamicRobot::rightFoot () const
    {
      return rightFoot_;
    }

    void HumanoidDynamicRobot::leftFoot (to_pointer<CjrlFoot>::type leftFoot)
    {
      getPtrFromBase (leftFoot_, leftFoot);
    }

    to_pointer<CjrlFoot>::type HumanoidDynamicRobot::leftFoot () const
    {
      return leftFoot_;
    }

    void HumanoidDynamicRobot::gazeJoint (to_pointer<CjrlJoint>::type gazeJoint)
    {
      getPtrFromBase (gazeJoint_, gazeJoint);
    }

    to_pointer<CjrlJoint>::type HumanoidDynamicRobot::gazeJoint () const
    {
      return getSharedPointer (gazeJoint_);
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
