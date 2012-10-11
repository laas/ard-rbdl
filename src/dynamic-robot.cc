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

/// \file src/model/dynamic-robot.cc
/// \brief Implementation of DynamicRobot.

#include <stdexcept>

#include <boost/foreach.hpp>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/dynamic-robot.hh>

namespace ard
{
  namespace rbdl
  {
    DynamicRobot::DynamicRobot () :
      boost::enable_shared_from_this<DynamicRobot> (),
      rootJoint_ (),
      jointVector_ (),
      rbdlModel_ (),
      configuration_ (),
      velocity_ (),
      acceleration_ (),
      forces_ (),
      torques_ (),
      jointTorques_ (),
      positionCenterOfMass_ (),
      velocityCenterOfMass_ (),
      accelerationCenterOfMass_ (),
      linearMomentumRobot_ (),
      derivativeLinearMomentum_ (),
      angularMomentumRobot_ (),
      derivativeAngularMomentum_ (),
      mass_ (),
      inertiaMatrix_ (),
      actuatedJoints_ ()
    {
    }

    DynamicRobot::~DynamicRobot ()
    {
    }

    bool DynamicRobot::initialize ()
    {
      throw std::runtime_error ("Method not supported.");
    }

    void DynamicRobot::rootJoint (CjrlJoint& joint)
    {
      jointPtr_t ptr;
      getPtrFromBase (ptr, &joint);
      if (ptr)
	rootJoint (*ptr);
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    to_pointer<CjrlJoint>::type DynamicRobot::rootJoint () const
    {
      return rootJoint_;
    }

    std::vector<to_pointer<CjrlJoint>::type > DynamicRobot::jointVector ()
    {
      ardJointShPtrs_t ardJoints (jointVector_.size ());
      for (unsigned i = 0; i < jointVector_.size (); ++i)
	ardJoints[i] = getSharedPointer (jointVector_[i]);
      return ardJoints;
    }

    std::vector<to_pointer<CjrlJoint>::type >
    DynamicRobot::jointsBetween (const CjrlJoint& startJoint,
				 const CjrlJoint& endJoint) const
    {
      ardJointShPtrs_t robotRoot2StartJoint
	= startJoint.jointsFromRootToThis ();
      ardJointShPtrs_t robotRoot2EndJoint
	= endJoint.jointsFromRootToThis ();

      unsigned int lastCommonJointRank = 0;
      unsigned int minChainLength
	= (robotRoot2StartJoint.size () < robotRoot2EndJoint.size())
	? robotRoot2StartJoint.size() : robotRoot2EndJoint.size();

      for (unsigned int i = 1; i < minChainLength; ++i)
	if ((robotRoot2StartJoint[i] == robotRoot2EndJoint[i]))
	  ++lastCommonJointRank;

      ardJointShPtrs_t outJoints;
      for (unsigned int i = robotRoot2StartJoint.size() - 1;
	   i > lastCommonJointRank;
	   i--)
	outJoints.push_back (robotRoot2StartJoint[i]);
      if (lastCommonJointRank == 0)
	outJoints.push_back (robotRoot2EndJoint[0]);
      for (unsigned int i = lastCommonJointRank + 1;
	   i < robotRoot2EndJoint.size ();
	   i++)
	outJoints.push_back (robotRoot2EndJoint[i]);

      return outJoints;
    }

    double DynamicRobot::upperBoundDof (unsigned int rankInConfiguration)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    double DynamicRobot::lowerBoundDof (unsigned int rankInConfiguration)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    double DynamicRobot::upperBoundDof (unsigned int rankInConfiguration,
					const vectorN& config)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    double DynamicRobot::lowerBoundDof (unsigned int rankInConfiguration,
					const vectorN& config)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    unsigned int DynamicRobot::numberDof () const
    {
      return rbdlModel_.dof_count;
    }

    void
    DynamicRobot::setJointOrderInConfig
    (std::vector<to_pointer<CjrlJoint>::type > jointVector)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    bool DynamicRobot::currentConfiguration (const vectorN& config)
    {
      if (config.size () == numberDof ())
	{
	  configuration_ = config;
	  return true;
	}
      else
	return false;
    }

    const vectorN& DynamicRobot::currentConfiguration () const
    {
      return configuration_;
    }

    bool DynamicRobot::currentVelocity (const vectorN& velocity)
    {
      if (velocity.size () == numberDof ())
	{
	  velocity_ = velocity;
	  return true;
	}
      else
	return false;
    }

    const vectorN& DynamicRobot::currentVelocity () const
    {
      return velocity_;
    }

    bool DynamicRobot::currentAcceleration (const vectorN& acceleration)
    {
      if (acceleration.size () == numberDof ())
	{
	  acceleration_ = acceleration;
	  return true;
	}
      else
	return false;
    }

    const vectorN& DynamicRobot::currentAcceleration () const
    {
      return acceleration_;
    }

    const matrixNxP& DynamicRobot::currentForces () const
    {
      return forces_;
    }

    const matrixNxP& DynamicRobot::currentTorques () const
    {
      return torques_;
    }

    const vectorN& DynamicRobot::currentJointTorques () const
    {
      return jointTorques_;
    }

    bool DynamicRobot::computeForwardKinematics ()
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    bool DynamicRobot::computeCenterOfMassDynamics ()
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    const vector3d& DynamicRobot::positionCenterOfMass () const
    {
      return positionCenterOfMass_;
    }

    const vector3d& DynamicRobot::velocityCenterOfMass ()
    {
      return velocityCenterOfMass_;
    }

    const vector3d& DynamicRobot::accelerationCenterOfMass ()
    {
      return accelerationCenterOfMass_;
    }

    const vector3d& DynamicRobot::linearMomentumRobot ()
    {
      return linearMomentumRobot_;
    }

    const vector3d& DynamicRobot::derivativeLinearMomentum ()
    {
      return derivativeLinearMomentum_;
    }

    const vector3d& DynamicRobot::angularMomentumRobot ()
    {
      return angularMomentumRobot_;
    }

    const vector3d& DynamicRobot::derivativeAngularMomentum ()
    {
      return derivativeAngularMomentum_;
    }

    double DynamicRobot::mass () const
    {
      return mass_;
    }

    bool DynamicRobot::isSupported (const std::string& key)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    bool DynamicRobot::getProperty (const std::string & key,
				    std::string& value) const
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    bool DynamicRobot::setProperty (std::string& key,
				    const std::string& value)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    bool
    DynamicRobot::getJacobian (const CjrlJoint& inStartJoint,
			       const CjrlJoint& inEndJoint,
			       const vector3d& inFrameLocalPosition,
			       matrixNxP& outjacobian,
			       unsigned int offset,
			       bool inIncludeStartFreeFlyer)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    bool
    DynamicRobot::getPositionJacobian (const CjrlJoint& inStartJoint,
				       const CjrlJoint& inEndJoint,
				       const vector3d& inFrameLocalPosition,
				       matrixNxP& outjacobian,
				       unsigned int offset,
				       bool inIncludeStartFreeFlyer)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    bool
    DynamicRobot::getOrientationJacobian (const CjrlJoint& inStartJoint,
					  const CjrlJoint& inEndJoint,
					  matrixNxP& outjacobian,
					  unsigned int offset,
					  bool inIncludeStartFreeFlyer)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    bool DynamicRobot::getJacobianCenterOfMass (const CjrlJoint& inStartJoint,
						matrixNxP& outjacobian,
						unsigned int offset,
						bool inIncludeStartFreeFlyer)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");      
    }

    void DynamicRobot::computeInertiaMatrix ()
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    const matrixNxP& DynamicRobot::inertiaMatrix () const
    {
      return inertiaMatrix_;
    }

    const std::vector<to_pointer<CjrlJoint>::type >&
    DynamicRobot::getActuatedJoints () const
    {
      ardJointShPtrs_t ardJoints (actuatedJoints_.size ());
      for (unsigned i = 0; i < actuatedJoints_.size (); ++i)
	ardJoints[i] = getSharedPointer (actuatedJoints_[i]);
      return ardJoints;
    }
 
    void
    DynamicRobot::setActuatedJoints
    (std::vector<to_pointer<CjrlJoint>::type >& actuatedJoints)
    {
      actuatedJoints_.clear ();
      BOOST_FOREACH (ardJointShPtr_t ardJoint, actuatedJoints)
	{
	  jointWkPtr_t jointWkPtr;
	  getPtrFromBase (jointWkPtr, ardJoint);
	  actuatedJoints_.push_back (jointWkPtr);
	}
    }

    void DynamicRobot::rootJoint (joint_t& joint)
    {
      rootJoint_ = joint.shared_from_this ();
    }

  } // end of namespace rbdl.
} // end of namespace ard.
