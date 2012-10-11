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


/// \file src/model/joint.cc
/// \brief Implementation of Joint.

#include <stdexcept>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/joint.hh>

namespace ard
{
  namespace rbdl
  {
    Joint::Joint () :
      boost::enable_shared_from_this<Joint> (),
      name_ (),
      parentJoint_ (),
      childJoints_ (),
      rbdlJoint_ (),
      rankInConfiguration_ (),
      initialPosition_ (),
      currentTransformation_ (),
      velocity_ (),
      acceleration_ (vector3d (0, 0, 0), vector3d (0, 0, 0)),
      numberDof_ (),
      lowerBound_ (),
      upperBound_ (),
      lowerVelocityBound_ (),
      upperVelocityBound_ (),
      lowerTorqueBound_ (),
      upperTorqueBound_ (),
      jacobian_ (),
      linkedBody_ ()
    {
    }
    
    Joint::Joint (const JointType jointType, const matrix4d& initialPosition) :
      boost::enable_shared_from_this<Joint> (),
      name_ (),
      parentJoint_ (),
      childJoints_ (),
      rbdlJoint_ (),
      rankInConfiguration_ (),
      initialPosition_ (initialPosition),
      currentTransformation_ (),
      velocity_ (),
      acceleration_ (vector3d (0, 0, 0), vector3d (0, 0, 0)),
      numberDof_ (),
      lowerBound_ (),
      upperBound_ (),
      lowerVelocityBound_ (),
      upperVelocityBound_ (),
      lowerTorqueBound_ (),
      upperTorqueBound_ (),
      jacobian_ (),
      linkedBody_ ()
    {
      switch (jointType)
	{
	case JOINT_TYPE_FIXED:
	  {
	    throw std::runtime_error ("Fixed joints not handled yet by rbdl.");
	    break;
	  }
	case JOINT_TYPE_REVOLUTE:
	  {
	    rbdlJoint_ = rbdlJoint_t (RigidBodyDynamics::JointTypeRevolute,
				      vector3d (1, 0, 0));
	    break;
	  }
	case JOINT_TYPE_PRISMATIC:
	  {
	    rbdlJoint_ = rbdlJoint_t (RigidBodyDynamics::JointTypePrismatic,
				      vector3d (1, 0, 0));
	    break;
	  }
	case JOINT_TYPE_FREEFLYER:
	  {
	    throw std::runtime_error ("Freeflyer joints not handled yet.");
	    break;
	  }
	default:
	  throw std::runtime_error ("Incorrect joint type");
	}
    }

    Joint::Joint (Joint& joint):
      boost::enable_shared_from_this<Joint> (),
      acceleration_ (vector3d (0, 0, 0), vector3d (0, 0, 0))
    {
      name_ = joint.getName ();
      
      jointShPtr_t parentJointShPtr;
      getPtrFromBase (parentJointShPtr, joint.parentJoint ());
      if (parentJointShPtr)
	setParentJoint (parentJointShPtr);
      else
	throw std::runtime_error ("Null pointer to parent joint.");

      for (unsigned i = 0; i < joint.countChildJoints (); ++i)
	addChildJoint (*(joint.childJoint (i)));

      rbdlJoint_ = joint.rbdlJoint ();
      rankInConfiguration_ = joint.rankInConfiguration ();
      initialPosition_ = joint.initialPosition ();
      currentTransformation_ = joint.currentTransformation ();
      velocity_ = joint.jointVelocity();
      acceleration_ = joint.jointAcceleration ();
      numberDof_ = joint.numberDof ();

      lowerBound_.resize (joint.numberDof ());
      upperBound_.resize (joint.numberDof ());
      lowerVelocityBound_.resize (joint.numberDof ());
      upperVelocityBound_.resize (joint.numberDof ());
      lowerTorqueBound_.resize (joint.numberDof ());
      upperTorqueBound_.resize (joint.numberDof ());
      for (unsigned i = 0; i < joint.numberDof (); ++i)
	{
	  lowerBound_[i] = joint.lowerBound (i);
	  upperBound_[i] = joint.upperBound (i);
	  lowerVelocityBound_[i] = joint.lowerVelocityBound (i);
	  upperVelocityBound_[i] = joint.upperVelocityBound (i);
	  lowerTorqueBound_[i] = joint.lowerTorqueBound (i);
	  upperTorqueBound_[i] = joint.upperTorqueBound (i);
	}

      jacobian_ = joint.jacobianJointWrtConfig ();

      bodyShPtr_t linkedBodyShPtr;
      getPtrFromBase (linkedBodyShPtr, joint.linkedBody ());
      if (linkedBodyShPtr)
	setLinkedBody (*linkedBodyShPtr);
      else
	throw std::runtime_error ("Null pointer to linked body.");
    }
    
    Joint::~Joint ()
    {
    }

    const std::string& Joint::getName () const
    {
      return name_;
    }

    void Joint::setName (const std::string& name)
    {
      name_ = name;
    }

    to_pointer<CjrlJoint>::type Joint::parentJoint () const
    {
      return getSharedPointer (parentJoint_);
    }

    bool Joint::addChildJoint (CjrlJoint& joint)
    {
      // Link joints in abstract robot dynamics. The rbdl joints
      // will be linked later during initialization.
      jointPtr_t jointPtr;
      getPtrFromBase (jointPtr, &joint);
      if (jointPtr)
	return addChildJoint (*jointPtr);
      else
	return false;
    }

    unsigned int Joint::countChildJoints() const
    {
      return childJoints_.size ();
    }

    to_pointer<CjrlJoint>::type Joint::childJoint (unsigned int jointRank) const
    {
      return childJoints_[jointRank];
    }

    rbdlJoint_t Joint::rbdlJoint () const
    {
      return rbdlJoint_;
    }

    std::vector<to_pointer<CjrlJoint>::type> Joint::jointsFromRootToThis () const
    {
      ardJointShPtrs_t fromRootToThis;

      // Update vector of joints going starting from root joint.
      // Const cast this joint pointer because method returns a vector
      // of non-const pointers.
      jointShPtr_t ptr
	= boost::const_pointer_cast<joint_t> (shared_from_this ());
      fromRootToThis.push_back (ptr);
      ardJointShPtr_t parentJoint = getSharedPointer (parentJoint_);
      while (parentJoint != 0)
	{
	  fromRootToThis.insert(fromRootToThis.begin (), parentJoint);
	  parentJoint = parentJoint->parentJoint ();
	}
      
      return fromRootToThis;
    }

    unsigned int Joint::rankInConfiguration () const
    {
      return rankInConfiguration_;
    }

    const matrix4d& Joint::initialPosition () const
    {
      return initialPosition_;
    }

    bool Joint::updateTransformation (const vectorN& dofVector)
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    const matrix4d& Joint::currentTransformation () const
    {
      return currentTransformation_;
    }

    CjrlRigidVelocity Joint::jointVelocity () const
    {
      return velocity_;
    }

    CjrlRigidAcceleration Joint::jointAcceleration () const
    {
      return acceleration_;
    }

    unsigned int Joint::numberDof () const
    {
      return numberDof_;
    }

    double Joint::lowerBound (unsigned int dofRank) const
    {
      return lowerBound_[dofRank];
    }

    double Joint::upperBound (unsigned int dofRank) const
    {
      return upperBound_[dofRank];
    }

    void Joint::lowerBound (unsigned int dofRank, double lowerBound)
    {
      lowerBound_[dofRank] = lowerBound;
    }

    void Joint::upperBound (unsigned int dofRank, double upperBound)
    {
      upperBound_[dofRank] = upperBound;
    }

    double Joint::lowerVelocityBound (unsigned int dofRank) const
    {
      return lowerVelocityBound_[dofRank];
    }

    double Joint::upperVelocityBound (unsigned int dofRank) const
    {
      return upperVelocityBound_[dofRank];
    }

    void Joint::lowerVelocityBound (unsigned int dofRank, double lowerBound)
    {
      lowerVelocityBound_[dofRank] = lowerBound;
    }

    void Joint::upperVelocityBound (unsigned int dofRank, double upperBound)
    {
      upperVelocityBound_[dofRank] = upperBound;
    }

    double Joint::lowerTorqueBound (unsigned int dofRank) const
    {
      return lowerTorqueBound_[dofRank];
    }

    double Joint::upperTorqueBound (unsigned int dofRank) const
    {
      return upperTorqueBound_[dofRank];
    }

    void Joint::lowerTorqueBound (unsigned int dofRank, double lowerBound)
    {
      lowerTorqueBound_[dofRank] = lowerBound;
    }

    void Joint::upperTorqueBound (unsigned int dofRank, double upperBound)
    {
      upperTorqueBound_[dofRank] = upperBound;
    }

    const matrixNxP& Joint::jacobianJointWrtConfig () const
    {
      return jacobian_;
    }

    void Joint::computeJacobianJointWrtConfig ()
    {
      getJacobianPointWrtConfig (vector3d (0,0,0), jacobian_);
    }

    void Joint::getJacobianPointWrtConfig
    (const vector3d& inPointJointFrame, matrixNxP& outjacobian) const
    {
      // We assume this method will not be called.
      throw std::runtime_error ("Method not supported.");
    }

    to_pointer<CjrlBody>::type Joint::linkedBody () const
    {
      return linkedBody_;
    }

    void Joint::setLinkedBody (CjrlBody& body)
    {
      bodyPtr_t bodyPtr;
      getPtrFromBase (bodyPtr, &body);
      if (bodyPtr)
	return setLinkedBody (*bodyPtr);
      else
	throw std::runtime_error ("Null pointer to body.");
    }

    void Joint::setParentJoint (jointShPtr_t joint)
    {
      assert (!!joint && "Null pointer to joint.");
      parentJoint_ = jointWkPtr_t (joint);
    }

    bool Joint::addChildJoint (joint_t& joint)
    {
      jointShPtr_t jointShPtr = joint.shared_from_this ();
      if (!isJointInVector (jointShPtr, childJoints_))
	{
	  jointShPtr->setParentJoint (shared_from_this ());
	  childJoints_.push_back (jointShPtr);
	  return true;
	}
      return false;
    }

    void Joint::setLinkedBody (body_t& body)
    {
      linkedBody_ = body.shared_from_this ();
    }

  } // end of namespace rbdl.
} // end of namespace ard.
