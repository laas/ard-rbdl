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
      name_ (),
      parentJoint_ (),
      childJoints_ (),
      rbdlJoint_ (),
      fromRootToThis_ (),
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
    
    Joint::Joint (Joint& joint):
      acceleration_ (vector3d (0, 0, 0), vector3d (0, 0, 0))
    {
      name_ = joint.getName ();
      jointShPtr_t parentJoint (joint.parentJoint ());
      setParentJoint (parentJoint);

      for (unsigned i = 0; i < joint.countChildJoints (); ++i)
	addChildJoint (*(joint.childJoint (i)));

      rbdlJoint_ = joint.rbdlJoint ();
      fromRootToThis_ = joint.jointsFromRootToThis ();
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
      linkedBody_ = bodyShPtr_t (joint.linkedBody ());
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

    CjrlJoint* Joint::parentJoint () const
    {
      return (parentJoint_.lock ()).get ();
    }

    void Joint::setParentJoint (jointShPtr_t joint)
    {
      assert (!!joint && "Null pointer to joint.");
      jointWkPtr_t jointWkPtr (joint);
      parentJoint_ = jointWkPtr;

      // Update vector of joints going starting from root joint.
      fromRootToThis_.clear ();
      fromRootToThis_.push_back (this);
      jointPtr_t parentJoint = (parentJoint_.lock ()).get ();
      while (parentJoint != 0)
	{
	  fromRootToThis_.insert(fromRootToThis_.begin (), parentJoint);
	  parentJoint = parentJoint->parentJoint ();
	}
    }

    bool Joint::addChildJoint (CjrlJoint& joint)
    {
      if (!isJointInVector (joint, childJoints_))
	{
	  // Link joints in abstract robot dynamics. The rbdl joints
	  // will be linked later during initialization.
	  Joint* jointPtr = (Joint*)&joint;
	  jointPtr->setParentJoint (jointShPtr_t (this));
	  jointShPtr_t jointShPtr (jointPtr);
	  childJoints_.push_back (jointShPtr);
	  return true;
	}
      return false;
    }

    unsigned int Joint::countChildJoints() const
    {
      return childJoints_.size ();
    }

    CjrlJoint* Joint::childJoint (unsigned int jointRank) const
    {
      assert (!!childJoints_[jointRank] && "Null pointer to joint.");
      return childJoints_[jointRank].get ();
    }

    rbdlJoint_t Joint::rbdlJoint () const
    {
      return rbdlJoint_;
    }

    std::vector<CjrlJoint*> Joint::jointsFromRootToThis () const
    {
      return fromRootToThis_;
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

    CjrlBody* Joint::linkedBody () const
    {
      return linkedBody_.get ();
    }

    void Joint::setLinkedBody (CjrlBody& body)
    {
      bodyShPtr_t bodyPtr (&body);
      linkedBody_ = bodyPtr;
    }

  } // end of namespace rbdl.
} // end of namespace ard.
