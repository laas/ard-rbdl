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


/// \file src/model/hand.cc
/// \brief Implementation of Hand.

#include <stdexcept>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/hand.hh>

namespace ard
{
  namespace rbdl
  {
    Hand::Hand () :
      boost::enable_shared_from_this<Hand> (),
      wristJoint_ (),
      center_ (),
      thumbAxis_ (),
      foreFingerAxis_ (),
      palmNormal_ ()
    {
    }

    Hand::Hand (const jointShPtr_t& wrist) :
      boost::enable_shared_from_this<Hand> (),
      wristJoint_ (wrist),
      center_ (),
      thumbAxis_ (),
      foreFingerAxis_ (),
      palmNormal_ ()
    {
    }

    Hand::~Hand ()
    {
    }

    CjrlJoint* Hand::associatedWrist () const
    {
      return getUnsafePointer (wristJoint_);
    }

    void Hand::setAssociatedWrist (CjrlJoint* joint)
    {
      jointPtr_t jointPtr = dynamic_cast<jointPtr_t> (joint);
      if (jointPtr)
	wristJoint_ = jointWkPtr_t (jointPtr->shared_from_this ());
      else
	throw std::runtime_error ("Null pointer to joint.");
    }

    void Hand::getCenter (vector3d& center) const
    {
      center = center_;
    }

    void Hand::setCenter (const vector3d& center)
    {
      center_ = center;
    }

    void Hand::getThumbAxis (vector3d& thumbAxis) const
    {
      thumbAxis = thumbAxis_;
    }

    void Hand::setThumbAxis (const vector3d& thumbAxis)
    {
      thumbAxis_ = thumbAxis;
    }

    void Hand::getForeFingerAxis (vector3d& foreFingerAxis) const
    {
      foreFingerAxis = foreFingerAxis_;
    }

    void Hand::setForeFingerAxis (const vector3d& foreFingerAxis)
    {
      foreFingerAxis_ = foreFingerAxis;
    }

    void Hand::getPalmNormal (vector3d& palmNormal) const
    {
      palmNormal = palmNormal_;
    }

    void Hand::setPalmNormal (const vector3d& palmNormal)
    {
      palmNormal_ = palmNormal;
    }

  } // end of namespace rbdl.
} // end of namespace ard.
