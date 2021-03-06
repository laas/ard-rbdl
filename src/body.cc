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


/// \file src/model/body.cc
/// \brief Implementation of Body.

#include <stdexcept>

#include <ard/rbdl/tools/pointer-util.hh>
#include <ard/rbdl/tools/math-util.hh>
#include <ard/rbdl/model/body.hh>

namespace ard
{
  namespace rbdl
  {
    Body::Body () :
      boost::enable_shared_from_this<Body> (),
      name_ (),
      rbdlBody_ (),
      joint_ ()
    {
    }

    Body::Body (const double& mass,
		const vector3d& com,
		const matrix3d& inertia,
		joint_t& joint) : 
      boost::enable_shared_from_this<Body> (),
      name_ (),
      rbdlBody_ (),
      joint_ ()
    {
      rbdlBody_ = rbdlBody_t (mass,
			      toRbdlFromMal (com),
			      toRbdlFromMal (inertia));
      jointShPtr_t shPtr = joint.shared_from_this ();
      joint_ = jointWkPtr_t (shPtr);
    }

    Body::Body (const Body& body) :
      boost::enable_shared_from_this<Body> ()
    {
      name_ = body.getName ();
      rbdlBody_ = body.rbdlBody ();
      body.joint (joint_);
    }

    Body Body::operator= (const Body& body)
    {
      name_ = body.getName ();
      rbdlBody_ = body.rbdlBody ();
      body.joint (joint_);

      return *this;
    }

    Body::~Body ()
    {
    }

    rbdlBody_t Body::rbdlBody () const
    {
      return rbdlBody_;
    }

    std::string Body::getName () const
    {
      return name_;
    }

    void Body::setName (const std::string& name)
    {
      name_ = name;
    }

    const vector3d& Body::localCenterOfMass () const
    {
      return toMalFromRbdl (rbdlBody_.mCenterOfMass);
    }

    void Body::localCenterOfMass (const vector3d& localCenterOfMass)
    {
      // Recreate rbdl body with new center of mass as spatial inertia
      // will also be modified.
      // FIXME
      throw std::runtime_error ("Method not supported.");      
    }

    const matrix3d& Body::inertiaMatrix () const
    {
      // Some processing needs to be done on the spatial inertia matrix.
      // FIXME
      throw std::runtime_error ("Method not supported.");      
    }
    
    void Body::inertiaMatrix (const matrix3d& inertiaMatrix)
    {
      // Recreate rbdl body as spatial inertia will be modified.
      rbdlBody_ = rbdlBody_t (rbdlBody_.mMass,
			      rbdlBody_.mCenterOfMass,
			      toRbdlFromMal (inertiaMatrix));
    }

    double Body::mass () const
    {
      return rbdlBody_.mMass;
    }

    void Body::mass (double inMass)
    {
      // Recreate body as center of mass and spatial inertia matrix
      // will be modified.
      // FIXME
      throw std::runtime_error ("Method not supported.");
    }

    to_pointer<const CjrlJoint>::type Body::joint () const
    {
      return getSharedPointer (joint_);
    }

    void Body::joint (jointWkPtr_t& joint) const
    {
      joint = joint_;
    }
  
  } // end of namespace rbdl.
} // end of namespace ard.
