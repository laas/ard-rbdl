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

#include "ard/rbdl/tools/util.hh"
#include "ard/rbdl/model/body.hh"

namespace ard
{
  namespace rbdl
  {
    Body::Body (const double& mass,
		const vector3d& com,
		const matrix3d& inertia,
		joint_t& joint) : 
      rbdlBody_ (mass, com, inertia)
    {
      jointShPtr_t jointPtr (&joint);
      joint_ = jointWkPtr_t (jointPtr);
    }

    Body::~Body ()
    {
    }

    const vector3d& Body::localCenterOfMass () const
    {
      return rbdlBody_.mCenterOfMass;
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
			      inertiaMatrix);
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

    const CjrlJoint* Body::joint () const
    {
      
      return (joint_.lock ()).get ();
    }

  } // end of namespace rbdl.
} // end of namespace ard.
