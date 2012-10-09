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

/// \brief Declaration of Body class.

#ifndef ARD_RBDL_MODEL_BODY_HH_
# define ARD_RBDL_MODEL_BODY_HH_

# include <rbdl/Body.h>

# include "ard/rbdl/tools/types.hh"
# include <abstract-robot-dynamics/body.hh>

namespace ard
{
  namespace rbdl
  {
    class Body : public CjrlBody
    {
    public:
      /// \brief Constructor.
      ///
      /// \param mass Body mass
      /// \param com Center of mass in body frame.
      /// \param inertia Inertia matrix in body frame.
      /// \param joint Parent joint.
      Body (const double& mass,
	    const vector3d& com,
	    const matrix3d& inertia,
	    joint_t& joint);

      /// \brief Destructor.
      virtual ~Body ();

      /// \brief Get position of center of mass in joint local
      /// reference frame.
      virtual const vector3d& localCenterOfMass () const;

      /// \brief Set postion of center of mass in joint reference
      /// frame.
      virtual void localCenterOfMass (const vector3d& inlocalCenterOfMass);

      /// \brief Get Intertia matrix expressed in joint local
      /// reference frame.
      virtual const matrix3d& inertiaMatrix () const;

      /// \brief Set inertia matrix.
      virtual void inertiaMatrix (const matrix3d& inInertiaMatrix);

      /// \brief Get mass.
      virtual double mass () const;

      /// \brief Set mass.
      virtual void mass (double inMass);

      /// \brief Get const pointer to the joint the body is attached
      /// to.
      virtual const CjrlJoint* joint () const;
	
    private:
      /// \brief rbdl body attribute.
      rbdlBody_t rbdlBody_;
      /// \brief Weak pointer to parent joint attribute.
      jointWkPtr_t joint_;
    };
  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_BODY_HH
