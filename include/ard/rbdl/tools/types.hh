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

/// \brief Declarations of types.

#ifndef ARD_RBDL_TYPES_HH
# define ARD_RBDL_TYPES_HH

# include <boost/shared_ptr.hpp>
# include <boost/weak_ptr.hpp>

# include <rbdl/rbdl.h>

// Math typedefs.
typedef RigidBodyDynamics::Math::Vector3d vector3d;
typedef RigidBodyDynamics::Math::Matrix3d matrix3d;
typedef Eigen::Matrix<double, 4, 4> matrix4d;
typedef RigidBodyDynamics::Math::VectorNd vectorN;
typedef RigidBodyDynamics::Math::MatrixNd matrixNxP;
typedef RigidBodyDynamics::Math::SpatialTransform rbdlSpatialTransform_t;
typedef RigidBodyDynamics::Math::SpatialVector rbdlSpatialVector_t;

# include <abstract-robot-dynamics/traits/shared-pointer.hh>
# include <abstract-robot-dynamics/abstract-robot-dynamics.hh>

# include <ard/rbdl/tools/fwd.hh>

namespace ard
{
  namespace rbdl
  {
    // Define gravity vector.
    static const vector3d gravity (0, 0, -9.81);

    // RBDL typedefs.
    ARD_RBDL_DEFINE_TYPES (RigidBodyDynamics::Joint, rbdlJoint);
    ARD_RBDL_DEFINE_TYPES (RigidBodyDynamics::Body, rbdlBody);
    ARD_RBDL_DEFINE_TYPES (RigidBodyDynamics::Model, rbdlModel);
    
    // Abstract robot dynamics typedefs.
    ARD_RBDL_DEFINE_TYPES (CjrlJoint, ardJoint);
    ARD_RBDL_DEFINE_TYPES (CjrlBody, ardBody);
    ARD_RBDL_DEFINE_TYPES (CjrlDynamicRobot, ardDynamicRobot);
    ARD_RBDL_DEFINE_TYPES (CjrlHand, ardHand);
    ARD_RBDL_DEFINE_TYPES (CjrlFoot, ardFoot);
    ARD_RBDL_DEFINE_TYPES (CjrlHumanoidDynamicRobot, ardHumanoidDynamicRobot);
    typedef CjrlRigidVelocity ardVelocity_t;
    typedef CjrlRigidAcceleration ardAcceleration_t;

    // Define enum for joint types.
    enum JointType {
      JOINT_TYPE_UNDEFINED = 0,
      JOINT_TYPE_FIXED,
      JOINT_TYPE_REVOLUTE,
      JOINT_TYPE_PRISMATIC,
      JOINT_TYPE_FREEFLYER
    };

  } // end of namespace rbdl.
} // end of namespace ard.
  
#endif //! ARD_RBDL_TYPES_HH
