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

# include <jrl/mathtools/vector3.hh>
# include <jrl/mathtools/matrix3x3.hh>
# include <jrl/mathtools/matrix4x4.hh>
# include <boost/numeric/ublas/vector.hpp>
# include <boost/numeric/ublas/matrix.hpp>

typedef jrlMathTools::Vector3D<double> vector3d;
typedef jrlMathTools::Matrix3x3<double> matrix3d;
typedef jrlMathTools::Matrix4x4<double> matrix4d;
typedef boost::numeric::ublas::vector<double> vectorN;
typedef boost::numeric::ublas::matrix<double> matrixNxP;

# include <abstract-robot-dynamics/traits/shared-pointer.hh>
# include <abstract-robot-dynamics/abstract-robot-dynamics.hh>

# include <ard/rbdl/tools/fwd.hh>

namespace ard
{
  namespace rbdl
  {
    // Define gravity vector.
    static const vector3d defaultGravity (0, 0, -9.81);

    // RBDL typedefs.
    ARD_RBDL_DEFINE_TYPES (RigidBodyDynamics::Joint, rbdlJoint);
    ARD_RBDL_DEFINE_TYPES (RigidBodyDynamics::Body, rbdlBody);
    ARD_RBDL_DEFINE_TYPES (RigidBodyDynamics::Model, rbdlModel);

    // RBDL Math typedefs.
    typedef RigidBodyDynamics::Math::Vector3d rbdlVector3d_t;
    typedef RigidBodyDynamics::Math::Matrix3d rbdlMatrix3d_t;
    typedef Eigen::Matrix<double, 4, 4> rbdlMatrix4d_t;
    typedef RigidBodyDynamics::Math::VectorNd rbdlVectorN_t;
    typedef RigidBodyDynamics::Math::MatrixNd rbdlMatrixNxP_t;
    typedef RigidBodyDynamics::Math::SpatialTransform rbdlSpatialTransform_t;
    typedef RigidBodyDynamics::Math::SpatialVector rbdlSpatialVector_t;
    
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
