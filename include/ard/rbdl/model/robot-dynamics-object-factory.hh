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

/// \brief Declaration of RobotDynamicsObjectFactory class.

#ifndef ARD_RBDL_MODEL_ROBOT_DYNAMICS_OBJECT_FACTORY_HH
# define ARD_RBDL_MODEL_ROBOT_DYNAMICS_OBJECT_FACTORY_HH

# include <boost/enable_shared_from_this.hpp>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

namespace ard
{
  namespace rbdl
  {
    class RobotDynamicsObjectFactory :
      public CjrlRobotDynamicsObjectFactory,
      public boost::enable_shared_from_this<RobotDynamicsObjectFactory>
    {
    public:
      /// \brief Default constructor.
      explicit RobotDynamicsObjectFactory ();

      /// \brief Destructor.
      virtual ~RobotDynamicsObjectFactory ();

      /// \brief Construct and return a pointer to a dynamic robot.
      virtual to_pointer<CjrlDynamicRobot>::type createDynamicRobot ();

      /// \brief Construct and return a pointer to a humanoid dynamic robot.
      virtual to_pointer<CjrlHumanoidDynamicRobot>::type
      createHumanoidDynamicRobot ();

      /// \brief Construct and return a pointer to a freeflyer joint.
      ///
      /// \param inInitialPosition position of the local frame of the
      /// joint when the robot is in initial configuration.
      virtual to_pointer<CjrlJoint>::type
      createJointFreeflyer (const matrix4d& inInitialPosition);

      /// \brief Construct and return a pointer to a rotation joint.
      ///
      /// \param inInitialPosition position of the local frame of the
      /// joint when the robot is in initial configuration.
      virtual to_pointer<CjrlJoint>::type
      createJointRotation (const matrix4d& inInitialPosition);

      /// \brief Construct and return a pointer to a translation joint.
      ///
      /// \param inInitialPosition position of the local frame of the
      /// joint when the robot is in initial configuration.
      virtual to_pointer<CjrlJoint>::type
      createJointTranslation (const matrix4d& inInitialPosition);

      /// \brief Construct and return a pointer to a body
      virtual to_pointer<CjrlBody>::type createBody ();

      /// \brief Construct and return a pointer to a hand.
      ///
      /// \param inWristJoint The joint the hand is attached to.
      virtual to_pointer<CjrlHand>::type createHand
      (to_pointer<CjrlJoint>::type inWristJoint);

      /// \brief Construct and return a pointer to a foot.
      ///
      /// \param inAnkle The joint the foot is attached to.
      virtual to_pointer<CjrlFoot>::type
      createFoot (to_pointer<CjrlJoint>::type inAnkle);
    };

  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_ROBOT_DYNAMICS_OBJECT_FACTORY_HH
