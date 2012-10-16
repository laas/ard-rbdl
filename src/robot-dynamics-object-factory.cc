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


/// \file src/model/robot-dynamics-object-factory.cc
/// \brief Implementation of RobotDynamicsObjectFactory.

#include <stdexcept>

#include <boost/make_shared.hpp>

#include <ard/rbdl/tools/pointer-util.hh>
#include <ard/rbdl/model/joint.hh>
#include <ard/rbdl/model/body.hh>
#include <ard/rbdl/model/dynamic-robot.hh>
#include <ard/rbdl/model/hand.hh>
#include <ard/rbdl/model/foot.hh>
#include <ard/rbdl/model/humanoid-dynamic-robot.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

namespace ard
{
  namespace rbdl
  {
    RobotDynamicsObjectFactory::RobotDynamicsObjectFactory () :
      boost::enable_shared_from_this<RobotDynamicsObjectFactory> ()
    {
    }

    RobotDynamicsObjectFactory::~RobotDynamicsObjectFactory ()
    {
    }

    to_pointer<CjrlDynamicRobot>::type
    RobotDynamicsObjectFactory::createDynamicRobot ()
    {
      dynamicRobotShPtr_t shPtr (new dynamicRobot_t ());
      return shPtr;
    }

    to_pointer<CjrlHumanoidDynamicRobot>::type
    RobotDynamicsObjectFactory::createHumanoidDynamicRobot ()
    {
      humanoidDynamicRobotShPtr_t shPtr (new humanoidDynamicRobot_t ());
      return shPtr;
    }

    to_pointer<CjrlJoint>::type
    RobotDynamicsObjectFactory::createJointFreeflyer
    (const matrix4d& initialPosition)
    {
      jointShPtr_t shPtr (new joint_t (JOINT_TYPE_FREEFLYER, initialPosition));
      return shPtr;
    }

    to_pointer<CjrlJoint>::type
    RobotDynamicsObjectFactory::createJointRotation
    (const matrix4d& initialPosition)
    {
      jointShPtr_t shPtr (new joint_t (JOINT_TYPE_REVOLUTE, initialPosition));
      return shPtr;
    }

    to_pointer<CjrlJoint>::type
    RobotDynamicsObjectFactory::createJointTranslation
    (const matrix4d& initialPosition)
    {
      jointShPtr_t shPtr (new joint_t (JOINT_TYPE_PRISMATIC, initialPosition));
      return shPtr;
    }

    to_pointer<CjrlBody>::type RobotDynamicsObjectFactory::createBody ()
    {
      bodyShPtr_t shPtr (new body_t);
      return shPtr;
    }

    to_pointer<CjrlHand>::type
    RobotDynamicsObjectFactory::createHand (to_pointer<CjrlJoint>::type wrist)
    {
      handShPtr_t shPtr (new hand_t (wrist));
      return shPtr;
    }

    to_pointer<CjrlFoot>::type
    RobotDynamicsObjectFactory::createFoot (to_pointer<CjrlJoint>::type ankle)
    {
      footShPtr_t shPtr (new foot_t (ankle));
      return shPtr;
    }

  } // end of namespace rbdl.
} // end of namespace ard.
