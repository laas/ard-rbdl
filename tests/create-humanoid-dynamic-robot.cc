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

#define BOOST_TEST_MODULE create-humanoid-dynamic-robot

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/humanoid-dynamic-robot.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

BOOST_AUTO_TEST_CASE (create_humanoid_dynamic_robot)
{
  using namespace ard::rbdl;

  // Create dynamic robot with defaut constructor.
  ardHumanoidDynamicRobotShPtr_t robotDefault (new humanoidDynamicRobot_t);

  BOOST_CHECK_EQUAL (!robotDefault, 0);

  // Create joints from factory.
  RobotDynamicsObjectFactory factory = RobotDynamicsObjectFactory ();
  
  ardHumanoidDynamicRobotShPtr_t robot = factory.createHumanoidDynamicRobot ();

  BOOST_CHECK_EQUAL (!robot, 0);

  // Create dynamic robot with copy constructor.
  humanoidDynamicRobotShPtr_t robotDynCast;
  getPtrFromBase (robotDynCast, robot);

  ardHumanoidDynamicRobotShPtr_t robotCopy
    (new humanoidDynamicRobot_t (*robotDynCast));

  BOOST_CHECK_EQUAL (!robotCopy, 0);

  // Create dynamic robot by assignment.
  humanoidDynamicRobot_t robotAssigned = *robotDynCast;
}
