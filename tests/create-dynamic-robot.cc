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

#define BOOST_TEST_MODULE create-dynamic-robot

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <ard/rbdl/tools/pointer-util.hh>
#include <ard/rbdl/model/dynamic-robot.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

BOOST_AUTO_TEST_CASE (create_dynamic_robot)
{
  using namespace ard::rbdl;

  // Create dynamic robot with defaut constructor.
  ardDynamicRobotShPtr_t robotDefault (new dynamicRobot_t);

  BOOST_CHECK_EQUAL (!robotDefault, 0);

  // Create joints from factory.
  RobotDynamicsObjectFactory factory = RobotDynamicsObjectFactory ();
  
  ardDynamicRobotShPtr_t robot = factory.createDynamicRobot ();

  BOOST_CHECK_EQUAL (!robot, 0);

  // Create dynamic robot with copy constructor.
  dynamicRobotShPtr_t robotDynCast;
  getPtrFromBase (robotDynCast, robot);

  ardDynamicRobotShPtr_t robotCopy (new dynamicRobot_t (*robotDynCast));

  BOOST_CHECK_EQUAL (!robotCopy, 0);

  // Create dynamic robot by assignment.
  dynamicRobot_t robotAssigned = *robotDynCast;
}
