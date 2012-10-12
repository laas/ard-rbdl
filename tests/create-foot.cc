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

#define BOOST_TEST_MODULE create-foot

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/foot.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

BOOST_AUTO_TEST_CASE (create_foot)
{
  using namespace ard::rbdl;

  // Create foot with default constructor.
  ardFootShPtr_t foot (new foot_t);
  
  BOOST_CHECK_EQUAL (!foot, 0);  

  // Create foot using constructor with ard joint.
  ardJointShPtr_t joint1 (new joint_t);
  
  ardFootShPtr_t foot1 (new foot_t (joint1));
  
  BOOST_CHECK_EQUAL (!foot1, 0);

  // Create foot using constructor with joint.
  jointShPtr_t joint2 (new joint_t);
  
  ardFootShPtr_t foot2 (new foot_t (joint2));
  
  BOOST_CHECK_EQUAL (!foot2, 0);
  
  // Create foot from factory.
  RobotDynamicsObjectFactory factory = RobotDynamicsObjectFactory ();
  
  ardFootShPtr_t footFromFactory = factory.createFoot (joint1);

  BOOST_CHECK_EQUAL (!footFromFactory, 0);

  // Create foot with copy constructor.
  footShPtr_t footDynCast;
  getPtrFromBase (footDynCast, foot);
  ardFootShPtr_t footCopy (new foot_t (*footDynCast));

  BOOST_CHECK_EQUAL (!footCopy, 0);
}
