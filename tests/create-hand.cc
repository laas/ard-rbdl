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

#define BOOST_TEST_MODULE create-joint

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/hand.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

BOOST_AUTO_TEST_CASE (create_joint)
{
  using namespace ard::rbdl;

  // Create hand with default constructor.
  ardHandShPtr_t hand (new hand_t);
  
  BOOST_CHECK_EQUAL (!hand, 0);  

  // Create hand using constructor with ard joint.
  ardJointShPtr_t joint1 (new joint_t);
  
  ardHandShPtr_t hand1 (new hand_t (joint1));
  
  BOOST_CHECK_EQUAL (!hand1, 0);

  // Create hand using constructor with joint.
  jointShPtr_t joint2 (new joint_t);
  
  ardHandShPtr_t hand2 (new hand_t (joint2));
  
  BOOST_CHECK_EQUAL (!hand2, 0);
  
  // Create hand from factory.
  RobotDynamicsObjectFactory factory = RobotDynamicsObjectFactory ();
  
  ardHandShPtr_t handFromFactory = factory.createHand (joint1);

  BOOST_CHECK_EQUAL (!handFromFactory, 0);

  // Create hand with copy constructor.
  handShPtr_t handDynCast;
  getPtrFromBase (handDynCast, hand);
  ardHandShPtr_t handCopy (new hand_t (*handDynCast));

  BOOST_CHECK_EQUAL (!handCopy, 0);
}
