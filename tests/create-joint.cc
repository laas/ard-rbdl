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

#include <ard/rbdl/tools/pointer-util.hh>
#include <ard/rbdl/model/joint.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

BOOST_AUTO_TEST_CASE (create_joint)
{
  using namespace ard::rbdl;

  // Create joint with defaut constructor.
  ardJointShPtr_t joint (new joint_t ());

  BOOST_CHECK_EQUAL (!joint, 0);

  // Create joints from factory.
  RobotDynamicsObjectFactory factory = RobotDynamicsObjectFactory ();
  matrix4d mat;
  mat.setIdentity ();
  
  ardJointShPtr_t jointRotation = factory.createJointRotation (mat);
  ardJointShPtr_t jointTranslation = factory.createJointTranslation (mat);
  ardJointShPtr_t jointFreeflyer = factory.createJointFreeflyer (mat);

  BOOST_CHECK_EQUAL (!jointRotation, 0);
  BOOST_CHECK_EQUAL (!jointTranslation, 0);
  BOOST_CHECK_EQUAL (!jointFreeflyer, 0);

  // Create joint with copy constructor.
  jointShPtr_t jointRotationDynCast;
  getPtrFromBase (jointRotationDynCast, jointRotation);
  ardJointShPtr_t jointRotationCopy (new joint_t (*jointRotationDynCast));

  BOOST_CHECK_EQUAL (!jointRotationCopy, 0);

  // Create joint by assignment.
  joint_t jointRotationAssigned = *jointRotationDynCast;
}
