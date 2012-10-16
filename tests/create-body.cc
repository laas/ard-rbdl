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

#define BOOST_TEST_MODULE create-body

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/body.hh>
#include <ard/rbdl/model/robot-dynamics-object-factory.hh>

BOOST_AUTO_TEST_CASE (create_body)
{
  using namespace ard::rbdl;

  // Create body using constructor.
  double mass = 1;
  vector3d com (0,0,0);
  matrix3d inertia;
  inertia.setIdentity ();
  jointShPtr_t joint (new joint_t);
  
  ardBodyShPtr_t body (new body_t (mass, com, inertia, *joint));
  
  BOOST_CHECK_EQUAL (!body, 0);
  
  // Create body from factory.
  RobotDynamicsObjectFactory factory = RobotDynamicsObjectFactory ();
  
  ardBodyShPtr_t bodyFromFactory = factory.createBody ();

  BOOST_CHECK_EQUAL (!bodyFromFactory, 0);

  // Create body with copy constructor.
  bodyShPtr_t bodyDynCast;
  getPtrFromBase (bodyDynCast, body);
  ardBodyShPtr_t bodyCopy (new body_t (*bodyDynCast));

  BOOST_CHECK_EQUAL (!bodyCopy, 0);

  // Create body by assignment.
  body_t bodyAssigned = *bodyDynCast;
}
