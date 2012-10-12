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


/// \file src/model/foot.cc
/// \brief Implementation of Foot.

#include <stdexcept>

#include <ard/rbdl/tools/util.hh>
#include <ard/rbdl/model/foot.hh>

namespace ard
{
  namespace rbdl
  {
    Foot::Foot () :
      boost::enable_shared_from_this<Foot> (),
      ankleJoint_ (),
      soleLength_ (),
      soleWidth_ (),
      ankleInLocalFrame_ ()
    {
    }

    Foot::Foot (const to_pointer<CjrlJoint>::type& ankle) : 
      boost::enable_shared_from_this<Foot> (),
      ankleJoint_ (),
      soleLength_ (),
      soleWidth_ (),
      ankleInLocalFrame_ ()
    {
      getPtrFromBase (ankleJoint_, ankle);
    }

    Foot::Foot (const jointShPtr_t& ankle) :
      boost::enable_shared_from_this<Foot> (),
      ankleJoint_ (ankle),
      soleLength_ (),
      soleWidth_ (),
      ankleInLocalFrame_ ()
    {
    }

    Foot::Foot (Foot& foot) :
      boost::enable_shared_from_this<Foot> ()
    {
      if (foot.associatedAnkle ())
	getPtrFromBase (ankleJoint_, foot.associatedAnkle ());
      else
	ankleJoint_.reset ();

      foot.getSoleSize (soleLength_, soleWidth_);
      foot.getAnklePositionInLocalFrame (ankleInLocalFrame_);
    }

    Foot::~Foot ()
    {
    }

    to_pointer<CjrlJoint>::type Foot::associatedAnkle () const
    {
      return getSharedPointer (ankleJoint_);
    }

    void Foot::setAssociatedAnkle (to_pointer<CjrlJoint>::type joint)
    {
      return getPtrFromBase (ankleJoint_, joint);
    }

    void Foot::getSoleSize (double& length, double& width) const
    {
      length = soleLength_;
      width = soleWidth_;
    }

    void Foot::setSoleSize (const double& length, const double& width)
    {
      soleLength_ = length;
      soleWidth_ = width;
    }

    void Foot::getAnklePositionInLocalFrame (vector3d& coordinates) const
    {
      coordinates = ankleInLocalFrame_;
    }

    void Foot::setAnklePositionInLocalFrame (const vector3d& coordinates)
    {
      ankleInLocalFrame_ = coordinates;
    }

  } // end of namespace rbdl.
} // end of namespace ard.
