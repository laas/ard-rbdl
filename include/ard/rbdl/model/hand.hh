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

/// \brief Declaration of Hand class.

#ifndef ARD_RBDL_MODEL_HAND_HH
# define ARD_RBDL_MODEL_HAND_HH

# include <boost/enable_shared_from_this.hpp>

# include <abstract-robot-dynamics/hand.hh>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <ard/rbdl/model/joint.hh>

namespace ard
{
  namespace rbdl
  {
    class Hand :
      public CjrlHand,
      public boost::enable_shared_from_this<Hand>
    {
    public:
      /// \brief Default constructor.
      Hand ();

      /// \brief Destructor.
      virtual ~Hand ();

      /// \brief Get the wrist joint to which the hand is attached.
      virtual CjrlJoint* associatedWrist () const;

      /// \brief Get the wrist joint to which the hand is attached.
      virtual void setAssociatedWrist (CjrlJoint* inJoint );

      /// \brief Get the center of the hand
      ///
      /// \retval outCenter Center of the hand in the frame of the
      /// wrist.
      virtual void getCenter (vector3d& outCenter) const;

      /// \brief Set the center of the hand
      ///
      /// \param inCenter Center of the hand in the frame of the
      /// wrist.
      virtual void setCenter (const vector3d& inCenter);

      /// \brief Get thumb axis when had is in open position
      ///
      /// \retval outThumbAxis Axis of the thumb in wrist frame in
      /// open position
      virtual void getThumbAxis (vector3d& outThumbAxis) const;

      /// \brief Set thumb axis in wrist frame when had is in open
      /// position
      ///
      /// \param inThumbAxis Axis of the thumb in wrist frame in open
      /// position
      virtual void setThumbAxis (const vector3d& inThumbAxis);

      /// \brief Get forefinger axis.
      ///
      /// \retval outForeFingerAxis axis of the forefinger in wrist
      /// frame in open position.
      virtual void getForeFingerAxis (vector3d& outForeFingerAxis) const;

      /// \brief Set forefinger axis.
      ///
      /// \param inForeFingerAxis axis of the forefinger in wrist
      ///                         frame in open position.
      virtual void setForeFingerAxis (const vector3d& inForeFingerAxis);

      /// \brief Get palm normal
      ///
      /// \retval outPalmNormal normal to the palm in the frame of the
      /// wrist.
      virtual void getPalmNormal (vector3d& outPalmNormal) const;

      /// \brief Set palm normal
      ///
      /// \param inPalmNormal normal to the palm in the frame of the
      /// wrist.
      virtual void setPalmNormal (const vector3d& inPalmNormal);

    private:
      jointWkPtr_t wristJoint_;
      vector3d center_;
      vector3d thumbAxis_;
      vector3d foreFingerAxis_;
      vector3d palmNormal_;
    };

  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_HAND_HH
