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

/// \brief Declaration of Foot class.

#ifndef ARD_RBDL_MODEL_FOOT_HH
# define ARD_RBDL_MODEL_FOOT_HH

# include <boost/enable_shared_from_this.hpp>

# include <abstract-robot-dynamics/foot.hh>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <ard/rbdl/model/joint.hh>

namespace ard
{
  namespace rbdl
  {
    class Foot :
      public CjrlFoot,
      public boost::enable_shared_from_this<Foot>
    {
    public:
      /// \brief Default construcor.
      explicit Foot ();

      /// \brief Create foot with associated ankle joint.
      ///
      /// \param ankle associated ankle joint.
      explicit Foot (const to_pointer<CjrlJoint>::type& ankle);

      /// \brief Create foot with associated ankle joint.
      ///
      /// \param ankle associated ankle joint.
      explicit Foot (const jointShPtr_t& ankle);

      /// \brief Copy constructor.
      Foot (const Foot& foot);

      /// \brief Destructor
      virtual ~Foot ();

      /// \brief Get the ankle to which the foot is attached
      virtual to_pointer<CjrlJoint>::type associatedAnkle () const;

      /// \brief Set the ankle to which the hand is attached.
      virtual void setAssociatedAnkle (to_pointer<CjrlJoint>::type inJoint);

      /// \brief Get size of the rectagular sole
      ///
      /// \param outLength length of the sole (see Figure)
      /// \param outWidth width of the sole (see Figure)
      virtual void getSoleSize (double& outLength, double& outWidth) const;

      /// \brief Set size of the rectagular sole
      ///
      /// \param inLength length of the sole (see Figure)
      /// \param inWidth width of the sole (see Figure)
      virtual void setSoleSize (const double &inLength, const double &inWidth);

      /// \brief  Get position of the ankle in the foot local coordinate frame
      ///
      /// \retval outCoordinates coordinates of the ankle joint center
      virtual void getAnklePositionInLocalFrame (vector3d& outCoordinates) const;

      /// \brief  Set position of the ankle in the foot local coordinate frame
      ///
      /// \param inCoordinates coordinates of the ankle joint center
      virtual void setAnklePositionInLocalFrame (const vector3d& inCoordinates);

    private:
      // Associated ankle joint attribute.
      jointWkPtr_t ankleJoint_;
      // Sole length attribute.
      double soleLength_;
      // Sole width attribute.
      double soleWidth_;
      // Ankle position in local frame attribute.
      vector3d ankleInLocalFrame_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_FOOT_HH
