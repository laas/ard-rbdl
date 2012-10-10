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

/// \brief Declaration of HumanoidDynamicRobot class.

#ifndef ARD_RBDL_MODEL_HUMANOID_DYNAMIC_ROBOT_HH
# define ARD_RBDL_MODEL_HUMANOID_DYNAMIC_ROBOT_HH

# include <boost/enable_shared_from_this.hpp>

# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

# include <ard/rbdl/tools/fwd.hh>
# include <ard/rbdl/tools/types.hh>
# include <ard/rbdl/model/dynamic-robot.hh>
# include <ard/rbdl/model/hand.hh>
# include <ard/rbdl/model/foot.hh>

namespace ard
{
  namespace rbdl
  {
    class HumanoidDynamicRobot :
      public CjrlHumanoidDynamicRobot,
      public DynamicRobot,
      public boost::enable_shared_from_this<HumanoidDynamicRobot>
    {
    public:
      /// \brief Default construcor.
      HumanoidDynamicRobot ();

      /// \brief Destructor.
      virtual ~HumanoidDynamicRobot();

      /// \name Joints specific to humanoid robots
      /// \{

      /// \brief Set the pointer to the waist.
      virtual void waist (CjrlJoint* inWaist);

      /// \brief Get a pointer to the waist.
      virtual CjrlJoint* waist () const;

      /// \brief Set the pointer to the chest.
      ///
      /// \note for some humanoid robots, the waist and the chest are
      /// the same joints.
      virtual void chest (CjrlJoint* inChest);

      /// \brief Get a pointer to the chest.
      ///
      ///\note for some humanoid robots, the waist and the chest are
      ///the same joints.
      virtual CjrlJoint* chest () const;

      /// \brief Set the pointer to the left wrist joint.
      virtual void leftWrist (CjrlJoint* inLefWrist);

      /// \brief Get a pointer to the left wrist.
      virtual CjrlJoint* leftWrist () const;

      /// \brief Set the pointer to the right wrist joint.
      virtual void rightWrist (CjrlJoint* inRightWrist);

      /// \brief Get a pointer to the right wrist.
      virtual CjrlJoint* rightWrist () const;

      /// \brief Set the pointer to the right hand.
      virtual void rightHand (CjrlHand* inRightHand);

      /// \brief Get a pointer to the right hand.
      virtual CjrlHand* rightHand () const;

      /// \brief Set the pointer to the left hand.
      virtual void leftHand (CjrlHand* inLeftHand);

      /// \brief Get a pointer to the left hand.
      virtual CjrlHand* leftHand () const;

      /// \brief Get the hand clench value.
      /// This is a scalar value ranging between 0 and 1 which
      /// describes the hand clench (0 for open and 1 for closed hand)
      virtual double getHandClench (CjrlHand* inHand);

      /// \brief Set the hand clench value. This is a scalar value
      /// ranging between 0 and 1 which describes the hand clench
      /// (0 for open and 1 for closed hand)
      /// \return false if parameter 2 is out of range
      virtual bool setHandClench (CjrlHand* inHand, double inClenchingValue);

      /// \brief Set the pointer to the left ankle joint.
      virtual void leftAnkle (CjrlJoint* inLefAnkle);

      /// \brief Get a pointer to the left ankle.
      virtual CjrlJoint* leftAnkle () const;

      /// \brief Set the pointer to the right ankle joint.
      virtual void rightAnkle (CjrlJoint* inRightAnkle);

      /// \brief Get a pointer to the right ankle.
      virtual CjrlJoint* rightAnkle () const;

      /// \brief Set the pointer to the left foot joint.
      virtual void leftFoot (CjrlFoot* inLeftFoot);

      /// \brief Get a pointer to the left foot.
      virtual CjrlFoot* leftFoot () const;

      /// \brief Set the pointer to the right foot joint.
      virtual void rightFoot (CjrlFoot* inRightFoot);

      /// \brief Get a pointer to the right foot.
      virtual CjrlFoot* rightFoot () const;

      /// \brief Set gaze joint.
      ///
      /// \note For most humanoid robots, the gaze joint is the head.
      virtual void gazeJoint (CjrlJoint* inGazeJoint);

      /// \brief Get gaze joint.
      virtual CjrlJoint* gazeJoint () const;

      /// \brief Set the gaze orientation and position in the local frame
      /// of the gaze joint.
      /// \return inOrigin a point on the gaze straight line,
      /// \return inDirection the direction of the gaze joint.
      virtual void gaze (const vector3d& inDirection, const vector3d& inOrigin);

      /// \brief Get a point on the gaze straight line.
      virtual const vector3d& gazeOrigin () const;

      /// \brief Get the direction of gaze
      virtual const vector3d& gazeDirection () const;

      /// \}

      /// \name Zero momentum point
      /// \{

      /// \brief return the coordinates of the Zero Momentum Point.
      virtual const vector3d& zeroMomentumPoint () const;

      /// \}

    private:
      /// \brief Waist joint attribute.
      jointWkPtr_t waist_;
      /// \brief Chest joint attribute.
      jointWkPtr_t chest_;
      /// \brief Left wrist joint attribute.
      jointWkPtr_t leftWrist_;
      /// \brief Right wrist joint attribute.
      jointWkPtr_t rightWrist_;
      /// \brief Left hand joint attribute.
      handShPtr_t leftHand_;
      /// \brief Right hand joint attribute.
      handShPtr_t rightHand_;
      /// \brief Left hand clench attribute.
      double leftHandClench_;
      /// \brief Right hand clench attribute.
      double rightHandClench_;
      /// \brief Left ankle joint attribute.
      jointWkPtr_t leftAnkle_;
      /// \brief Right ankle joint attribute.
      jointWkPtr_t rightAnkle_;
      /// \brief Left foot joint attribute.
      footShPtr_t leftFoot_;
      /// \brief Right foot joint attribute.
      footShPtr_t rightFoot_;
      /// \brief Gaze joint attribute.
      jointWkPtr_t gazeJoint_;
      /// \brief Gaze origin attribute.
      vector3d gazeOrigin_;
      /// \brief Gaze direction attribute.
      vector3d gazeDirection_;
      /// \brief Zero moment point attribute.
      vector3d zeroMomentPoint_;
    };

  } // end of namespace rbdl.
} // end of namespace ard.

#endif //! ARD_RBDL_MODEL_HUMANOID_DYNAMIC_ROBOT_HH
