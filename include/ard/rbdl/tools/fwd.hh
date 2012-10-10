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

/// \brief Declarations of types.

#ifndef ARD_RBDL_FWD_HH
# define ARD_RBDL_FWD_HH

# include <boost/shared_ptr.hpp>
# include <boost/weak_ptr.hpp>

/// \def ARD_RBDL_DEFINE_TYPES(T, NAME)
///
/// Create typedefs from class.
///
/// This macro defines new pointer and vector types from \a T and
/// prefixes them with \a NAME.
///
/// \param T class name
/// \param Name desired name
# define ARD_RBDL_DEFINE_TYPES(T, NAME)				\
  typedef T NAME##_t;						\
  typedef T* NAME##Ptr_t;					\
  typedef boost::shared_ptr<T> NAME##ShPtr_t;			\
  typedef boost::weak_ptr<T> NAME##WkPtr_t;			\
  typedef std::vector<T> NAME##s_t;				\
  typedef std::vector<T*> NAME##Ptrs_t;				\
  typedef std::vector<boost::shared_ptr<T> > NAME##ShPtrs_t;	\
  typedef std::vector<boost::weak_ptr<T> > NAME##WkPtrs_t

namespace ard
{
  namespace rbdl
  {
    class Joint;
    ARD_RBDL_DEFINE_TYPES (Joint, joint);
    class Body;
    ARD_RBDL_DEFINE_TYPES (Body, body);
    class DynamicRobot;
    ARD_RBDL_DEFINE_TYPES (DynamicRobot, dynamicRobot);
    class Hand;
    ARD_RBDL_DEFINE_TYPES (Hand, hand);
    class Foot;
    ARD_RBDL_DEFINE_TYPES (Foot, foot);
    class HumanoidDynamicRobot;
    ARD_RBDL_DEFINE_TYPES (HumanoidDynamicRobot, humanoidDynamicRobot);

  } // end of namespace rbdl.
} // end of namespace ard.
  
#endif //! ARD_RBDL_FWD_HH
