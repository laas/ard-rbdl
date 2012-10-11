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

/// \brief Declarations of utility functions.

#ifndef ARD_RBDL_UTIL_HH
# define ARD_RBDL_UTIL_HH

# include <boost/foreach.hpp>

# include <ard/rbdl/tools/types.hh>

namespace ard
{
  namespace rbdl
  {
    template <class T>
    inline T* getUnsafePointer (boost::weak_ptr<T> wkPtr)
    {
      boost::shared_ptr<T> shPtr = wkPtr.lock ();
      if (shPtr)
	return shPtr.get ();
      else
	return 0;
    }

    template <class T>
    inline T* getUnsafePointer (boost::shared_ptr<T> shPtr)
    {
      if (shPtr)
	return shPtr.get ();
      else
	return 0;
    }

    inline bool isJointInVector (const jointShPtr_t& joint,
				 const jointShPtrs_t& joints)
    {
      BOOST_FOREACH (jointShPtr_t joint_i, joints)
	if (joint_i == joint)
	  return true;
      return false;
    }

  } // end of namespace rbdl.
} // end of namespace ard.
  
#endif //! ARD_RBDL_UTIL_HH
