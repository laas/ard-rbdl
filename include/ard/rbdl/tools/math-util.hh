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

#ifndef ARD_RBDL_MATH_UTIL_HH
# define ARD_RBDL_MATH_UTIL_HH

# include <boost/foreach.hpp>

# include <ard/rbdl/tools/types.hh>

namespace ard
{
  namespace rbdl
  {
    inline void toRbdlFromMal (rbdlVector3d_t& dst, const vector3d& src)
    {
      dst[0] = src[0];
      dst[1] = src[1];
      dst[2] = src[2];
    }

    inline rbdlVector3d_t toRbdlFromMal (const vector3d& src)
    {
      rbdlVector3d_t dst;
      toRbdlFromMal (dst, src);
      return dst;
    }

    inline void toRbdlFromMal (rbdlMatrix3d_t& dst, const matrix3d& src)
    {
      dst(0,0) = src(0,0);
      dst(0,1) = src(0,1);
      dst(0,2) = src(0,2);
      dst(1,0) = src(1,0);
      dst(1,1) = src(1,1);
      dst(1,2) = src(1,2);
      dst(2,0) = src(2,0);
      dst(2,1) = src(2,1);
      dst(2,2) = src(2,2);
    }

    inline rbdlMatrix3d_t toRbdlFromMal (const matrix3d& src)
    {
      rbdlMatrix3d_t dst;
      toRbdlFromMal (dst, src);
      return dst;
    }

    inline void toRbdlFromMal (rbdlMatrix4d_t& dst, const matrix4d& src)
    {
      dst(0,0) = src(0,0);
      dst(0,1) = src(0,1);
      dst(0,2) = src(0,2);
      dst(0,3) = src(0,3);
      dst(1,0) = src(1,0);
      dst(1,1) = src(1,1);
      dst(1,2) = src(1,2);
      dst(1,3) = src(1,3);
      dst(2,0) = src(2,0);
      dst(2,1) = src(2,1);
      dst(2,2) = src(2,2);
      dst(2,3) = src(2,3);
      dst(3,0) = src(3,0);
      dst(3,1) = src(3,1);
      dst(3,2) = src(3,2);
      dst(3,3) = src(3,3);
    }    

    inline rbdlMatrix4d_t toRbdlFromMal (const matrix4d& src)
    {
      rbdlMatrix4d_t dst;
      toRbdlFromMal (dst, src);
      return dst;
    }

    inline void toRbdlFromMal (rbdlVectorN_t& dst, const vectorN& src)
    {
      dst.resize (src.size ());
      for (unsigned i = 0; i < src.size (); ++i)
	dst[i] = src[i];
    }

    inline rbdlVectorN_t toRbdlFromMal (const vectorN& src)
    {
      rbdlVectorN_t dst;
      toRbdlFromMal (dst, src);
      return dst;
    }

    inline void toRbdlFromMal (rbdlMatrixNxP_t& dst, const matrixNxP& src)
    {
      dst.resize (src.size1 (), src.size2 ());
      for (unsigned i = 0; i < src.size1 (); ++i)
	for (unsigned j = 0; j < src.size2 (); ++j)
	  dst(i,j) = src(i,j);
    }

    inline rbdlMatrixNxP_t toRbdlFromMal (const matrixNxP& src)
    {
      rbdlMatrixNxP_t dst;
      toRbdlFromMal (dst, src);
      return dst;
    }

    inline void toMalFromRbdl (vector3d& dst, const rbdlVector3d_t& src)
    {
      dst[0] = src[0];
      dst[1] = src[1];
      dst[2] = src[2];
    }

    inline vector3d toMalFromRbdl (const rbdlVector3d_t& src)
    {
      vector3d dst;
      toMalFromRbdl (dst, src);
      return dst;
    }

    inline void toMalFromRbdl (matrix3d& dst, const rbdlMatrix3d_t& src)
    {
      dst(0,0) = src(0,0);
      dst(0,1) = src(0,1);
      dst(0,2) = src(0,2);
      dst(1,0) = src(1,0);
      dst(1,1) = src(1,1);
      dst(1,2) = src(1,2);
      dst(2,0) = src(2,0);
      dst(2,1) = src(2,1);
      dst(2,2) = src(2,2);
    }

    inline matrix3d toMalFromRbdl (const rbdlMatrix3d_t& src)
    {
      matrix3d dst;
      toMalFromRbdl (dst, src);
      return dst;
    }

    inline void toMalFromRbdl (matrix4d& dst, const rbdlMatrix4d_t& src)
    {
      dst(0,0) = src(0,0);
      dst(0,1) = src(0,1);
      dst(0,2) = src(0,2);
      dst(0,3) = src(0,3);
      dst(1,0) = src(1,0);
      dst(1,1) = src(1,1);
      dst(1,2) = src(1,2);
      dst(1,3) = src(1,3);
      dst(2,0) = src(2,0);
      dst(2,1) = src(2,1);
      dst(2,2) = src(2,2);
      dst(2,3) = src(2,3);
      dst(3,0) = src(3,0);
      dst(3,1) = src(3,1);
      dst(3,2) = src(3,2);
      dst(3,3) = src(3,3);
    }    

    inline matrix4d toMalFromRbdl (const rbdlMatrix4d_t& src)
    {
      matrix4d dst;
      toMalFromRbdl (dst, src);
      return dst;
    }

    inline void toMalFromRbdl (vectorN& dst, const rbdlVectorN_t& src)
    {
      dst.resize (src.size ());
      for (unsigned i = 0; i < src.size (); ++i)
	dst[i] = src[i];
    }

    inline vectorN toMalFromRbdl (const rbdlVectorN_t& src)
    {
      vectorN dst;
      toMalFromRbdl (dst, src);
      return dst;
    }

    inline void toMalFromRbdl (matrixNxP& dst, const rbdlMatrixNxP_t& src)
    {
      dst.resize (src.cols (), src.rows ());
      for (unsigned i = 0; i < src.cols (); ++i)
	for (unsigned j = 0; j < src.rows (); ++j)
	  dst(i,j) = src(i,j);
    }

    inline matrixNxP toMalFromRbdl (const rbdlMatrixNxP_t& src)
    {
      matrixNxP dst;
      toMalFromRbdl (dst, src);
      return dst;
    }

  } // end of namespace rbdl.
} // end of namespace ard.
  
#endif //! ARD_RBDL_MATH_UTIL_HH
