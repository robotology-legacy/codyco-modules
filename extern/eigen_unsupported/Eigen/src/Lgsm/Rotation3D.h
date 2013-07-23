// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2011 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef LIE_ROTATION3D_H
#define LIE_ROTATION3D_H

/***********************************************************************************
* There are no need for new constructor or accessor. Some typedefs will do the job.
************************************************************************************/

/** single precision 3D Rotation type */
typedef LieGroup<Quaternion<float> > Rotation3f;
/** double precision 3D Rotation type */
typedef LieGroup<Quaternion<double> > Rotation3d;

/** Map an unaligned array of single precision scalar as a 3D Rotation */
typedef Map<LieGroup<Quaternion<float> >, 0>         Rotation3Mapf;
/** Map an unaligned array of double precision scalar as a 3D Rotation */
typedef Map<LieGroup<Quaternion<double> >, 0>        Rotation3Mapd;
/** Map a 16-bits aligned array of double precision scalars as a 3D Rotation */
typedef Map<LieGroup<Quaternion<float> >, Aligned>   Rotation3MapAlignedf;
/** Map a 16-bits aligned array of double precision scalars as a 3D Rotation */
typedef Map<LieGroup<Quaternion<double> >, Aligned>  Rotation3MapAlignedd;

#endif
