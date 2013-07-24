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

#ifndef LIE_TORQUE_H
#define LIE_TORQUE_H

/***********************************************************************************
* There are no need for new constructor or accessor. Some typedefs will do the job.
************************************************************************************/

/** single precision angular velocity type */
typedef LieAlgebraDual<Matrix<float, 3, 1> > Torquef;
/** double precision angular velocity type */
typedef LieAlgebraDual<Matrix<double, 3, 1> > Torqued;

/** Map an unaligned array of single precision scalar as an angular velocity */
typedef Map<LieAlgebraDual<Matrix<float, 3, 1> >, 0>         TorqueMapf;
/** Map an unaligned array of double precision scalar as an angular velocity */
typedef Map<LieAlgebraDual<Matrix<double, 3, 1> >, 0>        TorqueMapd;
/** Map a 16-bits aligned array of double precision scalars an angular velocity */
typedef Map<LieAlgebraDual<Matrix<float, 3, 1> >, Aligned>  TorqueMapAlignedf;
/** Map a 16-bits aligned array of double precision scalars an angular velocity */
typedef Map<LieAlgebraDual<Matrix<double, 3, 1> >, Aligned>  TorqueMapAlignedd;

#endif
