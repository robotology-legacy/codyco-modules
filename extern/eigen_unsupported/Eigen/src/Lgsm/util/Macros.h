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

#ifndef LIE_MACROS_H
#define LIE_MACROS_H

#define LIE_INHERIT_MATRIX_BASE(r, c)\
  inline Index rows() const { return r;} \
  inline Index cols() const { return c;} \
  \
  inline Scalar& coeffRef(Index row, Index col) { return this->get().coeffRef(row, col); } \
  inline const Scalar& coeff(Index row, Index col) const { return this->get().coeff(row, col); } \
  inline Scalar& coeffRef(Index index) { return this->get().coeffRef(index); } \
  inline const Scalar& coeff(Index index) const { return this->get().coeff(index); } \
  \
  template<int LoadMode> inline PacketScalar packet(Index index) const { return derived().get().template packet<LoadMode> (index);} \
  template<int LoadMode> inline PacketScalar packet(Index row, Index col) const { return derived().get().template packet<LoadMode> (row, col);} \
  template<int LoadMode> inline void writePacket(Index row, Index col, const PacketScalar& x) { derived().get().template writePacket<LoadMode>(row, col, x);} \
  template<int LoadMode> inline void writePacket(Index index, const PacketScalar& x) { derived().get().template writePacket<LoadMode>(index, x);} \
  \
  inline Index innerStride() const { return 1; } \
  inline Index outerStride() const { return this->innerSize(); } \
  \
  EIGEN_STRONG_INLINE const Scalar* data() const { return get().data(); } \
  EIGEN_STRONG_INLINE Scalar* data() { return get().data(); } \

#endif
