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

#ifndef LIE_GROUP_H
#define LIE_GROUP_H

/***************************************************************************
* Definition/implementation of LieGroupBase<G, Derived>
***************************************************************************/

/**
 * Definition of LieGroupBase<G, Derived>
 *
 * \class LieGroupBase
 *
 * \brief Base class for all Lie Group class. 
 *
 * \tparam G the wrapped class
 * \tparam Derived the derived class holding the coefficients which are of type G or Map<G>
 *
 * This class must be specialized to add a new group. This class wrap an Eigen class G and
 * define some function to describe a Lie Group.
 */

namespace internal {
  template<class G, class Derived>
  struct traits<LieGroupBase<G, Derived> > {
    typedef LieGroup<G> PlainObject;
  };
}


template<class G, class Derived> class LieGroupBase {
public:
  /* List of typedef */
  /** The kind of stored coefficients */
  typedef typename Derived::Coefficients Coefficients;
  /** The type of stored coefficients */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** The wrapped class */
  typedef G BaseType;
  /** The plain object returned, while using Map<LieGroup< > >*/
  typedef typename internal::traits<Derived>::PlainObject PlainObject;
  /** The type of the adjoint Matrix */
  typedef typename Derived::AdjointMatrix AdjointMatrix;
  /** The type of the associated Algebra */
  typedef typename Derived::Algebra Algebra;
  /** The type of the dual Algebra */
  typedef typename Derived::CoAlgebra CoAlgebra;

  /** \returns a plain object describing the inverse element*/
  PlainObject inverse() const;
  /** \returns The plain object describing the identity element of the Lie group*/
  static PlainObject Identity();
  /** \returns The plain object describing the composition of two elements*/
  template<class OtherDerived> PlainObject operator*(const LieGroupBase<G, OtherDerived>& other) const;

  /** \returns The Matrix which is the adjoint representation of the element */
  AdjointMatrix adjoint(void) const;
  /** \returns The algebra which is the product of the adjoint representation and a element of the associated algebra */
  Algebra adjoint(const Algebra& ) const;
  /** \returns The coalgebra which is the product of the adjoint representation and a element of the associated coalgebra */
  CoAlgebra adjointTr(const CoAlgebra& ) const;

  /** \returns The element of the associated Algebra through the exponential map 
   *  \sa the exponential map is given by LieAlgebra::exp()
   */
  Algebra log(const Scalar precision = 1e-6) const;

  /** Assignement operator */
  template<class OtherDerived> LieGroupBase& operator=(const LieGroupBase<G, OtherDerived>& );

  /** The read-only accessor to the derived class */
  inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
  /** The accessor to the derived class */
  inline Derived& derived() { return *static_cast<Derived*>(this); }

  /** \returns The stored coefficients */
  inline Coefficients& get();
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const;
};

/***************************************************************************
* Definition of LieGroup<G>
***************************************************************************/

namespace internal {
  template<class G>
    struct traits<LieGroup<G> > : public traits<LieGroupBase<G, LieGroup<G> > >
    {
      typedef G Coefficients;
      typedef typename G::Scalar Scalar;
    };
}

/**
 * Definition of LieGroup<G>
 *
 * \class LieGroup
 *
 * \brief Class describing an element of a Lie Group. 
 *
 * \tparam G the wrapped class
 *
 * This class must be specialized to add new constructors for a specific group. 
 *
 * \sa The methods are defined in  LieGroupBase
 */

template<class G> class LieGroup : public LieGroupBase<G, LieGroup<G> > {
protected:
  /** Inherited class */
  typedef LieGroupBase<G, LieGroup<G> > Base;
public:
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(LieGroup)

  /** The stored coefficients */
  typedef typename internal::traits<LieGroup<G> >::Coefficients Coefficients;

  /** Copy constructor : do nothing */
  inline LieGroup(const LieGroup&) {};
  /** Constructor : do nothing */
  inline LieGroup() {};
    
  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/***************************************************************************
* Definition/implementation of Map<LieGroup< >
***************************************************************************/


namespace internal {
  template<class G, int PacketAccess>
    struct traits<Map<LieGroup<G>, PacketAccess> > : public traits<LieGroupBase<G, Map<LieGroup<G>, PacketAccess> > >
    {
      typedef Map<G, PacketAccess> Coefficients;
      typedef typename G::Scalar Scalar;
    };

  template<class G, int PacketAccess>
    struct traits<Map<const LieGroup<G>, PacketAccess> > : public traits<LieGroupBase<G, Map<const LieGroup<G>, PacketAccess> > >
    {
      typedef Map<const G, PacketAccess> Coefficients;
      typedef typename G::Scalar Scalar;
    };
}

/**
 * Definition of Map<LieGroup>
 *
 * \brief Class describing a map element of a Lie Group. 
 *
 * \param G the wrapped class
 * \param PacketAccess \see Map<Matrix>
 *
 * This class must be specialized to add new constructors for a specific group. 
 *
 * \sa The methods are defined in  LieGroupBase
 */

template<class G, int PacketAccess> class Map<LieGroup<G>, PacketAccess> : public LieGroupBase<G, Map<LieGroup<G>, PacketAccess> > {
  protected:
    /** Inherited class */
    typedef LieGroupBase<G, Map<LieGroup<G>, PacketAccess > > Base;
  public:
    // inherit assignement operator
    EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
      /** Coefficients type */
      typedef typename internal::traits<Map<LieGroup<G>, PacketAccess> >::Scalar Scalar;
    /** The stored coefficients */
    typedef typename internal::traits<Map<LieGroup<G>, PacketAccess> >::Coefficients Coefficients;

    /** Maps a class G */
    inline Map(const G& g) : m_coeffs(g) {};
    /** Maps an Array */
    template<int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> 
      inline Map(Array<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& g) : m_coeffs(g.data()) {};
    /** Maps an array of scalar */
    inline Map(Scalar* data) : m_coeffs(data) {};
    /** Maps another Map<LieGroup> */
    inline Map(const Map& m) : m_coeffs(m.get()) {};

    /** \returns The stored coefficients */
    inline Coefficients& get() { return m_coeffs; }
    /** \returns The read-only access to the stored coefficients */
    inline const Coefficients& get() const { return m_coeffs; }

  protected:
    /** The wrapped coefficients */
    Coefficients m_coeffs;
};

template<class G, int PacketAccess> class Map<const LieGroup<G>, PacketAccess> : public LieGroupBase<G, Map<const LieGroup<G>, PacketAccess> > {
  protected:
    /** Inherited class */
    typedef LieGroupBase<G, Map<const LieGroup<G>, PacketAccess > > Base;
  public:
    // inherit assignement operator
    EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
      /** Coefficients type */
      typedef typename internal::traits<Map<const LieGroup<G>, PacketAccess> >::Scalar Scalar;
    /** The stored coefficients */
    typedef typename internal::traits<Map<const LieGroup<G>, PacketAccess> >::Coefficients Coefficients;

    /** Maps a class G */
    inline Map(const G& g) : m_coeffs(g) {};
    /** Maps an Array */
    template<int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> 
      inline Map(Array<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& g) : m_coeffs(g.data()) {};
    /** Maps an array of scalar */
    inline Map(const Scalar* data) : m_coeffs(data) {};
    /** Maps another Map<LieGroup> */
    inline Map(const Map& m) : m_coeffs(m.get()) {};

    /** \returns The stored coefficients */
    inline Coefficients& get() { return m_coeffs; }
    /** \returns The read-only access to the stored coefficients */
    inline const Coefficients& get() const { return m_coeffs; }

  protected:
    /** The wrapped coefficients */
    Coefficients m_coeffs;
};

#endif


