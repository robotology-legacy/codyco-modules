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

#ifndef LIE_ALGEBRA_H
#define LIE_ALGEBRA_H

/***************************************************************************
* Definition/implementation of LieAlgebraBase<A, Derived>
***************************************************************************/

/**
 * \brief Base class for all Lie Algebra class. 
 *
 * \tparam A the wrapped class
 * \tparam Derived the derived class holding the coefficients which are of type A or Map<A>
 *
 * This class must be specialized to add a new algebra. This class wrap an Eigen class A and
 * define some function to describe a Lie Algebra. 
 *
 * Since a Lie Algebra is a vector Space (check if it's true in the general case) it's inherited from MatrixBase
 */

namespace internal {
  template<class A, class Derived>
    struct traits<LieAlgebraBase<A, Derived> > : public traits<A> {};
}

template<class A, class Derived> class LieAlgebraBase : public MatrixBase<Derived> {
protected:
  /** The inherited class */
  typedef MatrixBase<Derived> Base;
public:  
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraBase)
  // inherit operator=
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraBase)
  // accessor needed for MatrixBase inheritance
  LIE_INHERIT_MATRIX_BASE(A::RowsAtCompileTime, A::ColsAtCompileTime)

  /** The wrapped class */
  typedef A BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieAlgebra< > >*/
  typedef LieAlgebra<BaseType> PlainObject;   
  /** The type of the dual Algebra */
  typedef LieAlgebraDual<BaseType> AlgebraDual;
  /** The type of the associated Lie Group */
  typedef typename Derived::LieGroup Group;


  /** Lie Bracket*/
  template<class OtherDerived> inline PlainObject bracket(const LieAlgebraBase<BaseType, OtherDerived>& a) const;
  /** Adjoint representation of the Lie Algebra*/
  template<class OtherDerived> inline const PlainObject adjoint(const LieAlgebraBase<BaseType, OtherDerived>& a) const;
  
  /** \returns The element of the associated Lie Group through the exponential map
   */
  Group exp(Scalar precision = 1.e-6) const;

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
* Definition/implementation of LieAlgebraDualBase<A, Derived>
***************************************************************************/

/**
 * \brief Base class for all Lie aglebra dual class. 
 *
 * \tparam A the wrapped class
 * \tparam Derived the derived class holding the coefficients which are of type A or Map<A>
 *
 * This class must be specialized to add a new Lie aglebra dual. This class wrap an Eigen class A and
 * define some function to describe a Lie algebra dual. This algebra dual is the dual to the Algebra defined in LieAlgebra
 *
 * A Lie algebra dual is a vector space and is not an algebra it's inherited from MatrixBase
 */

namespace internal {
  template<class A, class Derived>
    struct traits<LieAlgebraDualBase<A, Derived> > : public traits<A> {};
}

template<class A, class Derived> class LieAlgebraDualBase : public MatrixBase<Derived > {
protected:
  /** The inherited class */
  typedef MatrixBase<Derived>  Base;
public:
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraDualBase)
  // inherit operator=
  //EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraDualBase)
  // accessor needed for MatrixBase inheritance
  LIE_INHERIT_MATRIX_BASE(A::RowsAtCompileTime, A::ColsAtCompileTime)

  
  /** Default assignement operator */
  EIGEN_STRONG_INLINE LieAlgebraDualBase& operator=(const LieAlgebraDualBase& other) {
    this->get() = other.get();
    return *this;
  }
  /** Assignement operator between derived type*/
  template<class BaseType, class OtherDerived> EIGEN_STRONG_INLINE Derived& operator=(const LieAlgebraDualBase<BaseType, OtherDerived>& other) {
    this->get() = other.get();
    return derived();
  }

  /** The wrapped class */
  typedef A BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieAlgebraDual< > >*/
  typedef LieAlgebraDual<BaseType> PlainObject;   
  /** The type of the dual Algebra */
  typedef LieAlgebra<BaseType> Algebra;
  /** The type of the associated Lie Group */
  typedef typename internal::traits<Derived>::Group Group;

  /** The read-only accessor to the derived class */
  inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
  /** The accessor to the derived class */
  inline Derived& derived() { return *static_cast<Derived*>(this); }

  /** \returns The stored coefficients */
  inline Coefficients& get() {return this->derived().get(); }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const {return this->derived().get(); }
};


/***************************************************************************
* Definition of LieAlgebra<A>
***************************************************************************/

namespace internal {
  template<class A>
    struct traits<LieAlgebra<A> > : public traits<LieAlgebraBase<A, LieAlgebra<A> > >
    {
      typedef A Coefficients;
    };
}

/**
 * \brief Class describing an element of a Lie Algebra. 
 *
 * \tparam A the wrapped class
 *
 * This class must be specialized to add new constructors for a specific algebra. 
 *
 * \sa The methods are defined in  LieAlgebraBase
 */

template<class A> class LieAlgebra : public LieAlgebraBase<A, LieAlgebra<A> > {
protected:
  /** Inherited class */
  typedef LieAlgebraBase<A, LieAlgebra<A> > Base;
public:
  // inherit MatrixBase operator
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebra)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebra)

  /** The stored coefficients */
  typedef typename internal::traits<LieAlgebra<A> >::Coefficients Coefficients;

  /** Copy constructor : do nothing */
  inline LieAlgebra(const LieAlgebra&);

  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/***************************************************************************
* Definition of LieAlgebraDual<A>
***************************************************************************/

namespace internal {
  template<class A>
    struct traits<LieAlgebraDual<A> > : public traits<LieAlgebraDualBase<A, LieAlgebraDual<A> > >
    {
      typedef A Coefficients;
    };
}

/**
 * \brief Class describing an element of a Lie algebra dual. 
 *
 * \tparam A the wrapped class
 *
 * This class must be specialized to add new constructors for a specific algebra dual. 
 *
 * \sa The methods are defined in LieAlgebraDualBase
 */

template<class A> class LieAlgebraDual : public LieAlgebraDualBase<A, LieAlgebraDual<A> > {
protected:
  typedef LieAlgebraDualBase<A, LieAlgebraDual<A> > Base;
  /** Inherited class */
public:
  // inherit MatrixBase operator
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraDual)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraDual)

  /** The stored coefficients */
  typedef typename internal::traits<LieAlgebraDual<A> >::Coefficients Coefficients;

  /** Copy constructor : do nothing */
  inline LieAlgebraDual(const LieAlgebraDual&);

  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }
protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};


/***************************************************************************
* Definition of Map<LieAlgebra<A> >
***************************************************************************/

namespace internal {
  template<class A, int Options>
    struct traits<Map<LieAlgebra<A>, Options> > : public traits<LieAlgebraBase<A, Map<LieAlgebra<A>, Options> > >
    {
      typedef Map<A, Options> Coefficients;
    };
}

namespace internal {
  template<class A, int Options>
    struct traits<Map<const LieAlgebra<A>, Options> > : public traits<LieAlgebraBase<A, Map<const LieAlgebra<A>, Options> > >
    {
      typedef Map<const A, Options> Coefficients;
    };
}

/**
 * Definition of Map<LieAlgebra>
 *
 * \brief Class describing a map element of a Lie Algebra. 
 *
 * \param G the wrapped class
 * \param PacketAccess \see Map<Matrix>
 *
 * This class must be specialized to add new constructors for a specific group. 
 *
 * \sa The methods are defined in  LieAlgebraBase
 */

template<class A, int Options>
class Map<LieAlgebra<A>, Options> : public LieAlgebraBase<A, Map<LieAlgebra<A>, Options> > {
  protected:
    /** Inherited class */
    typedef LieAlgebraBase<A, Map<LieAlgebra<A>, Options> > Base;
  public:
    // inherit MatrixBase operator
    EIGEN_DENSE_PUBLIC_INTERFACE(Map)
      // inherit assignement operator
      EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Map)

      /** The stored coefficients */
      typedef typename internal::traits<Map<LieAlgebra<A>, Options> >::Coefficients Coefficients;

    /** Maps a class A */
    Map(const A& a) : m_coeffs(a.data()) {};
    /** Maps an array of scalar */
    Map(Scalar* data) : m_coeffs(data) {};
    /** Maps another Map<LieAlgebra> */
    Map(const Map& m) : m_coeffs(m.get()) {};

    /** \returns The stored coefficients */
    Coefficients& get() { return m_coeffs; }
    /** \returns The read-only access to the stored coefficients */
    const Coefficients& get() const { return m_coeffs; }

  protected:
    /** The wrapped coefficients */
    Coefficients m_coeffs;
};

template<class A, int Options>
class Map<const LieAlgebra<A>, Options> : public LieAlgebraBase<A, Map<const LieAlgebra<A>, Options> > {
  protected:
    /** Inherited class */
    typedef LieAlgebraBase<A, Map<const LieAlgebra<A>, Options> > Base;
  public:
    // inherit MatrixBase operator
    EIGEN_DENSE_PUBLIC_INTERFACE(Map)

      /** The stored coefficients */
      typedef typename internal::traits<Map<const LieAlgebra<A>, Options> >::Coefficients Coefficients;

    /** Maps a class A */
    Map(const A& a) : m_coeffs(a.data()) {};
    /** Maps an array of scalar */
    Map(const Scalar* data) : m_coeffs(data) {};
    /** Maps another Map<LieAlgebra> */
    Map(const Map& m) : m_coeffs(m.get()) {};

    /** \returns The stored coefficients */
    Coefficients& get() { return m_coeffs; }
    /** \returns The read-only access to the stored coefficients */
    const Coefficients& get() const { return m_coeffs; }

  protected:
    /** The wrapped coefficients */
    Coefficients m_coeffs;
};


/***************************************************************************
 * Definition of Map<LieAlgebraDual<A> >
 ***************************************************************************/

namespace internal {
  template<class A, int Options>
    struct traits<Map<LieAlgebraDual<A>, Options> > : public traits<LieAlgebraDualBase<A, Map<LieAlgebraDual<A>, Options> > >
    {
      typedef Map<A, Options> Coefficients;
    };

  template<class A, int Options>
    struct traits<Map<const LieAlgebraDual<A>, Options> > : public traits<LieAlgebraDualBase<A, Map<const LieAlgebraDual<A>, Options> > >
    {
      typedef Map<A, Options> Coefficients;
    };

}
/**
 * Definition of Map<LieAlgebraDual>
 *
 * \brief Class describing a map element of a Lie algebra dual.. 
 *
 * \param G the wrapped class
 * \param PacketAccess \see Map<Matrix>
 *
 * This class must be specialized to add new constructors for a specific group. 
 *
 * \sa The methods are defined in  LieAlgebraDualBase
 */

template<class A, int Options>
class Map<LieAlgebraDual<A>, Options> : public LieAlgebraDualBase<A, Map<LieAlgebraDual<A>, Options> > {
  protected:
    /** Inherited class */
    typedef LieAlgebraDualBase<A, Map<LieAlgebraDual<A>, Options> > Base;
  public:
    // inherit MatrixBase operator
    EIGEN_DENSE_PUBLIC_INTERFACE(Map)
      // inherit assignement operator
      EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Map)

      /** The stored coefficients */
      typedef typename internal::traits<Map<LieAlgebraDual<A>, Options> >::Coefficients Coefficients;

    /** Maps a class A */
    Map(const A& a) : m_coeffs(a) {};
    /** Maps an array of scalar */
    Map(Scalar* data) : m_coeffs(data) {};
    /** Maps another Map<LieAlgebra> */
    Map(const Map& m) : m_coeffs(m.get()) {};

    /** \returns The stored coefficients */
    Coefficients& get() { return m_coeffs; }
    /** \returns The read-only access to the stored coefficients */
    const Coefficients& get() const { return m_coeffs; }

  protected:
    /** The wrapped coefficients */
    Coefficients m_coeffs;
};

template<class A, int Options>
class Map<const LieAlgebraDual<A>, Options> : public LieAlgebraDualBase<A, Map<const LieAlgebraDual<A>, Options> > {
  protected:
    /** Inherited class */
    typedef LieAlgebraDualBase<A, Map<const LieAlgebraDual<A>, Options> > Base;
  public:
    // inherit MatrixBase operator
    EIGEN_DENSE_PUBLIC_INTERFACE(Map)

      /** The stored coefficients */
      typedef typename internal::traits<Map<const LieAlgebraDual<A>, Options> >::Coefficients Coefficients;

    /** Maps a class A */
    Map(const A& a) : m_coeffs(a) {};
    /** Maps an array of scalar */
    Map(const Scalar* data) : m_coeffs(data) {};
    /** Maps another Map<LieAlgebra> */
    Map(const Map& m) : m_coeffs(m.get()) {};

    /** \returns The stored coefficients */
    Coefficients& get() { return m_coeffs; }
    /** \returns The read-only access to the stored coefficients */
    const Coefficients& get() const { return m_coeffs; }

  protected:
    /** The wrapped coefficients */
    Coefficients m_coeffs;
};

#endif

