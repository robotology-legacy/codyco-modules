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

#ifndef LIE_WRENCH_H
#define LIE_WRENCH_H

/*******************************************************************************
* Definition/implementation of WrenchBase
********************************************************************************/

/**
 * \class WrenchBase
 *
 * \brief Base class describing a Wrench. 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Matrix<Scalar, 6, 1> or Map<Matrix<Scalar, 6, 1> >
 *
 * This class abstracts the underlying mathematical definition and add some accessors. A wrench has two part, a torque and a force.
 * It uses an Matrix internally to store its coefficients.
 */

namespace internal {
  template<class Derived>
    struct traits<WrenchBase<Derived> > : traits<LieAlgebraDualBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived> > {
    };
}

template<class Derived> 
class WrenchBase : public LieAlgebraDualBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>{
protected:
  /** The inherited class */
  typedef LieAlgebraDualBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived> Base;
public:
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(WrenchBase)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(WrenchBase)

  /** The wrapped class */
  typedef typename Base::BaseType BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;

  /** The type of the force */ 
  typedef Matrix<Scalar, 3, 1> Force;
  /** The type of the torque */ 
  typedef LieAlgebraDual<Matrix<Scalar, 3, 1> > Torque;

    /** \returns the \c tx coefficient */
  inline Scalar tx() const { return this->getTorque().x(); }
  /** \returns the \c ty coefficient */
  inline Scalar ty() const { return this->getTorque().y(); }
  /** \returns the \c tz coefficient */
  inline Scalar tz() const { return this->getTorque().z(); }
  /** \returns the \c fx coefficient */
  inline Scalar fx() const { return this->getForce().x(); }
  /** \returns the \c fy coefficient */
  inline Scalar fy() const { return this->getForce().y(); }
  /** \returns the \c tz coefficient */
  inline Scalar fz() const { return this->getForce().z(); }

  /** \returns a reference to the \c tx coefficient */
  inline Scalar& tx() { return this->getTorque().x(); }
  /** \returns a reference to the \c ty coefficient */
  inline Scalar& ty() { return this->getTorque().y(); }
  /** \returns a reference to the \c tz coefficient */
  inline Scalar& tz() { return this->getTorque().z(); }
  /** \returns a reference to the \c fx coefficient */
  inline Scalar& fx() { return this->getForce().x(); }
  /** \returns a reference to the \c fy coefficient */
  inline Scalar& fy() { return this->getForce().y(); }
  /** \returns a reference to the \c tz coefficient */
  inline Scalar& fz() { return this->getForce().z(); }

  
  /** The accessor to the torque */
  inline Map<Torque> getTorque(){ return Map<Torque>(this->data()); }
  /** The read-only accessor to the torque */
  inline Map<const Torque> getTorque() const {return  Map<const Torque>(this->data()); }
  /** The accessor to the force */
  inline Map<Force> getForce() { return Map<Force>(this->derived().get().template tail<3>().data()); }
  /** The read-only accessor to the force */
  inline Map<const Force> getForce() const { return Map<const Force>(this->derived().get().template tail<3>().data()); }

  template<class RotationDerived> inline Wrench<Scalar> changeFrame(const LieGroupBase<Quaternion<Scalar>, RotationDerived>&) const;

  template<class OtherDerived> inline Wrench<Scalar> changePoint(const MatrixBase<OtherDerived>& point) const;
};


template<class Derived>
template<class RotationDerived> 
inline Wrench<typename internal::traits<WrenchBase<Derived> >::Scalar> WrenchBase<Derived>::changeFrame(const LieGroupBase<Quaternion<Scalar>, RotationDerived>& rot) const {
  return Wrench<Scalar>(rot*this->getTorque(),
                        rot*this->getForce());
}


template<class Derived>
template<class OtherDerived> 
inline Wrench<typename internal::traits<WrenchBase<Derived> >::Scalar> WrenchBase<Derived>::changePoint(const MatrixBase<OtherDerived>& point) const {
  return Wrench<Scalar>(this->getTorque() + this->getForce.cross(point),
                        this->getForce() );
}

/**************************
 * Implementation of Wrench
 **************************/

namespace internal {
  template<typename _Scalar>
    struct traits<Wrench<_Scalar> > : traits<LieAlgebraDual<Matrix<_Scalar, 6, 1> > >
    {
      typedef _Scalar Scalar;
    };
}

/**
 * \brief Class describing a Wrench. 
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class add some specific constructors
 *
 * \sa The methods are defined in LieAlgebraBase and WrenchBase
 */

template<typename _Scalar>
class Wrench : public WrenchBase<Wrench<_Scalar> >{
protected:
  typedef WrenchBase<Wrench<_Scalar> > Base;
public:
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(Wrench)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Wrench)

  /** The inherited class */
  typedef typename Base::BaseType BaseType;
  /** the stored coefficients */
  typedef typename internal::traits<Wrench>::Coefficients Coefficients;


  /** Default constructor */
  inline Wrench() : m_coeffs() {}
  /** Copy constructor */
  inline Wrench(const Wrench& g) : m_coeffs(g.get() ) {}
  /** Copy constructor */
  inline Wrench(const BaseType& g) : m_coeffs(g) {}
  /** Copy constructor */
  inline Wrench(const typename Base::PlainObject& g) : m_coeffs(g.get() ) {}
  /** Constructs an element of se(3) from 6 scalar */
  inline Wrench(Scalar tx, Scalar ty, Scalar tz, Scalar fx, Scalar fy, Scalar fz) {
    m_coeffs << tx, ty, tz, fx, fy, fz;
  }
  /** Copy constructor : need to build a twist from a CWiseBinaryOp or CWiseUnaryOp*/
  template<typename OtherDerived>
  EIGEN_STRONG_INLINE Wrench(const MatrixBase<OtherDerived>& other)
    : m_coeffs(other)
  {
  }
  /** Assignement operator : need to copy a CWiseBinaryOp or CWiseUnaryOp to a Twist*/
  template<typename OtherDerived>
  EIGEN_STRONG_INLINE Wrench& operator=(const MatrixBase<OtherDerived>& other) 
  {
    m_coeffs = other;
    return *this;
  }
  /** Constructs Twist from torque \c t and a force \c f */
  inline Wrench(const typename Base::Torque& t, const typename Base::Force& f) {
    this->getForce() = f;
    this->getTorque() = t;
  }
  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/** single precision wrench type */
typedef Wrench<double> Wrenchd;
/** double precision wrench type */
typedef Wrench<float> Wrenchf;

/**************************************
 * Implementation of Map<Wrench>
 **************************************/

namespace internal {
  template<typename _Scalar, int PacketAccess>
    struct traits<Map<Wrench<_Scalar>, PacketAccess> > : traits<Map<LieAlgebraDual<Matrix<_Scalar, 6, 1> >, PacketAccess> >
    {
      typedef _Scalar Scalar;
    };
}
/**
 * \brief Class map an array to wrench. 
 *
 * \tparam _Scalar the type of the underlying array
 * \tparam PacketAccess alignement option
 *
 * \sa The methods are defined in LieAlgebraBase and WrenchBase
 */

template<typename _Scalar, int PacketAccess>
class Map<Wrench<_Scalar>, PacketAccess> : public WrenchBase<Map<Wrench<_Scalar>, PacketAccess> >{
  protected:
    typedef WrenchBase<Map<Wrench<_Scalar> > > Base;
  public:
    EIGEN_DENSE_PUBLIC_INTERFACE(Map)
      EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)

      typedef typename internal::traits<Map>::Coefficients Coefficients;

    inline Map(const Wrench<Scalar>& w) : m_coeffs(w.get()) {};
    template<int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> 
      inline Map(const Array<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& g) : m_coeffs(g.data()) {};

    inline Map(Scalar* data) : m_coeffs(data) {};
    inline Map(const Map& m) : m_coeffs(m.get()) {};

    inline Coefficients& get() { return m_coeffs; }
    inline const Coefficients& get() const { return m_coeffs; }

  protected:
    Coefficients m_coeffs;
};

#endif

