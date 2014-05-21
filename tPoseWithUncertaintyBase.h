//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    rrlib/localization/tPoseWithUncertaintyBase.h
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-21
 *
 * \brief   Contains tPoseWithUncertaintyBase
 *
 * \b tPoseWithUncertaintyBase
 *
 * Base class for classes related to poses with additional uncertainty (covariance matrix)
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tPoseWithUncertaintyBase_h__
#define __rrlib__localization__tPoseWithUncertaintyBase_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tMatrix.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tPoseBase.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Base class for classes related to poses with additional uncertainty (covariance matrix)
 */
template <size_t Tdimension, typename TLinear, typename TAngular>
class tPoseWithUncertaintyBase : public tPoseBase<Tdimension, TLinear, TAngular>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef tPoseBase<Tdimension, TLinear, TAngular> tSuper;

  /** the internal representation of covariance, as all covariances are stored in a matrix this has to be a single type */
  typedef double tInternalCovarianceType;

  using tSuper::dimension;

  typedef TLinear tLinear;
  typedef decltype(TLinear() * TLinear()) tLinearSquared;
  typedef TAngular tAngular;
  typedef decltype(TAngular() * TAngular()) tAngularSquared;
  typedef decltype(TLinear() * TAngular()) tLinearCrossAngular;

  tPoseWithUncertaintyBase &operator= (const tPoseWithUncertaintyBase &o) = default;


  // inherit all constructors, these then represent a pose with zero uncertainty
  using tSuper::tSuper;

  /* first, the variances (var(A) = cov(A, A)) */
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 1>::type >
  tLinearSquared GetVarianceX()
  {
    return covariance[0][0];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 1>::type >
  void SetVarianceX(const tLinearSquared &v)
  {
    covariance[0][0] = static_cast<double>(v);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  tLinearSquared GetVarianceY()
  {
    return covariance[1][1];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  void SetVarianceY(const tLinearSquared &v)
  {
    covariance[1][1] = static_cast<double>(v);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearSquared GetVarianceZ()
  {
    return covariance[2][2];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetVarianceZ(const tLinearSquared &v)
  {
    covariance[2][2] = static_cast<double>(v);
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetVarianceRoll()
  {
    return covariance[3][3];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetVarianceRoll(const tAngularSquared &v)
  {
    covariance[3][3] = static_cast<double>(v);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetVariancePitch()
  {
    return covariance[4][4];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetVariancePitch(const tAngularSquared &v)
  {
    covariance[4][4] = static_cast<double>(v);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetVarianceYaw()
  {
    return covariance[5][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetVarianceYaw(const tAngularSquared &v)
  {
    covariance[5][5] = static_cast<double>(v);
  }


  /* and now the covariances for all pairs
   * (NOTE 1: for reasons of consistency, the variances are given again,
   *  NOTE 2: for reasons of commutativity, only one direction is given)
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 1>::type >
  tLinearSquared GetCovarianceXX()
  {
    return covariance[0][0];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 1>::type >
  void SetCovarianceXX(const tLinearSquared &c)
  {
    covariance[0][0] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  tLinearSquared GetCovarianceXY()
  {
    return covariance[0][1];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  void SetCovarianceXY(const tLinearSquared &c)
  {
    covariance[0][1] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearSquared GetCovarianceXZ()
  {
    return covariance[0][2];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceXZ(const tLinearSquared &c)
  {
    covariance[0][2] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceXRoll()
  {
    return covariance[0][3];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceXRoll(const tLinearCrossAngular &c)
  {
    covariance[0][3] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceXPitch()
  {
    return covariance[0][4];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceXPitch(const tLinearCrossAngular &c)
  {
    covariance[0][4] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceXYaw()
  {
    return covariance[0][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceXYaw(const tLinearCrossAngular &c)
  {
    covariance[0][5] = static_cast<double>(c);
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  tLinearSquared GetCovarianceYY()
  {
    return covariance[1][1];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  void SetCovarianceYY(const tLinearSquared &c)
  {
    covariance[1][1] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearSquared GetCovarianceYZ()
  {
    return covariance[1][2];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceYZ(const tLinearSquared &c)
  {
    covariance[1][2] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceYRoll()
  {
    return covariance[1][3];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceYRoll(const tLinearCrossAngular &c)
  {
    covariance[1][3] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceYPitch()
  {
    return covariance[1][4];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceYPitch(const tLinearCrossAngular &c)
  {
    covariance[1][4] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceYYaw()
  {
    return covariance[1][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceYYaw(const tLinearCrossAngular &c)
  {
    covariance[1][5] = static_cast<double>(c);
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearSquared GetCovarianceZZ()
  {
    return covariance[2][2];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceZZ(const tLinearSquared &c)
  {
    covariance[2][2] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceZRoll()
  {
    return covariance[2][3];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceZRoll(const tLinearCrossAngular &c)
  {
    covariance[2][3] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceZPitch()
  {
    return covariance[2][4];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceZPitch(const tLinearCrossAngular &c)
  {
    covariance[2][4] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tLinearCrossAngular GetCovarianceZYaw()
  {
    return covariance[2][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceZYaw(const tLinearCrossAngular &c)
  {
    covariance[2][5] = static_cast<double>(c);
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetCovarianceRollRoll()
  {
    return covariance[3][3];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceRollRoll(const tAngularSquared &c)
  {
    covariance[3][3] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetCovarianceRollPitch()
  {
    return covariance[3][4];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceRollPitch(const tAngularSquared &c)
  {
    covariance[3][4] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetCovarianceRollYaw()
  {
    return covariance[3][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceRollYaw(const tAngularSquared &c)
  {
    covariance[3][5] = static_cast<double>(c);
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetCovariancePitchPitch()
  {
    return covariance[4][4];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovariancePitchPitch(const tAngularSquared &c)
  {
    covariance[4][4] = static_cast<double>(c);
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetCovariancePitchYaw()
  {
    return covariance[4][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovariancePitchYaw(const tAngularSquared &c)
  {
    covariance[4][5] = static_cast<double>(c);
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  tAngularSquared GetCovarianceYawYaw()
  {
    return covariance[5][5];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  void SetCovarianceYawYaw(const tAngularSquared &c)
  {
    covariance[5][5] = static_cast<double>(c);
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  template <size_t Tdimension_, typename TLinear_, typename TAngular_>
  friend std::ostream& operator << (std::ostream& stream, const tPoseWithUncertaintyBase<Tdimension_, TLinear_, TAngular_>& o);
  template <size_t Tdimension_, typename TLinear_, typename TAngular_>
  friend rrlib::serialization::tStringOutputStream& operator << (rrlib::serialization::tStringOutputStream& stream, const tPoseWithUncertaintyBase<Tdimension_, TLinear_, TAngular_>& o);
  template <size_t Tdimension_, typename TLinear_, typename TAngular_>
  friend rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tPoseWithUncertaintyBase<Tdimension_, TLinear_, TAngular_>& o);
  template <size_t Tdimension_, typename TLinear_, typename TAngular_>
  friend rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tPoseWithUncertaintyBase<Tdimension_, TLinear_, TAngular_>& o);




  // FIXME: actually, with all the template types we would need a special matrix with different element types, depending on the position, but for now we do it with all doubles
  // FIXME: actually, this is a triangular matrix
  rrlib::math::tMatrix<dimension * 2, dimension * 2, tInternalCovarianceType> covariance;


};

template <size_t Tdimension, typename TLinear, typename TAngular>
std::ostream &operator << (std::ostream &stream, const tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>& o)
{
  stream << static_cast <
         const typename tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>::tSuper&
         >(o) << "Covariance matrix: " << o.covariance;
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>& o)
{
  stream << static_cast <
         const typename tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>::tSuper&
         >(o) << o.covariance;
  return stream;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>& o)
{
  stream >> static_cast <
         typename tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>::tSuper&
         >(o) >> o.covariance;
  return stream;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tStringOutputStream& operator << (rrlib::serialization::tStringOutputStream& stream, const tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>& o)
{
  stream << static_cast <
         typename tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>::tSuper
         >(o) << "Covariance matrix: " << o.covariance;
  return stream;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tStringInputStream& operator >> (rrlib::serialization::tStringInputStream& stream, tPoseWithUncertaintyBase<Tdimension, TLinear, TAngular>& o)
{
  throw std::runtime_error("Deserializing from string is not supported");
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/localization/tPoseWithUncertaintyBase.hpp"

#endif
