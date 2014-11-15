//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    rrlib/localization/pose/tUncertainPoseBase.h
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-21
 *
 * \brief   Contains tUncertainPoseBase
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__uncertain_pose__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tUncertainPose.h" instead.
#endif

#ifndef __rrlib__localization__tUncertainPoseBase_h__
#define __rrlib__localization__tUncertainPoseBase_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tMatrix.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
class tUncertainPose;

namespace pose
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Base class for poses with additional uncertainty (attached covariance matrix)
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
class tUncertainPoseBase : public tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>
{
  template <typename TPoseElement = TElement>
  using tPose = localization::tPose<Tdimension, TPoseElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPositionElement = TElement>
  using tPosition = typename tPose<>::template tPosition<TPositionElement>;

  template <typename TPositionElement = TElement>
  using tPositionComponent = typename tPose<>::template tPositionComponent<TPositionElement>;

  template <typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientation = typename tPose<>::template tOrientation<TOrientationElement, TAutoWrapPolicy>;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientationComponent = typename tPose<>::template tOrientationComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  template <typename TCovarianceElement = TElement>
  using tCovarianceMatrix = math::tMatrix < tPosition<>::cSIZE + tOrientation<>::cSIZE, tPosition<>::cSIZE + tOrientation<>::cSIZE, TCovarianceElement >;

  using tPose<>::tPose;

  tUncertainPoseBase();

  template <typename TPose, typename TCovarianceElement>
  tUncertainPoseBase(const TPose &pose, const tCovarianceMatrix<TCovarianceElement> &covariance);

  template <typename TPosition, typename TOrientation, typename TCovarianceElement>
  tUncertainPoseBase(const TPosition &position, const TOrientation &orientation, const tCovarianceMatrix<TCovarianceElement> &covariance);

  inline const tCovarianceMatrix<> &Covariance() const
  {
    return this->covariance;
  }
  inline tCovarianceMatrix<> &Covariance()
  {
    return this->covariance;
  }

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &operator += (const tUncertainPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &operator -= (const tUncertainPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tCovarianceMatrix<> covariance;

};

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tUncertainPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator + (const tUncertainPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tUncertainPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tUncertainPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator - (const tUncertainPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tUncertainPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/localization/pose/tUncertainPoseBase.hpp"

#endif
