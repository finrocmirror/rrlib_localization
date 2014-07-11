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
/*!\file    rrlib/localization/pose/tUncertainPoseBase.h
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-21
 *
 * \brief   Contains tUncertainPoseBase
 *
 * \b tUncertainPoseBase
 *
 * Base class for poses with additional uncertainty (covariance matrix)
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

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
class tUncertainPose;

namespace pose
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Base class for poses with additional uncertainty (covariance matrix)
 */
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
class tUncertainPoseBase : public tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>
{
  template <typename TPoseElement = TElement>
  using tPose = localization::tPose<Tdimension, TPoseElement, TPositionSIUnit, TOrientationSIUnit>;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPositionElement = TElement>
  using tPosition = typename tPose<>::template tPosition<TPositionElement>;

  template <typename TPositionElement = TElement>
  using tPositionComponent = typename tPose<>::template tPositionComponent<TPositionElement>;

  template <typename TOrientationElement = TElement>
  using tOrientation = typename tPose<>::template tOrientation<TOrientationElement>;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = math::angle::NoWrap>
  using tOrientationComponent = typename tPose<>::template tOrientationComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  template <typename TCovarianceElement = TElement>
  using tCovarianceMatrix = math::tMatrix < sizeof(tPose<>) / sizeof(TElement), sizeof(tPose<>) / sizeof(TElement), TCovarianceElement >;

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

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tCovarianceMatrix<> covariance;

};

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
std::ostream &operator << (std::ostream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
std::istream &operator >> (std::istream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/localization/pose/tUncertainPoseBase.hpp"

#endif
