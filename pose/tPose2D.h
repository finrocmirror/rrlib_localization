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
/*!\file    rrlib/localization/pose/tPose2D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPose2D
 *
 * \b tPose2D
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__pose__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tPose.h" instead.
#endif

#ifndef __rrlib__localization__pose__tPose2D_h__
#define __rrlib__localization__pose__tPose2D_h__

#include "rrlib/localization/pose/tPoseBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include <iostream>

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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
//!
/*!
 *
 */
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
class tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> : public pose::tPoseBase<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>
{
  typedef pose::tPoseBase<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPoseBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPositionElement = TElement>
  using tPosition = typename tPoseBase::template tPosition<TPositionElement>;

  template <typename TPositionElement = TElement>
  using tPositionComponent = si_units::tQuantity<TPositionSIUnit, TPositionElement>;

  template <typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientation = typename tPoseBase::template tOrientation<TOrientationElement, TOrientationAutoWrapPolicy>;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientationComponent = typename tPoseBase::template tOrientationComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  using tPoseBase::tPoseBase;

  tPose();

  template <typename TX, typename TY, typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  tPose(TX x, TY y, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation = tOrientation<TElement>::Zero());

  template <typename TX, typename TY, typename TYaw>
  tPose(TX x, TY y, TYaw yaw);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  explicit tPose(const tPose<3, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);

  inline const tPositionComponent<> &X() const
  {
    return this->Position().X();
  }
  inline tPositionComponent<> &X()
  {
    return this->Position().X();
  }

  inline const tPositionComponent<> &Y() const
  {
    return this->Position().Y();
  }
  inline tPositionComponent<> &Y()
  {
    return this->Position().Y();
  }

  inline const tOrientationComponent<> &Yaw() const
  {
    return this->Orientation().Yaw();
  }
  inline tOrientationComponent<> &Yaw()
  {
    return this->Orientation().Yaw();
  }

  using tPoseBase::SetPosition;
  using tPoseBase::SetOrientation;
  using tPoseBase::Set;

  template <typename TX, typename TY>
  void SetPosition(TX x, TY y);

  template <typename TYaw>
  void SetOrientation(TYaw yaw);

  template <typename TX, typename TY, typename TYaw>
  void Set(TX x, TY y, TYaw yaw);

  template <typename TMatrixElement>
  void Set(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error = 1E-6);

  template <typename TYaw>
  void Rotate(TYaw yaw);

  template <typename TYaw>
  tPose Rotated(TYaw yaw) const;

  TElement GetEuclideanNorm() const;

};


template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/localization/pose/tPose2D.hpp"

#endif
