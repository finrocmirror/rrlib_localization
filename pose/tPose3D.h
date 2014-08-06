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
/*!\file    rrlib/localization/pose/tPose3D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPose3D
 *
 * \b tPose3D
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__pose__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tPose.h" instead.
#endif

#ifndef __rrlib__localization__pose__tPose3D_h__
#define __rrlib__localization__pose__tPose3D_h__

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
class tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> : public pose::tPoseBase<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>
{
  typedef pose::tPoseBase<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPoseBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPositionElement = typename tPoseBase::tElement>
  using tPosition = typename tPoseBase::template tPosition<TPositionElement>;

  template <typename TPositionElement = typename tPoseBase::tElement>
  using tPositionComponent = si_units::tQuantity<TPositionSIUnit, TPositionElement>;

  template <typename TOrientationElement = typename tPoseBase::tElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientation = typename tPoseBase::template tOrientation<TOrientationElement, TOrientationAutoWrapPolicy>;

  template <typename TAngleElement = typename tPoseBase::tElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientationComponent = typename tPoseBase::template tOrientationComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  using tPoseBase::tPoseBase;

  tPose();

  template <typename TX, typename TY, typename TZ, typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  tPose(TX x, TY y, TZ z, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation = tOrientation<>::Zero());

  template <typename TX, typename TY, typename TZ, typename TRoll, typename TPitch, typename TYaw>
  tPose(TX x, TY y, TZ z, TRoll roll, TPitch pitch, TYaw yaw);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  explicit tPose(const tPose<2, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);

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

  inline const tPositionComponent<> &Z() const
  {
    return this->Position().Z();
  }
  inline tPositionComponent<> &Z()
  {
    return this->Position().Z();
  }

  inline const tOrientationComponent<> &Roll() const
  {
    return this->Orientation().Roll();
  }
  inline tOrientationComponent<> &Roll()
  {
    return this->Orientation().Roll();
  }

  inline const tOrientationComponent<> &Pitch() const
  {
    return this->Orientation().Pitch();
  }
  inline tOrientationComponent<> &Pitch()
  {
    return this->Orientation().Pitch();
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

  template <typename TX, typename TY, typename TZ>
  void SetPosition(TX x, TY y, TZ z);

  template <typename TRoll, typename TPitch, typename TYaw>
  void SetOrientation(TRoll roll, TPitch pitch, TYaw yaw);

  template <typename TX, typename TY, typename TZ, typename TRoll, typename TPitch, typename TYaw>
  void Set(TX x, TY y, TZ, TRoll roll, TPitch pitch, TYaw yaw);

  template <typename TMatrixElement>
  void Set(const math::tMatrix<4, 4, TMatrixElement> &matrix, double max_error = 1E-6);

  using tPoseBase::Rotate;

  template <typename TRoll, typename TPitch, typename TYaw>
  void Rotate(TRoll roll, TPitch pitch, TYaw yaw);

  using tPoseBase::Rotated;

  template <typename TRoll, typename TPitch, typename TYaw>
  tPose Rotated(TRoll roll, TPitch pitch, TYaw yaw) const;

  TElement GetEuclideanNorm() const;

};


template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/localization/pose/tPose3D.hpp"

#endif
