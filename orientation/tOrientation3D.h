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
/*!\file    rrlib/localization/orientation/tOrientation3D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tOrientation3D
 *
 * \b tOrientation3D
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__orientation__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tOrientation.h" instead.
#endif

#ifndef __rrlib__localization__orientation__tOrientation3D_h__
#define __rrlib__localization__orientation__tOrientation3D_h__

#include "rrlib/localization/orientation/tOrientationBase.h"

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
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
class tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> : public orientation::tOrientationBase<3, TElement, TSIUnit, TAutoWrapPolicy>
{
  typedef orientation::tOrientationBase<3, TElement, TSIUnit, TAutoWrapPolicy> tOrientationBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const size_t cSIZE;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tComponent = typename tOrientationBase::template tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  tOrientation();

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  tOrientation(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw);

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  tOrientation(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw);

  template <typename TMatrixElement>
  tOrientation(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error = 1E-6);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tOrientation(const tOrientation<3, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other);

  inline const tComponent<> &Roll() const
  {
    return this->roll;
  }
  inline tComponent<> &Roll()
  {
    return this->roll;
  }
  inline const tComponent<> &Pitch() const
  {
    return this->pitch;
  }
  inline tComponent<> &Pitch()
  {
    return this->pitch;
  }
  inline const tComponent<> &Yaw() const
  {
    return this->yaw;
  }
  inline tComponent<> &Yaw()
  {
    return this->yaw;
  }

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  void Set(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw);

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  void Set(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw);

  template <typename TMatrixElement>
  void Set(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error = 1E-6, bool use_second_solution = false);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tOrientation &operator += (const tOrientation<3, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tOrientation &operator -= (const tOrientation<3, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other);

  math::tMatrix<3, 3, TElement> GetMatrix() const;

  math::tMatrix<4, 4, TElement> GetHomogeneousTransformationMatrix() const;

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  void Rotate(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw);

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  void Rotate(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw);

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  tOrientation Rotated(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const;

  template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
  tOrientation Rotated(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const;

  const TElement GetEuclideanNorm() const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tComponent<> roll;
  tComponent<> pitch;
  tComponent<> yaw;

};

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
bool IsEqual(const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &right, float max_error = 1E-6, math::tFloatComparisonMethod method = math::eFCM_ABSOLUTE_ERROR);

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator == (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &right);

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator < (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &right);

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/localization/orientation/tOrientation3D.hpp"

#endif
