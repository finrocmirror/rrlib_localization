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
/*!\file    rrlib/localization/tOrientation.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tOrientation
 *
 * \b tOrientation
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tOrientation_h__
#define __rrlib__localization__tOrientation_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#define __rrlib__localization__orientation__include_guard__

#include "rrlib/localization/orientation/tOrientation2D.h"
#include "rrlib/localization/orientation/tOrientation3D.h"

#undef __rrlib__localization__orientation__include_guard__

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
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tOrientation2D = tOrientation<2, TElement, si_units::tNoUnit, TAutoWrapPolicy>;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tOrientationChange2D = tOrientation<2, TElement, si_units::tHertz, TAutoWrapPolicy>;

template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tOrientation3D = tOrientation<3, TElement, si_units::tNoUnit, TAutoWrapPolicy>;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tOrientationChange3D = tOrientation<3, TElement, si_units::tHertz, TAutoWrapPolicy>;

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------
template <typename TElement, typename TValue>
tOrientation2D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (const tOrientationChange2D<TElement> &orientation_change, si_units::tTime<TValue> time)
{
  return tOrientation2D<decltype(TElement() * TValue()), math::angle::NoWrap>(orientation_change.Yaw() * time);
}
template <typename TElement, typename TValue>
tOrientation2D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tOrientationChange2D<TElement> &orientation_change)
{
  return orientation_change * time;
}

template <typename TElement, typename TValue>
tOrientation3D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (const tOrientationChange3D<TElement> &orientation_change, si_units::tTime<TValue> time)
{
  return tOrientation3D<decltype(TElement() * TValue()), math::angle::NoWrap>(orientation_change.Roll() * time, orientation_change.Pitch() * time, orientation_change.Yaw() * time);
}
template <typename TElement, typename TValue>
tOrientation3D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tOrientationChange3D<TElement> &orientation_change)
{
  return orientation_change * time;
}

template <typename TElement, typename TValue>
tOrientationChange2D < decltype(TElement() / TValue()) > operator / (const tOrientation2D<TElement> &orientation, si_units::tTime<TValue> time)
{
  return tOrientationChange2D < decltype(TElement() / TValue()) > (orientation.Yaw() / time);
}

template <typename TElement, typename TValue>
tOrientationChange3D < decltype(TElement() / TValue()) > operator / (const tOrientation3D<TElement> &orientation, si_units::tTime<TValue> time)
{
  return tOrientationChange3D < decltype(TElement() / TValue()) > (orientation.Roll() / time, orientation.Pitch() / time, orientation.Yaw() / time);
}

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tOrientation<2, double, si_units::tNoUnit, math::angle::Signed>;
extern template class tOrientation<2, float, si_units::tNoUnit, math::angle::Signed>;
extern template class tOrientation<2, double, si_units::tHertz, math::angle::NoWrap>;
extern template class tOrientation<2, float, si_units::tHertz, math::angle::NoWrap>;

extern template class tOrientation<3, double, si_units::tNoUnit, math::angle::Signed>;
extern template class tOrientation<3, float, si_units::tNoUnit, math::angle::Signed>;
extern template class tOrientation<3, double, si_units::tHertz, math::angle::NoWrap>;
extern template class tOrientation<3, float, si_units::tHertz, math::angle::NoWrap>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
