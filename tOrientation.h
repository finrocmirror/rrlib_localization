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
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TValue>
tOrientation<2, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> operator * (const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation, si_units::tTime<TValue> time)
{
  typedef tOrientation<2, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> tResult;
  return tResult(orientation.Yaw() * time);
}
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TValue>
tOrientation<2, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  return orientation * time;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TValue>
tOrientation<3, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> operator * (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation, si_units::tTime<TValue> time)
{
  typedef tOrientation<3, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> tResult;
  return tResult(orientation.Roll() * time, orientation.Pitch() * time, orientation.Yaw() * time);
}
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TValue>
tOrientation<3, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  return orientation * time;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TValue>
tOrientation < 2, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap > operator / (const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation, si_units::tTime<TValue> time)
{
  typedef tOrientation < 2, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap > tResult;
  return tResult(orientation.Yaw() / time);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TValue>
tOrientation < 3, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap > operator / (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation, si_units::tTime<TValue> time)
{
  typedef tOrientation < 3, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap > tResult;
  return tResult(orientation.Roll() / time, orientation.Pitch() / time, orientation.Yaw() / time);
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
