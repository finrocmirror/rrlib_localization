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
/*!\file    rrlib/localization/tPosition.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPosition
 *
 * \b tPosition
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tPosition_h__
#define __rrlib__localization__tPosition_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
using tPosition = math::tVector<Tdimension, si_units::tQuantity<TSIUnit, TElement>>;

template <typename TElement = double>
using tPosition2D = tPosition<2, TElement, si_units::tMeter>;
template <typename TElement = double>
using tPositionChange2D = tPosition < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 >>;

template <typename TElement = double>
using tPosition3D = tPosition<3, TElement, si_units::tMeter>;
template <typename TElement = double>
using tPositionChange3D = tPosition < 3, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 >>;

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TValue>
tPosition<2, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult> operator * (const tPosition<2, TElement, TSIUnit> &position, si_units::tTime<TValue> time)
{
  typedef tPosition<2, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult> tResult;
  return tResult(position.X() * time, position.Y() * time);
}
template <typename TElement, typename TSIUnit, typename TValue>
tPosition<2, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult> operator * (si_units::tTime<TValue> time, const tPosition<2, TElement, TSIUnit> &position)
{
  return position * time;
}

template <typename TElement, typename TSIUnit, typename TValue>
tPosition<3, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult> operator * (const tPosition<3, TElement, TSIUnit> &position, si_units::tTime<TValue> time)
{
  typedef tPosition<3, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult> tResult;
  return tResult(position.X() * time, position.Y() * time, position.Z() * time);
}
template <typename TElement, typename TSIUnit, typename TValue>
tPosition<3, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TSIUnit, si_units::tSecond>::tResult> operator * (si_units::tTime<TValue> time, const tPosition<3, TElement, TSIUnit> &position)
{
  return position * time;
}

template <typename TElement, typename TSIUnit, typename TValue>
tPosition < 2, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult > operator / (const tPosition<2, TElement, TSIUnit> &position, si_units::tTime<TValue> time)
{
  typedef tPosition < 2, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult > tResult;
  return tResult(position.X() / time, position.Y() / time);
}

template <typename TElement, typename TSIUnit, typename TValue>
tPosition < 3, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult > operator / (const tPosition<3, TElement, TSIUnit> &position, si_units::tTime<TValue> time)
{
  typedef tPosition < 3, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TSIUnit, si_units::tSecond>::tResult > tResult;
  return tResult(position.X() / time, position.Y() / time, position.Z() / time);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
