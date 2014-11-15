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
//! The position type is defined by a \ref math::tVector
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
using tPosition = math::tVector<Tdimension, si_units::tQuantity<TSIUnit, TElement>>;

//! The standard position for the two dimensional case.
template <typename TElement = double>
using tPosition2D = tPosition<2, TElement, si_units::tMeter>;
//! The standard change in position wrt time for the two dimensional case.
template <typename TElement = double>
using tPositionChange2D = tPosition < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 >>;

//! The standard position for the three dimensional case.
template <typename TElement = double>
using tPosition3D = tPosition<3, TElement, si_units::tMeter>;
//! The standard change in position wrt time for the three dimensional case.
template <typename TElement = double>
using tPositionChange3D = tPosition < 3, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 >>;

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------
//! Operator to multiply a \ref tPositionChange2D with time, result is a \ref tPosition2D
template <typename TElement, typename TValue>
tPosition2D<decltype(TElement() * TValue())> operator * (const tPositionChange2D<TElement> &position_change, si_units::tTime<TValue> time)
{
  return tPosition2D<decltype(TElement() * TValue())>(position_change.X() * time, position_change.Y() * time);
}
//! Operator to multiply time with a \ref tPositionChange2D, result is a \ref tPosition2D
template <typename TElement, typename TValue>
tPosition2D<decltype(TElement() * TValue())> operator * (si_units::tTime<TValue> time, const tPositionChange2D<TElement> &position_change)
{
  return position_change * time;
}

//! Operator to multiply a \ref tPositionChange3D with time, result is a \ref tPosition3D
template <typename TElement, typename TValue>
tPosition3D<decltype(TElement() * TValue())> operator * (const tPositionChange3D<TElement> &position_change, si_units::tTime<TValue> time)
{
  return tPosition3D<decltype(TElement() * TValue())>(position_change.X() * time, position_change.Y() * time, position_change.Z() * time);
}
//! Operator to multiply time with a \ref tPositionChange3D, result is a \ref tPosition3D
template <typename TElement, typename TValue>
tPosition3D<decltype(TElement() * TValue())> operator * (si_units::tTime<TValue> time, const tPositionChange3D<TElement> &position_change)
{
  return position_change * time;
}

//! Operator to divide a \ref tPosition2D by time, result is a \ref tPositionChange2D
template <typename TElement, typename TValue>
tPositionChange2D < decltype(TElement() / TValue()) > operator / (const tPosition2D<TElement> &position, si_units::tTime<TValue> time)
{
  return tPositionChange2D < decltype(TElement() / TValue()) > (position.X() / time, position.Y() / time);
}

//! Operator to divide a \ref tPosition3D by time, result is a \ref tPositionChange3D
template <typename TElement, typename TValue>
tPositionChange3D < decltype(TElement() / TValue()) > operator / (const tPosition3D<TElement> &position, si_units::tTime<TValue> time)
{
  return tPositionChange3D < decltype(TElement() / TValue()) > (position.X() / time, position.Y() / time, position.Z() / time);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
