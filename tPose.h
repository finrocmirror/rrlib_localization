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
/*!\file    rrlib/localization/tPose.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPose
 *
 * \b tPose
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tPose_h__
#define __rrlib__localization__tPose_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#define __rrlib__localization__pose__include_guard__

#include "rrlib/localization/pose/tPose2D.h"
#include "rrlib/localization/pose/tPose3D.h"

#undef __rrlib__localization__pose__include_guard__

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
template <typename TElement = double>
using tPose2D = tPose<2, TElement, si_units::tMeter, si_units::tNoUnit>;
template <typename TElement = double>
using tPoseChange2D = tPose < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
template <typename TElement = double>
using tTwist2D = tPoseChange2D<TElement>;

template <typename TElement = double>
using tPose3D = tPose<3, TElement, si_units::tMeter, si_units::tNoUnit>;
template <typename TElement = double>
using tPoseChange3D = tPose < 3, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
template <typename TElement = double>
using tTwist3D = tPoseChange3D<TElement>;

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------
template <typename TElement, typename TValue>
tPose2D<decltype(TElement() * TValue())> operator * (const tPoseChange2D<TElement> &pose_change, si_units::tTime<TValue> time)
{
  return tPose2D<decltype(TElement() * TValue())>(pose_change.Position() * time, pose_change.Orientation() * time);
}
template <typename TElement, typename TValue>
tPose2D<decltype(TElement() * TValue())> operator * (si_units::tTime<TValue> time, const tPoseChange2D<TElement> &pose_change)
{
  return pose_change * time;
}

template <typename TElement, typename TValue>
tPose3D<decltype(TElement() * TValue())> operator * (const tPoseChange3D<TElement> &pose_change, si_units::tTime<TValue> time)
{
  return tPose3D<decltype(TElement() * TValue())>(pose_change.Position() * time, pose_change.Orientation() * time);
}
template <typename TElement, typename TValue>
tPose3D<decltype(TElement() * TValue())> operator * (si_units::tTime<TValue> time, const tPoseChange3D<TElement> &pose_change)
{
  return pose_change * time;
}

template <typename TElement, typename TValue>
tPoseChange2D < decltype(TElement() / TValue()) > operator / (const tPose2D<TElement> &pose, si_units::tTime<TValue> time)
{
  return tPoseChange2D < decltype(TElement() / TValue()) > (pose.Position() / time, pose.Orientation() / time);
}

template <typename TElement, typename TValue>
tPoseChange3D < decltype(TElement() / TValue()) > operator / (const tPose3D<TElement> &pose, si_units::tTime<TValue> time)
{
  return tPoseChange3D < decltype(TElement() / TValue()) > (pose.Position() / time, pose.Orientation() / time);
}

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tPose<2, double, si_units::tMeter, si_units::tNoUnit>;
extern template class tPose<2, float, si_units::tMeter, si_units::tNoUnit>;
extern template class tPose < 2, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
extern template class tPose < 2, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;

extern template class tPose<3, double, si_units::tMeter, si_units::tNoUnit>;
extern template class tPose<3, float, si_units::tMeter, si_units::tNoUnit>;
extern template class tPose < 3, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
extern template class tPose < 3, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
