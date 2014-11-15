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

//! The standard pose for the two dimensional case
/*! For further documentation, see \ref tPose< 2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tPose2D = tPose<2, TElement, si_units::tMeter, si_units::tNoUnit, TAutoWrapPolicy>;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
//! The standard change of pose wrt time for the two dimensional case
/*! For further documentation, see \ref tPose< 2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
using tPoseChange2D = tPose < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy >;
//! Synonym for \ref tPoseChange2D
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tTwist2D = tPoseChange2D<TElement, TAutoWrapPolicy>;

//! The standard pose for the three dimensional case
/*! For further documentation, see \ref tPose< 3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tPose3D = tPose<3, TElement, si_units::tMeter, si_units::tNoUnit, TAutoWrapPolicy>;
//! The standard change of pose wrt time for the three dimensional case
/*! For further documentation, see \ref tPose< 3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tPoseChange3D = tPose < 3, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy >;
//! Synonym for \ref tPoseChange3D
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tTwist3D = tPoseChange3D<TElement, TAutoWrapPolicy>;

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------
//! Operator to multiply a \ref tPoseChange2D with time, result is a \ref tPose2D
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tPose2D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (const tPoseChange2D<TElement, TAutoWrapPolicy> &pose_change, si_units::tTime<TValue> time)
{
  return tPose2D<decltype(TElement() * TValue()), math::angle::NoWrap>(pose_change.Position() * time, pose_change.Orientation() * time);
}
//! Operator to multiply time with a \ref tPoseChange2D, result is a \ref tPose2D
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tPose2D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tPoseChange2D<TElement, TAutoWrapPolicy> &pose_change)
{
  return pose_change * time;
}

//! Operator to multiply a \ref tPoseChange3D with time, result is a \ref tPose3D
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tPose3D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (const tPoseChange3D<TElement, TAutoWrapPolicy> &pose_change, si_units::tTime<TValue> time)
{
  return tPose3D<decltype(TElement() * TValue()), math::angle::NoWrap>(pose_change.Position() * time, pose_change.Orientation() * time);
}
//! Operator to multiply time with a \ref tPoseChange3D, result is a \ref tPose3D
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tPose3D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tPoseChange3D<TElement, TAutoWrapPolicy> &pose_change)
{
  return pose_change * time;
}

//! Operator to divide a \ref tPose2D by time, result is a \ref tPoseChange2D
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tPoseChange2D < decltype(TElement() / TValue()) > operator / (const tPose2D<TElement, TAutoWrapPolicy> &pose, si_units::tTime<TValue> time)
{
  return tPoseChange2D < decltype(TElement() / TValue()), math::angle::NoWrap > (pose.Position() / time, pose.Orientation() / time);
}

//! Operator to divide a \ref tPose3D by time, result is a \ref tPoseChange3D
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tPoseChange3D < decltype(TElement() / TValue()) > operator / (const tPose3D<TElement, TAutoWrapPolicy> &pose, si_units::tTime<TValue> time)
{
  return tPoseChange3D < decltype(TElement() / TValue()), math::angle::NoWrap > (pose.Position() / time, pose.Orientation() / time);
}

//! Operator to multiply velocity with a factor
template <unsigned int Tdimension, typename TElement, typename TAutoWrapPolicy, typename TFactor>
tPose < Tdimension, decltype(TElement() * TFactor()), si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > operator * (const tPose < Tdimension, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > &pose, TFactor factor)
{
  return tPose < Tdimension, decltype(TElement() * TFactor()), si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > (pose.Position() * factor, pose.Orientation() * factor);
}

//! Operator to multiply a factor with velocity
template <unsigned int Tdimension, typename TElement, typename TAutoWrapPolicy, typename TFactor>
tPose < Tdimension, decltype(TElement() * TFactor()), si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > operator * (TFactor factor, const tPose < Tdimension, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > &pose)
{
  return pose * factor;
}

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tPose<2, double, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tPose<2, float, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tPose < 2, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;
extern template class tPose < 2, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;

extern template class tPose<3, double, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tPose<3, float, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tPose < 3, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;
extern template class tPose < 3, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
