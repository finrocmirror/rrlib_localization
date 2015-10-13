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
 * \brief   Contains definitions of common poses in Cartesian space
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

//! The standard pose for the two dimensional case.
/*! For further documentation, see \ref tPose< 2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tPose2D = tPose<2, TElement, si_units::tMeter, si_units::tNoUnit, TAutoWrapPolicy>;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
//! The standard change of pose wrt time for the two dimensional case.
/*! For further documentation, see \ref tPose< 2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
using tPoseChange2D = tPose < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy >;
//! Synonym for \ref tPoseChange2D
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tTwist2D = tPoseChange2D<TElement, TAutoWrapPolicy>;

//! The standard pose for the three dimensional case.
/*! For further documentation, see \ref tPose< 3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy >
 */
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tPose3D = tPose<3, TElement, si_units::tMeter, si_units::tNoUnit, TAutoWrapPolicy>;
//! The standard change of pose wrt time for the three dimensional case.
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
//! Operator to multiply a (maybe derived) \ref tPose with time (intergrate over time)
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TValue>
tPose<Tdimension, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TPositionSIUnit, si_units::tSecond>::tResult, typename si_units::operators::tProduct<TOrientationSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> operator * (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose, si_units::tTime<TValue> time)
{
  typedef tPose<Tdimension, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TPositionSIUnit, si_units::tSecond>::tResult, typename si_units::operators::tProduct<TOrientationSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> tResult;
  return tResult(pose.Position() * time, pose.Orientation() * time);
}
//! Operator to multiply time with a (maybe derived) \ref tPose (intergrate over time)
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TValue>
tPose<Tdimension, decltype(TElement() * TValue()), typename si_units::operators::tProduct<TPositionSIUnit, si_units::tSecond>::tResult, typename si_units::operators::tProduct<TOrientationSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return pose * time;
}

//! Operator to divide a (maybe derived) \ref tPose by time (differentiate by time)
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TValue>
tPose < Tdimension, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TPositionSIUnit, si_units::tSecond>::tResult, typename si_units::operators::tQuotient<TOrientationSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap > operator / (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose, si_units::tTime<TValue> time)
{
  typedef tPose < Tdimension, decltype(TElement() / TValue()), typename si_units::operators::tQuotient<TPositionSIUnit, si_units::tSecond>::tResult, typename si_units::operators::tQuotient<TOrientationSIUnit, si_units::tSecond>::tResult, math::angle::NoWrap > tResult;
  return tResult(pose.Position() / time, pose.Orientation() / time);
}

//! Operator to multiply velocity with a factor.
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TFactor>
tPose<Tdimension, decltype(TElement() * typename std::enable_if<std::is_scalar<TFactor>::value, TFactor>::type()), TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator * (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose, TFactor factor)
{
  typedef tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tResult;
  return tResult(pose.Position() * factor, pose.Orientation() * factor);
}
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TFactor>
tPose<Tdimension, decltype(TElement() * typename std::enable_if<std::is_scalar<TFactor>::value, TFactor>::type()), TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator * (TFactor factor, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
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
