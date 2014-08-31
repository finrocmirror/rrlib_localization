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
/*!\file    rrlib/localization/tUncertainPose.h
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-17
 *
 * \brief   Contains tUncertainPose
 *
 * \b tUncertainPose
 *
 * Representation of a Pose in Cartesian space with uncertainty (attached covariance matrix)
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tUncertainPose_h__
#define __rrlib__localization__tUncertainPose_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#define __rrlib__localization__uncertain_pose__include_guard__

#include "rrlib/localization/pose/tUncertainPose2D.h"
#include "rrlib/localization/pose/tUncertainPose3D.h"

#undef __rrlib__localization__uncertain_pose__include_guard__

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
using tUncertainPose2D = tUncertainPose<2, TElement, si_units::tMeter, si_units::tNoUnit, TAutoWrapPolicy>;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tUncertainPoseChange2D = tUncertainPose < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy >;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tUncertainTwist2D = tUncertainPoseChange2D<TElement, TAutoWrapPolicy>;

template <typename TElement = double, typename TAutoWrapPolicy = math::angle::Signed>
using tUncertainPose3D = tUncertainPose<3, TElement, si_units::tMeter, si_units::tNoUnit, TAutoWrapPolicy>;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tUncertainPoseChange3D = tUncertainPose < 3, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy >;
template <typename TElement = double, typename TAutoWrapPolicy = math::angle::NoWrap>
using tUncertainTwist3D = tUncertainPoseChange3D<TElement, TAutoWrapPolicy>;

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tUncertainPose2D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (const tUncertainPoseChange2D<TElement, TAutoWrapPolicy> &pose_change, si_units::tTime<TValue> time)
{
  // TODO: transform the uncertainties
  return tUncertainPose2D<decltype(TElement() * TValue()), math::angle::NoWrap>(pose_change.Position() * time, pose_change.Orientation() * time);
}
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tUncertainPose2D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tUncertainPoseChange2D<TElement, TAutoWrapPolicy> &pose_change)
{
  // TODO: transform the uncertainties
  return pose_change * time;
}

template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tUncertainPose3D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (const tUncertainPoseChange3D<TElement, TAutoWrapPolicy> &pose_change, si_units::tTime<TValue> time)
{
  // TODO: transform the uncertainties
  return tUncertainPose3D<decltype(TElement() * TValue()), math::angle::NoWrap>(pose_change.Position() * time, pose_change.Orientation() * time);
}
template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tUncertainPose3D<decltype(TElement() * TValue()), math::angle::NoWrap> operator * (si_units::tTime<TValue> time, const tUncertainPoseChange3D<TElement, TAutoWrapPolicy> &pose_change)
{
  // TODO: transform the uncertainties
  return pose_change * time;
}

template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tUncertainPoseChange2D < decltype(TElement() / TValue()) > operator / (const tUncertainPose2D<TElement, TAutoWrapPolicy> &pose, si_units::tTime<TValue> time)
{
  // TODO: transform the uncertainties
  return tUncertainPoseChange2D < decltype(TElement() / TValue()), math::angle::NoWrap > (pose.Position() / time, pose.Orientation() / time);
}

template <typename TElement, typename TAutoWrapPolicy, typename TValue>
tUncertainPoseChange3D < decltype(TElement() / TValue()) > operator / (const tUncertainPose3D<TElement, TAutoWrapPolicy> &pose, si_units::tTime<TValue> time)
{
  // TODO: transform the uncertainties
  return tUncertainPoseChange3D < decltype(TElement() / TValue()), math::angle::NoWrap > (pose.Position() / time, pose.Orientation() / time);
}

// Multiplication is defined for the first derivatives so that velocities can be multiplied with a factor
template <unsigned int Tdimension, typename TElement, typename TAutoWrapPolicy, typename TFactor>
tUncertainPose < Tdimension, decltype(TElement() * TFactor()), si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > operator * (const tUncertainPose < Tdimension, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > &pose, TFactor factor)
{
  // TODO: transform the uncertainties
  return tUncertainPose < Tdimension, decltype(TElement() * TFactor()), si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > (pose.Position() * factor, pose.Orientation() * factor);
}

template <unsigned int Tdimension, typename TElement, typename TAutoWrapPolicy, typename TFactor>
tUncertainPose < Tdimension, decltype(TElement() * TFactor()), si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > operator * (TFactor factor, const tUncertainPose < Tdimension, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, TAutoWrapPolicy > &pose)
{
  return pose * factor;
}

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tUncertainPose<2, double, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tUncertainPose<2, float, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tUncertainPose < 2, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;
extern template class tUncertainPose < 2, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;

extern template class tUncertainPose<3, double, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tUncertainPose<3, float, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
extern template class tUncertainPose < 3, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;
extern template class tUncertainPose < 3, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
