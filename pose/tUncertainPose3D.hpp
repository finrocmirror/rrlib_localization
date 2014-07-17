//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    rrlib/localization/pose/tUncertainPose3D.hpp
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-21
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include <sstream>
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tUncertainPose3D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tUncertainPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPose()
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TZ, typename TCovarianceElement>
tUncertainPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPose(TX x, TY y, TZ z, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPoseBase(tPose3D<>(x, y, z), covariance)
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename T1, typename T2, typename T3, typename T4, typename TCovarianceElement>
tUncertainPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPose(T1 arg_1, T2 arg_2, T3 arg_3, T4 arg_4, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPoseBase(tPose3D<>(arg_1, arg_2, arg_3, arg_4), covariance)
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TZ, typename TRoll, typename TPitch, typename TYaw, typename TCovarianceElement>
tUncertainPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPose(TX x, TY y, TZ z, TRoll roll, TPitch pitch, TYaw yaw, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPoseBase(tPose3D<>(x, y, z, roll, pitch, yaw), covariance)
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
