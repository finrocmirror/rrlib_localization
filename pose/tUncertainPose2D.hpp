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
// tUncertainPose2D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tUncertainPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::tUncertainPose()
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TX, typename TY, typename TYaw, typename TCovarianceElement>
tUncertainPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::tUncertainPose(TX x, TY y, TYaw yaw, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPoseBase(tPose2D<>(x, y, yaw), covariance)
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
