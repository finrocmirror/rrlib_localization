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
template <typename TElement = double>
using tUncertainPose2D = tUncertainPose<2, TElement, si_units::tMeter, si_units::tNoUnit>;
template <typename TElement = double>
using tUncertainPoseChange2D = tUncertainPose < 2, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
template <typename TElement = double>
using tUncertainTwist2D = tUncertainPoseChange2D<TElement>;

template <typename TElement = double>
using tUncertainPose3D = tUncertainPose<3, TElement, si_units::tMeter, si_units::tNoUnit>;
template <typename TElement = double>
using tUncertainPoseChange3D = tUncertainPose < 3, TElement, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
template <typename TElement = double>
using tUncertainTwist3D = tUncertainPoseChange3D<TElement>;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tUncertainPose<2, double, si_units::tMeter, si_units::tNoUnit>;
extern template class tUncertainPose<2, float, si_units::tMeter, si_units::tNoUnit>;
extern template class tUncertainPose < 2, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
extern template class tUncertainPose < 2, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;

extern template class tUncertainPose<3, double, si_units::tMeter, si_units::tNoUnit>;
extern template class tUncertainPose<3, float, si_units::tMeter, si_units::tNoUnit>;
extern template class tUncertainPose < 3, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;
extern template class tUncertainPose < 3, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz >;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
