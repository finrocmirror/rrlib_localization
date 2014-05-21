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
/*!\file    rrlib/localization/tPose.h
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-17
 *
 * \brief   Contains tPose
 *
 * \b tPose
 *
 * Representation of a 3D Pose in Cartesian space (3D Position + 3D Rotation)
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
#include "rrlib/localization/tPoseBase.h"

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
// Class declaration
//----------------------------------------------------------------------
//! Representation of a Pose in Cartesian space (Position + Rotation)
/*!
 * Representation of a Pose in Cartesian space (Position + Rotation)
 */
template <
size_t Tdimension = 3,
       typename TLinearPosition = rrlib::si_units::tLength<>,
       typename TAngularPosition = rrlib::math::tAngleRadSigned
       >
using tPose = tPoseBase<Tdimension, TLinearPosition, TAngularPosition>;

/** alias declaration for the "standard" 2D pose */
template <typename TElement = double>
using tPose2D = tPose<2, rrlib::si_units::tLength<TElement>, rrlib::math::tAngle<TElement, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;

/** alias declaration for the "standard" 3D pose */
template <typename TElement = double>
using tPose3D = tPose<3, rrlib::si_units::tLength<TElement>, rrlib::math::tAngle<TElement, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
