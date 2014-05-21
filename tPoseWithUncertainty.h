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
/*!\file    rrlib/localization/tPoseWithUncertainty.h
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-17
 *
 * \brief   Contains tPoseWithUncertainty
 *
 * \b tPoseWithUncertainty
 *
 * Representation of a 3D Pose in Cartesian space with uncertainty (attached covariance matrix)
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tPoseWithUncertainty_h__
#define __rrlib__localization__tPoseWithUncertainty_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/si_units/si_units.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tPoseWithUncertaintyBase.h"

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
//! Representation of a Pose in Cartesian space with uncertainty
/*!
 * Representation of a Pose in Cartesian space with uncertainty (attached covariance matrix)
 */
template <
size_t Tdimension = 3,
       typename TLinearPosition = rrlib::si_units::tLength<>,
       typename TAngularPosition = rrlib::math::tAngleRadSigned
       >
using tPoseWithUncertainty = tPoseWithUncertaintyBase<Tdimension, TLinearPosition, TAngularPosition>;

/** alias declaration for the "standard" 2D pose with uncertainty */
template <typename TElement = double>
using tPoseWithUncertainty2D = tPoseWithUncertainty<2, rrlib::si_units::tLength<TElement>, rrlib::math::tAngle<TElement, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;

/** alias declaration for the "standard" 3D pose with uncertainty */
template <typename TElement = double>
using tPoseWithUncertainty3D = tPoseWithUncertainty<3, rrlib::si_units::tLength<TElement>, rrlib::math::tAngle<TElement, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
