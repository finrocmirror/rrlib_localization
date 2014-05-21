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
/*!\file    rrlib/localization/tTwist.h
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-17
 *
 * \brief   Contains tTwist
 *
 * \b tTwist
 *
 * Representation of velocity in 3D Cartesian space (linear and angular velocities)
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tTwist_h__
#define __rrlib__localization__tTwist_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
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
//! Representation of velocity in 3D Cartesian space (linear and angular velocities
/*!
 * Representation of velocity in 3D Cartesian space (linear and angular velocities)
 */
template <
size_t Tdimension = 3,
       typename TLinearVelocity = rrlib::si_units::tVelocity<>,
       typename TAngularVelocity = rrlib::si_units::tFrequency<rrlib::math::tAngleRadSigned>
       >
using tTwist = tPoseBase<Tdimension, TLinearVelocity, TAngularVelocity>;

/** alias declaration for the "standard" 2D twist */
template <typename TElement = double>
using tTwist2D = tTwist<2, rrlib::si_units::tVelocity<TElement>, rrlib::si_units::tFrequency<double>>;
// does not work yet, because angle it bounded
//using tTwist2D = tTwist<2, rrlib::si_units::tVelocity<TElement>, rrlib::si_units::tFrequency<rrlib::math::tAngle<TElement, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>>;

/** alias declaration for the "standard" 3D twist */
template <typename TElement = double>
using tTwist3D = tTwist<3, rrlib::si_units::tVelocity<TElement>, rrlib::si_units::tFrequency<double>>;
// does not work yet, because angle it bounded
//using tTwist3D = tTwist<3, rrlib::si_units::tVelocity<TElement>, rrlib::si_units::tFrequency<rrlib::math::tAngle<TElement, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>>;


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
