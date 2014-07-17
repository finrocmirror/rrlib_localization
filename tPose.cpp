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
/*!\file    rrlib/localization/tPose2D.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2014-06-05
 *
 */
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace usage
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

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

template class tPose<2, double, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
template class tPose<2, float, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
template class tPose < 2, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;
template class tPose < 2, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;

template class tPose<3, double, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
template class tPose<3, float, si_units::tMeter, si_units::tNoUnit, math::angle::Signed>;
template class tPose < 3, double, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;
template class tPose < 3, float, si_units::tSIUnit < 1, 0, -1, 0, 0, 0, 0 > , si_units::tHertz, math::angle::NoWrap >;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
