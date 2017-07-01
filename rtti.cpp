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
/*!\file    rrlib/localization/rtti.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-05-12
 *
 * \brief   Contains Runtime Type Information initializations
 *
 */
//----------------------------------------------------------------------

#ifdef _LIB_RRLIB_RTTI_PRESENT_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/rtti.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"
#include "rrlib/localization/tUncertainPose.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::rtti;
using namespace rrlib::localization;

//----------------------------------------------------------------------
// Type initializers
//----------------------------------------------------------------------

namespace
{
tType cINIT_TYPES[] =
{
  tDataType<tPose2D<>>("rrlib.localization.Pose2D").AddName("rrlib.localization.Pose<2u, double, rrlib.si_units.SIUnit<1, 0, 0, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, 0, 0, 0, 0, 0>, rrlib.math.angle.Signed>"),
  tDataType<tPose3D<>>("rrlib.localization.Pose3D").AddName("rrlib.localization.Pose<3u, double, rrlib.si_units.SIUnit<1, 0, 0, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, 0, 0, 0, 0, 0>, rrlib.math.angle.Signed>"),
  tDataType<tUncertainPose2D<>>("rrlib.localization.UncertainPose2D").AddName("rrlib.localization.UncertainPose<2u, double, rrlib.si_units.SIUnit<1, 0, 0, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, 0, 0, 0, 0, 0>, rrlib.math.angle.Signed>"),
  tDataType<tUncertainPose3D<>>("rrlib.localization.UncertainPose3D").AddName("rrlib.localization.UncertainPose<3u, double, rrlib.si_units.SIUnit<1, 0, 0, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, 0, 0, 0, 0, 0>, rrlib.math.angle.Signed>"),

  tDataType<tTwist2D<>>("rrlib.localization.Twist2D").AddName("rrlib.localization.Pose<2u, double, rrlib.si_units.SIUnit<1, 0, -1, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, -1, 0, 0, 0, 0>, rrlib.math.angle.NoWrap>"),
  tDataType<tTwist3D<>>("rrlib.localization.Twist3D").AddName("rrlib.localization.Pose<3u, double, rrlib.si_units.SIUnit<1, 0, -1, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, -1, 0, 0, 0, 0>, rrlib.math.angle.NoWrap>"),
  tDataType<tUncertainTwist2D<>>("rrlib.localization.UncertainTwist2D").AddName("rrlib.localization.UncertainPose<2u, double, rrlib.si_units.SIUnit<1, 0, -1, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, -1, 0, 0, 0, 0>, rrlib.math.angle.NoWrap>"),
  tDataType<tUncertainTwist3D<>>("rrlib.localization.UncertainTwist3D").AddName("rrlib.localization.UncertainPose<3u, double, rrlib.si_units.SIUnit<1, 0, -1, 0, 0, 0, 0>, rrlib.si_units.SIUnit<0, 0, -1, 0, 0, 0, 0>, rrlib.math.angle.NoWrap>")
};
}

#endif

