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

static tDataType<tPose2D<>> init_type_pose_2d("rrlib.localization.Pose2D");
static tDataType<tPose3D<>> init_type_pose_3d("rrlib.localization.Pose3D");
static tDataType<tUncertainPose2D<>> init_type_uncertain_pose_2d("rrlib.localization.UncertainPose2D");
static tDataType<tUncertainPose3D<>> init_type_uncertain_pose_3d("rrlib.localization.UncertainPose3D");

static tDataType<tTwist2D<>> init_type_twist_2d("rrlib.localization.Twist2D");
static tDataType<tTwist3D<>> init_type_twist_3d("rrlib.localization.Twist3D");
static tDataType<tUncertainTwist2D<>> init_type_uncertain_twist_2d("rrlib.localization.UncertainTwist2D");
static tDataType<tUncertainTwist3D<>> init_type_uncertain_twist_3d("rrlib.localization.UncertainTwist3D");

#endif

