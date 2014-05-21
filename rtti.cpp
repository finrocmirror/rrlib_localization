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
/*!\file    rrlib/math/rtti.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-05-12
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
#include "rrlib/localization/tPoseWithUncertainty.h"
#include "rrlib/localization/tTwist.h"
#include "rrlib/localization/tTwistWithUncertainty.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::rtti;

//----------------------------------------------------------------------
// Type initializers
//----------------------------------------------------------------------

static tDataType<rrlib::localization::tPose2D<>> init_type_pose_2d;
static tDataType<rrlib::localization::tPose3D<>> init_type_pose_3d;
static tDataType<rrlib::localization::tPoseWithUncertainty2D<>> init_type_pose_with_uncertainty_2d;
static tDataType<rrlib::localization::tPoseWithUncertainty3D<>> init_type_pose_with_uncertainty_3d;

static tDataType<rrlib::localization::tTwist2D<>> init_type_twist_2d;
static tDataType<rrlib::localization::tTwist3D<>> init_type_twist_3d;
static tDataType<rrlib::localization::tTwistWithUncertainty2D<>> init_type_twist_with_uncertainty_2d;
static tDataType<rrlib::localization::tTwistWithUncertainty3D<>> init_type_twist_with_uncertainty_3d;

#endif

