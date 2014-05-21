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
/*!\file    rrlib/localization/tPose.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-17
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{

/* explicit class instantiation of common template types */
// TODO: unfortunately the explicit class instantiation does not seem to work with alias templates
//template class tPose2D<float>;
//template class tPoseBase<2, rrlib::si_units::tLength<float>, rrlib::math::tAngle<float, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;
//template class tPose2D<double>;
template class tPoseBase<2, rrlib::si_units::tLength<double>, rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;
//template class tPose3D<float>;
//template class tPoseBase<3, rrlib::si_units::tLength<float>, rrlib::math::tAngle<float, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;
//template class tPose3D<double>;
template class tPoseBase<3, rrlib::si_units::tLength<double>, rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

