//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    rrlib/localization/tDeadReckoning.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-06-05
 *
 */
//----------------------------------------------------------------------
#include "rrlib/localization/tDeadReckoning.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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
tDeadReckoning::tDeadReckoning(const tPose &initial_pose) : pose(initial_pose), previous_twist_available(false)
{
}

void tDeadReckoning::SetPose(const tPose &pose)
{
  this->pose = pose;
}

const tDeadReckoning::tPose & tDeadReckoning::GetPose() const
{
  return this->pose;
}

void tDeadReckoning::ResetTwist()
{
  this->previous_twist = tTwist();
  this->previous_twist_available = false;
}

void tDeadReckoning::UpdatePose(const tTwist &twist, const rrlib::time::tDuration &elapsed_time)
{
  if (previous_twist_available)
    UpdatePose(this->pose, this->previous_twist, twist, elapsed_time);
  else
    UpdatePose(this->pose, twist, elapsed_time);

  previous_twist = twist;
  previous_twist_available = true;
}


void tDeadReckoning::UpdatePose(tPose &pose, const tTwist &twist, const rrlib::time::tDuration &elapsed_time)
{
  rrlib::si_units::tTime<double> elapsed(std::chrono::duration_cast<std::chrono::duration<double>>(elapsed_time).count());
  pose.ApplyRelativePoseTransformation(twist * elapsed);
}

void tDeadReckoning::UpdatePose(tPose &pose, const tTwist &previous_twist, const tTwist &twist, const rrlib::time::tDuration &elapsed_time)
{
  // do the trapezoidal calculation
  UpdatePose(pose, (previous_twist + twist) * 0.5, elapsed_time);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
