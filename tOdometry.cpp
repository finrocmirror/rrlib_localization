//
// You received this file as part of RRLib
// Robotics Research Library
//
//Copyright (C) AG Robotersysteme TU Kaiserslautern
//
//This program is free software; you can redistribute it and/or
//modify it under the terms of the GNU General Public License
//as published by the Free Software Foundation; either version 2
//of the License, or (at your option) any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-
//----------------------------------------------------------------------
/*!\file    tOdometry.cpp
 *
 * \author  Jens Wettach
 * \date    2011-01-17
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// non MCA Includes - include with <>
// MCA Includes - include with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tOdometry.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{

//----------------------------------------------------------------------
// typedefs and enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// defines and consts
// global vars (prefer static class vars!)
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward class declarations
// Extern methods
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// class tOdometry constructor
//----------------------------------------------------------------------
tOdometry::tOdometry(bool _use_timestamp, math::tPose3D _initial_pose)
    :
    pose_changed(0),
    vel_veh(0.0f),
    av_z_veh(0.0f),
    side_slip_angle(0.0f),
    elapsed_time(0.0f),
    timer(),
    data_select(0),
    reset(false),
    use_timestamp(_use_timestamp),
    reset_timer(true),
    data_changed(0),
    current_pose_wcs(_initial_pose),
    previous_pose_wcs(_initial_pose),
    ci_delta_pose_wcs(math::tPose3D::Zero()),
    delta_pose_lcs(math::tPose3D::Zero()),
    velocity_conversion(math::tPose3D::Zero()),
    vel_vector_lcs(math::tPose3D::Zero()),
    vel_vector_wcs(math::tPose3D::Zero()),
    av_vector_wcs(math::tPose3D::Zero()),
    local_side_slip_rotation(math::tPose3D::Zero()),
    previous_pose_wcs_matrix(math::tMat4x4d::Identity())
{}

//----------------------------------------------------------------------
// class tOdometry destructor
//----------------------------------------------------------------------
tOdometry::~tOdometry()
{}

//----------------------------------------------------------------------
// class tOdometry CorrectPose()
//----------------------------------------------------------------------
void tOdometry::CorrectPose(
  bool _reset,
  int _pose_changed,
  const math::tPose3D& _current_pose_wcs,
  const math::tPose3D& _delta_pose_wcs)
{
  if (_reset)
  {
    reset = true;
  }
  else
  {
    reset = false;
    if (data_select == 1 && this->pose_changed != _pose_changed)
    {
      this->pose_changed = _pose_changed;
      this->current_pose_wcs = _current_pose_wcs;
    }
    if (data_select == 2 && this->pose_changed != _pose_changed)
    {
      this->ci_delta_pose_wcs = _delta_pose_wcs;
      this->current_pose_wcs += this->ci_delta_pose_wcs;
    }
  }
}// CorrectPose()

//----------------------------------------------------------------------
// class tOdometry UpdateTime()
//----------------------------------------------------------------------
void tOdometry::UpdateTime(const util::tTime& internal_time, const util::tTime& external_time)
{
  if (!this->use_timestamp)
  {
    if (this->reset_timer)
    {
      this->reset_timer = false;
      timer = internal_time;
    }
    elapsed_time = (internal_time - timer).ToUSec() * 0.000001;
    timer = internal_time;
  }
  else
  {
    if (this->reset_timer)
    {
      this->reset_timer = false;
      timer = external_time;
    }

    elapsed_time = (external_time - timer).ToUSec() * 0.000001;
    timer = external_time;
  }
} // UpdateTime()


//----------------------------------------------------------------------
// class tOdometry UpdatePose()
//----------------------------------------------------------------------
bool tOdometry::UpdatePose(double vel_veh, double av_z_veh, double side_slip_angle)
{
  if (this->reset)
  {
    this->current_pose_wcs.Reset();
    this->previous_pose_wcs.Reset();
    return false;
  }
  else
  {
    if (elapsed_time > 0)
    {
      //@todo: add trapez calculation for improvement

      //first reset poses:
      this->delta_pose_lcs.Reset();
      this->vel_vector_lcs.Reset();
      this->vel_vector_wcs.Reset();
      this->av_vector_wcs.Reset();
      this->local_side_slip_rotation.Reset();
      this->velocity_conversion.Reset();

      //pose containing the angular offset due to the side slip angle:
      this->local_side_slip_rotation.SetOrientation(0.0, 0.0, side_slip_angle);

      //calculate local pose change and velocity vector
      if (av_z_veh != 0.)
      {
        this->delta_pose_lcs.Set(
          vel_veh / av_z_veh * sin(av_z_veh * elapsed_time),
          vel_veh / av_z_veh *(1. - cos(av_z_veh * elapsed_time)),
          this->delta_pose_lcs.Z());
        this->vel_vector_lcs.Set(
          this->delta_pose_lcs.X() / elapsed_time,
          this->delta_pose_lcs.Y() / elapsed_time,
          this->vel_vector_lcs.Z());
      }
      else
      {
        this->delta_pose_lcs.Set(vel_veh * elapsed_time, this->delta_pose_lcs.Y(), this->delta_pose_lcs.Z());
        this->vel_vector_lcs.Set(vel_veh, this->vel_vector_lcs.Y(), this->vel_vector_lcs.Z());
      }

      //rotate vectors in lcs according to side slip:
      math::tMat4x4d local_side_slip_rotation_matrix = local_side_slip_rotation.GetTransformationMatrix();
      this->delta_pose_lcs.Set(local_side_slip_rotation_matrix * delta_pose_lcs.GetTransformationMatrix(), false);
      this->vel_vector_lcs.Set(local_side_slip_rotation_matrix * vel_vector_lcs.GetTransformationMatrix(), false);

      //after the orientation of the vector is correct, set the rotation:
      this->delta_pose_lcs.SetOrientation(
        delta_pose_lcs.Roll(),
        delta_pose_lcs.Pitch(),
        av_z_veh * elapsed_time);

      //velocity conversion vector: only rotation
      velocity_conversion = current_pose_wcs;
      velocity_conversion.Set(0., 0., 0.);

      math::tMat4x4d current_pose_wcs_matrix = current_pose_wcs.GetTransformationMatrix();
      math::tMat3x3d current_pose_wcs_rotation_matrix = current_pose_wcs.GetRotationMatrix();

      this->vel_vector_wcs.Set(this->velocity_conversion.GetTransformationMatrix() * vel_vector_lcs.GetTransformationMatrix(), false);

      this->previous_pose_wcs.Set(current_pose_wcs_matrix, false);
      current_pose_wcs_matrix *= delta_pose_lcs.GetTransformationMatrix();
      this->current_pose_wcs.Set(current_pose_wcs_matrix, false);

      //use difference to determine angular velocities:
      av_vector_wcs.SetOrientation(
        math::tAngleRad(current_pose_wcs.Roll() - previous_pose_wcs.Roll()) / elapsed_time,
        math::tAngleRad(current_pose_wcs.Pitch() - previous_pose_wcs.Pitch()) / elapsed_time,
        math::tAngleRad(current_pose_wcs.Yaw() - previous_pose_wcs.Yaw()) / elapsed_time);
      this->data_changed = (this->data_changed + 1) % 1000;
    }
    return true;
  }
} // UpdatePose()

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
}
}
