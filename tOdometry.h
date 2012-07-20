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
/*!\file    tOdometry.h
 *
 * \author  Jens Wettach
 * \date    2011-01-17
 *
 * \brief   Contains tOdometry
 *
 * \b tOdometry
 *
 * A few words for tOdometry
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_localization_tOdometry_h_
#define _rrlib_localization_tOdometry_h_

//----------------------------------------------------------------------
// Global includes - include with <>
// Local includes - include with ""
//----------------------------------------------------------------------
#include "rrlib/math/tPose3D.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/util/tTime.h"
#include "rrlib/logging/messages.h"

//----------------------------------------------------------------------
// typedefs and enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// defines and consts
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{
//----------------------------------------------------------------------
// Forward class declarations
// Extern methods
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
//! Short description of tOdometry
/*! A more detailed description of tOdometry, which
    Jens Wettach hasn't done yet !!
*/
class tOdometry
{
public:
  /*!
   */
  tOdometry(
    bool use_timestamp,
    math::tPose3D initial_pose = math::tPose3D::Zero()
  );

  /*!
   */
  ~tOdometry();

  /*!
    functions
   */
  inline const char *Description() const
  {
    return "tOdometry";
  }

  bool UseTimestamp() const
  {
    return this->use_timestamp;
  }

  void SetDataSelect(int val)
  {
    this->data_select = val;
  }

  void SetUseTimeStamp(bool val)
  {
    if (val && !this->use_timestamp)
    {
      this->reset_timer = true;
    }
    if (!val && this->use_timestamp)
    {
      this->reset_timer = true;
    }
    this->use_timestamp = val;
    RRLIB_LOG_PRINTF(DEBUG_VERBOSE_1, "Setting use timestamp to %d.\n", this->use_timestamp);
  }

  const math::tPose3D& CurrentPoseWcs() const
  {
    return this->current_pose_wcs;
  }

  const math::tPose3D& VelocityVectorWcs() const
  {
    return this->vel_vector_wcs;
  }

  const math::tPose3D& AngularVelocityVectorWcs() const
  {
    return this->av_vector_wcs;
  }

  void CorrectPose(
    bool _reset,
    int _pose_changed,
    const math::tPose3D& _current_pose_wcs,
    const math::tPose3D& _delta_pose_wcs);

  void UpdateTime(const util::tTime& internal_time, const util::tTime& external_time);

  bool UpdatePose(double velocity, double angular_velocity, double side_slip_angle);

  int DataChanged() const
  {
    return this->data_changed;
  }

private:

  int pose_changed;

  float vel_veh;
  float av_z_veh;
  float side_slip_angle;

  float elapsed_time;
  util::tTime timer;

  int data_select;
  bool reset;
  bool use_timestamp;
  bool reset_timer;

  int data_changed;

  math::tPose3D current_pose_wcs, previous_pose_wcs, ci_delta_pose_wcs, delta_pose_lcs;
  math::tPose3D velocity_conversion;
  math::tPose3D vel_vector_lcs;
  math::tPose3D vel_vector_wcs, av_vector_wcs;
  math::tPose3D local_side_slip_rotation;
  math::tMat4x4d previous_pose_wcs_matrix;
};
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}; // namespace localization
}; // namespace rrlib
#endif
