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
/*!\file    rrlib/localization/tDeadReckoning.h
 *
 * \author  Michael Arndt
 *
 * \date    2014-06-05
 *
 * \brief   Contains \ref rrlib::localization::tDeadReckoning
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tDeadReckoning_h__
#define __rrlib__localization__tDeadReckoning_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tUncertainPose.h"

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
//! Class containing functions to perform dead reckoning.
/** The calculations are based on trapezoidal approximation of the integration.
  * The class can either be instantiated or the static methods can be used, depending on what is more useful.
  */
class tDeadReckoning
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  //! The pose type used in this class
  typedef tUncertainPose3D<> tPose;
  //! The twist (linear and angular velocities) type used in this class
  typedef tUncertainTwist3D<> tTwist;

  //! Create a new dead reckoning instance with the specified initial pose
  /** @param initial_pose The initial pose to initialize the object with
   */
  tDeadReckoning(const tPose &initial_pose = tPose());

  //! Set the internal pose to a different one
  /** @param pose The pose to set internal state to
    */
  void SetPose(const tPose &pose);

  //! Get the internal pose
  /** @return The pose
    */
  const tPose & GetPose() const;

  //! Reset the internal twist.
  /** By calling this method, the previous twist will be marked as invalid and thus not used for integration.
    */
  void ResetTwist();

  //! Update the internal pose using the twist as well as the elapsed time.
  /** @param twist The linear and angular velocities
    * @param elapsed_time The elapsed time
    */
  void UpdatePose(const tTwist &twist, const rrlib::time::tDuration &elapsed_time);

  //! Update the specified pose using the twist as well as the elapsed time
  /** This is a static member and thus needs no object to operate on.
    * As the previous twist is not known in this method, the integration can only be approximated using the midpoint rule (also called rectangle rule).
    * For more precise results, use the method that also takes the previous twist into account.
    *
    * @param pose The pose to be updated
    * @param twist The linear and angular velocities
    * @param elapsed_time The elapsed time
    */
  static void UpdatePose(tPose &pose, const tTwist &twist, const rrlib::time::tDuration &elapsed_time);

  //! Update the specified pose using the twist as well as the elapsed time
  /** This is a static member and thus needs no object to operate on.
    * This method takes the previous twist into account to be able to approximate the integral using the trapezoidal rule.
    *
    * @param pose The pose to be updated
    * @param previous_twist The linear and angular velocities of the previous time-step
    * @param twist The currentlinear and angular velocities
    * @param elapsed_time The elapsed time
    */
  static void UpdatePose(tPose &pose, const tTwist &previous_twist, const tTwist &twist, const rrlib::time::tDuration &elapsed_time);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tPose pose;
  tTwist previous_twist;

  //! Indicates whether the previous twist is available, i.e. if we have been updated at least once
  bool previous_twist_available;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
