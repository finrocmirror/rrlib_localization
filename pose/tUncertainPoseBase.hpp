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
/*!\file    rrlib/localization/pose/tUncertainPoseBase.hpp
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-21
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include <sstream>
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{
namespace pose
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

//----------------------------------------------------------------------
// tUncertainPoseBase constructors
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tUncertainPoseBase()
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TPose, typename TCovarianceElement>
tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tUncertainPoseBase(const TPose &pose, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPose<>(pose),
  covariance(covariance)
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TPosition, typename TOrientation, typename TCovarianceElement>
tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tUncertainPoseBase(const TPosition &position, const TOrientation &orientation, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPose<>(position, orientation),
  covariance(covariance)
{}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
std::ostream &operator << (std::ostream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  return stream << static_cast<const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &>(pose) << pose.Covariance();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
std::istream &operator >> (std::istream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  return stream >> static_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &>(pose) >> pose.Covariance();
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  return stream << static_cast<const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &>(pose) << pose.Covariance();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  return stream >> static_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &>(pose) >> pose.Covariance();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  std::stringstream s;
  s << pose;
  return stream << s.str();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  stream.GetWrappedStringStream() >> pose;
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
