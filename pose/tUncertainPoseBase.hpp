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
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPoseBase()
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TPose, typename TCovarianceElement>
tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPoseBase(const TPose &pose, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPose<>(pose),
  covariance(covariance)
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TPosition, typename TOrientation, typename TCovarianceElement>
tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tUncertainPoseBase(const TPosition &position, const TOrientation &orientation, const tCovarianceMatrix<TCovarianceElement> &covariance) :
  tPose<>(position, orientation),
  covariance(covariance)
{}

//----------------------------------------------------------------------
// tPoseBase Addition assignment
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::operator += (const tUncertainPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->Position() += other.Position();
  this->Orientation() += other.Orientation();
  return reinterpret_cast<tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(*this);
}

//----------------------------------------------------------------------
// tPoseBase Subtraction assignment
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tUncertainPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::operator -= (const tUncertainPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->Position() -= other.Position();
  this->Orientation() -= other.Orientation();
  return reinterpret_cast<tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(*this);
}

//----------------------------------------------------------------------
// Unary minus
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator - (const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Zero() - pose;
}

//----------------------------------------------------------------------
// Addition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tUncertainPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator + (const tUncertainPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tUncertainPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right)
{
  tUncertainPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > temp(left);
  temp += right;
  return temp;
}

//----------------------------------------------------------------------
// Subtraction
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tUncertainPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator - (const tUncertainPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tUncertainPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right)
{
  tUncertainPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > temp(left);
  temp -= right;
  return temp;
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream << static_cast<const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(pose) << pose.Covariance();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream >> static_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(pose) >> pose.Covariance();
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream << static_cast<const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(pose) << pose.Covariance();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream >> static_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(pose) >> pose.Covariance();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  std::stringstream s;
  s << pose;
  return stream << s.str();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tUncertainPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
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
