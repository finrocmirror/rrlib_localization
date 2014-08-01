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
/*!\file    rrlib/localization/pose/tPose2D.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
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
// tPose2D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose()
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TOrientationElement, typename TOrientationAutoWrapPolicy>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose(TX x, TY y, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation) :
  tPoseBase(tPosition<>(x, y), orientation)
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TYaw>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose(TX x, TY y, TYaw yaw) :
  tPoseBase(tPosition<>(x, y), tOrientation<>(yaw))
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose(const tPose<3, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other) :
  tPoseBase(tPosition<>(other.X(), other.Y()), tOrientation<>(other.Yaw()))
{}

//----------------------------------------------------------------------
// tPose2D SetPosition
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetPosition(TX x, TY y)
{
  this->SetPosition(tPosition<>(x, y));
}

//----------------------------------------------------------------------
// tPose2D SetOrientation
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TYaw>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetOrientation(TYaw yaw)
{
  this->SetOrientation(tOrientation<>(yaw));
}

//----------------------------------------------------------------------
// tPose2D Set
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TYaw>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Set(TX x, TY y, TYaw yaw)
{
  this->SetPosition(x, y);
  this->SetOrientation(yaw);
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Set(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error)
{
  assert(math::IsEqual(matrix.Determinant(), 1.0, max_error));
  this->SetOrientation(math::tMatrix<2, 2, TElement>(matrix[0][0], matrix[0][1], matrix[1][0], matrix[1][1]), max_error);
  this->SetPosition(tPosition<>(matrix[0][2], matrix[1][2]));
}

//----------------------------------------------------------------------
// tPose2D Rotate
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TYaw>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Rotate(TYaw yaw)
{
  this->Orientation().Rotate(yaw);
}

//----------------------------------------------------------------------
// tPose2D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TYaw>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Rotated(TYaw yaw) const
{
  tPose temp(*this);
  temp.Orientation().Rotate(yaw);
  return temp;
}

//----------------------------------------------------------------------
// tPose2D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
TElement tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::GetEuclideanNorm() const
{
  return math::tVector<3, TElement>(this->X().Value(), this->Y().Value(), this->Yaw().Value().Value()).Length();
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  typedef math::tAngle<TElement, math::angle::Degree, TAutoWrapPolicy> tDegree;
  return stream << "(" << pose.X().Value() << ", " << pose.Y().Value() << ", " << tDegree(pose.Yaw().Value()) << ")";
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  std::istream::sentry stream_ok(stream, true);
  if (!stream_ok || stream.peek() == std::char_traits<char>::eof())
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  char a, b, c, d;
  TElement x, y;
  math::tAngle<TElement, math::angle::Degree, TAutoWrapPolicy> yaw;
  stream >> a >> x >> b >> y >> c >> yaw >> d;
  if (a != '(' || b != ',' || c != ',' || d != ')')
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  pose.Set(x, y, yaw);
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream << pose.X() << pose.Y() << pose.Yaw();
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream >> pose.X() >> pose.Y() >> pose.Yaw();
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
