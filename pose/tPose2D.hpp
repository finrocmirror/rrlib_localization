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
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::tPose()
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TX, typename TY, typename TOrientationElement>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::tPose(TX x, TY y, const tOrientation<TOrientationElement> &orientation) :
  tPoseBase(tPosition<>(x, y), orientation)
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TX, typename TY, typename TYaw>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::tPose(TX x, TY y, TYaw yaw) :
  tPoseBase(tPosition<>(x, y), tOrientation<>(yaw))
{}

//----------------------------------------------------------------------
// tPose2D SetPosition
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TX, typename TY>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::SetPosition(TX x, TY y)
{
  this->SetPosition(tPosition<>(x, y));
}

//----------------------------------------------------------------------
// tPose2D SetOrientation
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TYaw>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::SetOrientation(TYaw yaw)
{
  this->SetOrientation(tOrientation<>(yaw));
}

//----------------------------------------------------------------------
// tPose2D Set
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TX, typename TY, typename TYaw>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::Set(TX x, TY y, TYaw yaw)
{
  this->SetPosition(x, y);
  this->SetOrientation(yaw);
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TMatrixElement>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::Set(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error)
{
  assert(math::IsEqual(matrix.Determinant(), 1.0, max_error));
  this->SetOrientation(math::tMatrix<2, 2, TElement>(matrix[0][0], matrix[0][1], matrix[1][0], matrix[1][1]), max_error);
  this->SetPosition(tPosition<>(matrix[0][2], matrix[1][2]));
}

//----------------------------------------------------------------------
// tPose2D Rotate
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TYaw>
void tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::Rotate(TYaw yaw)
{
  this->Orientation().Rotate(yaw);
}

//----------------------------------------------------------------------
// tPose2D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TYaw>
tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::Rotated(TYaw yaw) const
{
  tPose temp(*this);
  temp.Orientation().Rotate(yaw);
  return temp;
}

//----------------------------------------------------------------------
// tPose2D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const TElement tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit>::GetEuclideanNorm() const
{
  return math::tVector<3, TElement>(this->X().Value(), this->Y().Value(), this->Yaw().Value().Value()).Length();
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
std::ostream &operator << (std::ostream &stream, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  typedef math::tAngle<TElement, math::angle::Degree, math::angle::Signed> tDegreeSigned;
  return stream << "(" << pose.X().Value() << ", " << pose.Y().Value() << ", " << tDegreeSigned(pose.Yaw().Value()) << ")";
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
std::istream &operator >> (std::istream &stream, tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  std::istream::sentry stream_ok(stream, true);
  if (!stream_ok || stream.peek() == std::char_traits<char>::eof())
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  char a, b, c, d;
  TElement x, y;
  math::tAngle<TElement, math::angle::Degree, math::angle::Signed> yaw;
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

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  stream << pose.X() << pose.Y() << pose.Yaw();
  return stream;
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  stream >> pose.X() >> pose.Y() >> pose.Yaw();
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
