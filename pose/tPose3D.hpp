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
/*!\file    rrlib/localization/pose/tPose3D.cpp
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
// tPose3D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose()
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TZ, typename TOrientationElement, typename TOrientationAutoWrapPolicy>
tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose(TX x, TY y, TZ z, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation) :
  tPoseBase(tPosition<>(x, y, z), orientation)
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TZ, typename TRoll, typename TPitch, typename TYaw>
tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose(TX x, TY y, TZ z, TRoll roll, TPitch pitch, TYaw yaw) :
  tPoseBase(tPosition<>(x, y, z), tOrientation<>(roll, pitch, yaw))
{}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPose(const tPose<2, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other) :
  tPoseBase(tPosition<>(other.X(), other.Y(), tPositionComponent<>()), tOrientation<>(tOrientationComponent<>(), tOrientationComponent<>(), other.Yaw()))
{}

//----------------------------------------------------------------------
// tPose3D SetPosition
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TZ>
void tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetPosition(TX x, TY y, TZ z)
{
  this->SetPosition(tPosition<>(x, y, z));
}

//----------------------------------------------------------------------
// tPose3D SetOrientation
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TRoll, typename TPitch, typename TYaw>
void tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetOrientation(TRoll roll, TPitch pitch, TYaw yaw)
{
  this->SetOrientation(tOrientation<>(roll, pitch, yaw));
}

//----------------------------------------------------------------------
// tPose3D Set
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TX, typename TY, typename TZ, typename TRoll, typename TPitch, typename TYaw>
void tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Set(TX x, TY y, TZ z, TRoll roll, TPitch pitch, TYaw yaw)
{
  this->SetPosition(x, y, z);
  this->SetOrientation(roll, pitch, yaw);
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Set(const math::tMatrix<4, 4, TMatrixElement> &matrix, double max_error)
{
  assert(math::IsEqual(matrix.Determinant(), 1.0, max_error));
  this->SetOrientation(math::tMatrix<3, 3, TElement>(matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1], matrix[2][2]), max_error);
  this->SetPosition(tPosition<>(matrix[0][3], matrix[1][3], matrix[2][3]));
}

//----------------------------------------------------------------------
// tPose3D Rotate
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TRoll, typename TPitch, typename TYaw>
void tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Rotate(TRoll roll, TPitch pitch, TYaw yaw)
{
  this->Orientation().Rotate(roll, pitch, yaw);
}

//----------------------------------------------------------------------
// tPose3D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TRoll, typename TPitch, typename TYaw>
tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Rotated(TRoll roll, TPitch pitch, TYaw yaw) const
{
  tPose temp(*this);
  temp.Orientation().Rotate(roll, pitch, yaw);
  return temp;
}

//----------------------------------------------------------------------
// tPose3D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
TElement tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::GetEuclideanNorm() const
{
  return math::tVector<6, TElement>(this->X().Value(), this->Y().Value(), this->Z().Value(), this->Roll().Value().Value(), this->Pitch().Value().Value(), this->Yaw().Value().Value()).Length();
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  typedef math::tAngle<TElement, math::angle::Degree, TAutoWrapPolicy> tDegree;
  return stream << "(" << pose.X().Value() << ", " << pose.Y().Value() << ", " << pose.Z().Value() << ", " << tDegree(pose.Roll().Value()) << ", " << tDegree(pose.Pitch().Value()) << ", " << tDegree(pose.Yaw().Value()) << ")";
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  std::istream::sentry stream_ok(stream, true);
  if (!stream_ok || stream.peek() == std::char_traits<char>::eof())
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  char a, b, c, d, e, f, g;
  TElement x, y, z;
  math::tAngle<TElement, math::angle::Degree, TAutoWrapPolicy> roll, pitch, yaw;
  stream >> a >> x >> b >> y >> c >> z >> d >> roll >> e >> pitch >> f >> yaw >> g;
  if (a != '(' || b != ',' || c != ',' || d != ',' || e != ',' || f != ',' || g != ')')
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  pose.Set(x, y, z, roll, pitch, yaw);
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream << pose.X() << pose.Y() << pose.Z() << pose.Roll() << pose.Pitch() << pose.Yaw();
}

template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return stream >> pose.X() >> pose.Y() >> pose.Z() >> pose.Roll() >> pose.Pitch() >> pose.Yaw();
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
