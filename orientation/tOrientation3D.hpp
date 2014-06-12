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
/*!\file    rrlib/localization/orientation/tOrientation3D.cpp
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
// tOrientation3D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
tOrientation<3, TElement, TSIUnit>::tOrientation() :
  roll(0),
  pitch(0),
  yaw(0)
{}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit>::tOrientation(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit>::tOrientation(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit>
template <typename TMatrixElement>
tOrientation<3, TElement, TSIUnit>::tOrientation(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error) :
  roll(0),
  pitch(0),
  yaw(0)
{
  this->Set(matrix, max_error);
}

template <typename TElement, typename TSIUnit>
template <typename TOtherElement>
tOrientation<3, TElement, TSIUnit>::tOrientation(const tOrientation<3, TOtherElement, TSIUnit> &other) :
  roll(other.Roll()),
  pitch(other.Pitch()),
  yaw(other.Yaw())
{}

//----------------------------------------------------------------------
// tOrientation3D Set
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit>::Set(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->roll = roll;
  this->pitch = pitch;
  this->yaw = yaw;
}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit>::Set(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->roll = tComponent<>(roll);
  this->pitch = tComponent<>(pitch);
  this->yaw = tComponent<>(yaw);
}

template <typename TElement, typename TSIUnit>
template <typename TMatrixElement>
void tOrientation<3, TElement, TSIUnit>::Set(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error, bool use_second_solution)
{
  math::tAngle<TElement, math::angle::Radian> roll, pitch, yaw;
  matrix.ExtractRollPitchYaw(roll, pitch, yaw, use_second_solution, max_error);
  this->roll = tComponent<>(roll);
  this->pitch = tComponent<>(pitch);
  this->yaw = tComponent<>(yaw);
}

//----------------------------------------------------------------------
// tOrientation3D operator +=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TOtherElement>
tOrientation<3, TElement, TSIUnit> &tOrientation<3, TElement, TSIUnit>::operator += (const tOrientation<3, TOtherElement, TSIUnit> &other)
{
  this->roll += other.Roll();
  this->pitch += other.Pitch();
  this->yaw += other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation3D operator -=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TOtherElement>
tOrientation<3, TElement, TSIUnit> &tOrientation<3, TElement, TSIUnit>::operator -= (const tOrientation<3, TOtherElement, TSIUnit> &other)
{
  this->roll -= other.Roll();
  this->pitch -= other.Pitch();
  this->yaw -= other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation3D GetMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
math::tMatrix<3, 3, TElement> tOrientation<3, TElement, TSIUnit>::GetMatrix() const
{
  return math::Get3DRotationMatrixFromRollPitchYaw<TElement>(this->roll.Value(), this->pitch.Value(), this->yaw.Value());
}

//----------------------------------------------------------------------
// tOrientation3D GetHomogeneousTransformationMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
math::tMatrix<4, 4, TElement> tOrientation<3, TElement, TSIUnit>::GetHomogeneousTransformationMatrix() const
{
  math::tMatrix<3, 3, TElement> rotation = this->GetMatrix();
  return math::tMatrix<4, 4, TElement>(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                       rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                       rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                       0, 0, 0, 1);
}

//----------------------------------------------------------------------
// tOrientation3D Rotate
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit>::Rotate(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->roll += roll;
  this->pitch += pitch;
  this->yaw += yaw;
  return *this;
}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit>::Rotate(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  return this->Rotate(tComponent<>(roll), tComponent<>(pitch), tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation3D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit> tOrientation<3, TElement, TSIUnit>::Rotated(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return tOrientation(this->roll + roll, this->pitch + pitch, this->yaw + yaw);
}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit> tOrientation<3, TElement, TSIUnit>::Rotated(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return this->Rotated(tComponent<>(roll), tComponent<>(pitch), tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation3D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
const TElement tOrientation<3, TElement, TSIUnit>::GetEuclideanNorm() const
{
  return math::tVector<3, TElement>(this->roll.Value().Value(), this->pitch.Value().Value(), this->yaw.Value().Value()).Length();
}

//----------------------------------------------------------------------
// Numeric equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
bool IsEqual(const tOrientation<3, TElement, TSIUnit> &left, const tOrientation<3, TElement, TSIUnit> &right, float max_error, math::tFloatComparisonMethod method)
{
  return IsEqual(left.Roll(), right.Roll(), max_error, method) && IsEqual(left.Pitch(), right.Pitch(), max_error, method) && IsEqual(left.Yaw(), right.Yaw(), max_error, method);
}

//----------------------------------------------------------------------
// Equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
const bool operator == (const tOrientation<3, TElement, TSIUnit> &left, const tOrientation<3, TElement, TSIUnit> &right)
{
  return left.Yaw() == right.Yaw();
}

//----------------------------------------------------------------------
// Ordering
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
const bool operator < (const tOrientation<3, TElement, TSIUnit> &left, const tOrientation<3, TElement, TSIUnit> &right)
{
  return left.Roll() < right.Roll() ||
         ((left.Roll() == right.Roll() && left.Pitch().Value() < right.Pitch().Value())) ||
         ((left.Roll() == right.Roll() && left.Pitch().Value() == right.Pitch().Value()) && left.Yaw() < right.Yaw());
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
std::ostream &operator << (std::ostream &stream, const tOrientation<3, TElement, TSIUnit> &orientation)
{
  typedef math::tAngle<TElement, math::angle::Degree, math::angle::Signed> tDegreeSigned;
  return stream << "(" << tDegreeSigned(orientation.Roll().Value()) << ", " << tDegreeSigned(orientation.Pitch().Value()) << ", " << tDegreeSigned(orientation.Yaw().Value()) << ")";
}

template <typename TElement, typename TSIUnit>
std::istream &operator >> (std::istream &stream, tOrientation<3, TElement, TSIUnit> &orientation)
{
  std::istream::sentry stream_ok(stream, true);
  if (!stream_ok || stream.peek() == std::char_traits<char>::eof())
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  char a, b, c, d;
  math::tAngle<TElement, math::angle::Degree, math::angle::Signed> roll, pitch, yaw;
  stream >> a >> roll >> b >> pitch >> c >> yaw >> d;
  if (a != '(' || b != ',' || c != ',' || d != ')')
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  orientation.Set(roll, pitch, yaw);
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TSIUnit>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tOrientation<3, TElement, TSIUnit> &orientation)
{
  stream << orientation.Roll() << orientation.Pitch() << orientation.Yaw();
  return stream;
}

template <typename TElement, typename TSIUnit>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tOrientation<3, TElement, TSIUnit> &orientation)
{
  stream >> orientation.Roll() >> orientation.Pitch() >> orientation.Yaw();
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
