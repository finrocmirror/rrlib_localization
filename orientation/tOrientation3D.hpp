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
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const size_t tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::cSIZE;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tOrientation3D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation() :
  roll(0),
  pitch(0),
  yaw(0)
{}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error) :
  roll(0),
  pitch(0),
  yaw(0)
{
  this->Set(matrix, max_error);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(const tOrientation<3, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other) :
  roll(other.Roll()),
  pitch(other.Pitch()),
  yaw(other.Yaw())
{}

//----------------------------------------------------------------------
// tOrientation3D Set
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Set(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->roll = roll;
  this->pitch = pitch;
  this->yaw = yaw;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Set(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->roll = tComponent<>(roll);
  this->pitch = tComponent<>(pitch);
  this->yaw = tComponent<>(yaw);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Set(const math::tMatrix<3, 3, TMatrixElement> &matrix, double max_error, bool use_second_solution)
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
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::operator += (const tOrientation<3, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->roll += other.Roll();
  this->pitch += other.Pitch();
  this->yaw += other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation3D operator -=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::operator -= (const tOrientation<3, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->roll -= other.Roll();
  this->pitch -= other.Pitch();
  this->yaw -= other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation3D GetMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
math::tMatrix<3, 3, TElement> tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::GetMatrix() const
{
  return math::Get3DRotationMatrixFromRollPitchYaw<TElement>(this->roll.Value(), this->pitch.Value(), this->yaw.Value());
}

//----------------------------------------------------------------------
// tOrientation3D GetTransformationMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
math::tMatrix<4, 4, TElement> tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::GetTransformationMatrix() const
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
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Rotate(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->roll += roll;
  this->pitch += pitch;
  this->yaw += yaw;
  return *this;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Rotate(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  return this->Rotate(tComponent<>(roll), tComponent<>(pitch), tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation3D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Rotated(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return tOrientation(this->roll + roll, this->pitch + pitch, this->yaw + yaw);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::Rotated(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> roll, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> pitch, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return this->Rotated(tComponent<>(roll), tComponent<>(pitch), tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation3D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const TElement tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy>::GetEuclideanNorm() const
{
  return math::tVector<3, TElement>(this->roll.Value().Value(), this->pitch.Value().Value(), this->yaw.Value().Value()).Length();
}

//----------------------------------------------------------------------
// Numeric equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
bool IsEqual(const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &right, float max_error, math::tFloatComparisonMethod method)
{
  return IsEqual(left.Roll(), right.Roll(), max_error, method) && IsEqual(left.Pitch(), right.Pitch(), max_error, method) && IsEqual(left.Yaw(), right.Yaw(), max_error, method);
}

//----------------------------------------------------------------------
// Multiplication
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TFactor>
tOrientation <3, decltype(TElement() * TFactor()), TSIUnit, TAutoWrapPolicy> operator * (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation, TFactor factor)
{
  tOrientation <3, decltype(TElement() * TFactor()), TSIUnit, TAutoWrapPolicy> temp(orientation);
  temp.Roll() = orientation.Roll() * factor;
  temp.Pitch() = orientation.Pitch() * factor;
  temp.Yaw() = orientation.Yaw() * factor;
  return temp;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TFactor>
tOrientation <3, decltype(TElement() * TFactor()), TSIUnit, TAutoWrapPolicy> operator * (TFactor factor, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  return orientation * factor;
}

//----------------------------------------------------------------------
// Equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator == (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &right)
{
  return left.Roll() == right.Roll() && left.Pitch() == right.Pitch() && left.Yaw() == right.Yaw();
}

//----------------------------------------------------------------------
// Ordering
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator < (const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &right)
{
  return left.Roll() < right.Roll() ||
         ((left.Roll() == right.Roll() && left.Pitch() < right.Pitch())) ||
         ((left.Roll() == right.Roll() && left.Pitch() == right.Pitch()) && left.Yaw() < right.Yaw());
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  typedef math::tAngle<TElement, math::angle::Degree, math::angle::Signed> tDegreeSigned;
  return stream << "(" << tDegreeSigned(orientation.Roll().Value()) << ", " << tDegreeSigned(orientation.Pitch().Value()) << ", " << tDegreeSigned(orientation.Yaw().Value()) << ")";
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
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

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  stream << orientation.Roll() << orientation.Pitch() << orientation.Yaw();
  return stream;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tOrientation<3, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
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
