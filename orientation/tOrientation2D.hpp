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
/*!\file    rrlib/localization/orientation/tOrientation2D.cpp
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
const size_t tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::cSIZE;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tOrientation2D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation() :
  yaw(0)
{}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(const math::tMatrix<2, 2, TMatrixElement> &matrix, double max_error)
{
  this->Set(matrix, max_error);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::tOrientation(const tOrientation<2, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other) :
  yaw(other.Yaw())
{}

//----------------------------------------------------------------------
// tOrientation2D Set
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Set(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->yaw = yaw;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Set(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->yaw = tComponent<>(yaw);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Set(const math::tMatrix<2, 2, TMatrixElement> &matrix, double max_error)
{
  assert(math::IsEqual(matrix.Determinant(), 1.0, max_error));
  this->yaw = tComponent<>(std::atan2(matrix[1][0], matrix[0][0]));
}

//----------------------------------------------------------------------
// tOrientation2D operator +=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::operator += (const tOrientation<2, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->yaw += other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation2D operator -=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::operator -= (const tOrientation<2, TOtherElement, TSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->yaw -= other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation2D GetMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
math::tMatrix<2, 2, TElement> tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::GetMatrix() const
{
  return math::Get2DRotationMatrix<TElement>(this->yaw.Value());
}

//----------------------------------------------------------------------
// tOrientation2D GetTransformationMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
math::tMatrix<3, 3, TElement> tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::GetTransformationMatrix() const
{
  math::tMatrix<2, 2, TElement> rotation = this->GetMatrix();
  return math::tMatrix<3, 3, TElement>(rotation[0][0], rotation[0][1], 0,
                                       rotation[1][0], rotation[1][1], 0,
                                       0, 0, 1);
}

//----------------------------------------------------------------------
// tOrientation2D Rotate
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Rotate(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->yaw += yaw;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Rotate(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->Rotate(tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation2D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Rotated(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return tOrientation(this->yaw + yaw);
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::Rotated(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return this->Rotated(tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation2D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const TElement tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy>::GetEuclideanNorm() const
{
  return this->yaw.Value().Value();
}

//----------------------------------------------------------------------
// Numeric equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
bool IsEqual(const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &right, float max_error, math::tFloatComparisonMethod method)
{
  return IsEqual(left.Yaw(), right.Yaw(), max_error, method);
}

//----------------------------------------------------------------------
// Multiplication
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TFactor>
tOrientation <2, decltype(TElement() * TFactor()), TSIUnit, TAutoWrapPolicy> operator * (const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation, TFactor factor)
{
  tOrientation <2, decltype(TElement() * TFactor()), TSIUnit, TAutoWrapPolicy> temp(orientation);
  temp.Yaw() = orientation.Yaw() * factor;
  return temp;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy, typename TFactor>
tOrientation <2, decltype(TElement() * TFactor()), TSIUnit, TAutoWrapPolicy> operator * (TFactor factor, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  return orientation * factor;
}

//----------------------------------------------------------------------
// Equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator == (const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &right)
{
  return left.Yaw() == right.Yaw();
}

//----------------------------------------------------------------------
// Ordering
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator < (const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &right)
{
  return left.Yaw() < right.Yaw();
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  return stream << math::tAngle<TElement, math::angle::Degree, math::angle::Signed>(orientation.Yaw().Value());
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  std::istream::sentry stream_ok(stream, true);
  if (!stream_ok || stream.peek() == std::char_traits<TElement>::eof())
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }
  math::tAngle<TElement, math::angle::Degree, math::angle::Signed> degree;
  stream >> degree;
  orientation.Set(degree);

  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  stream << orientation.Yaw();
  return stream;
}

template <typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tOrientation<2, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  stream >> orientation.Yaw();
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
