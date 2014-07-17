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
template <typename TElement, typename TSIUnit>
const size_t tOrientation<2, TElement, TSIUnit>::cSIZE = 1;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tOrientation2D constructors
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
tOrientation<2, TElement, TSIUnit>::tOrientation() :
  yaw(0)
{}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit>::tOrientation(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit>::tOrientation(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) :
  yaw(yaw)
{}

template <typename TElement, typename TSIUnit>
template <typename TMatrixElement>
tOrientation<2, TElement, TSIUnit>::tOrientation(const math::tMatrix<2, 2, TMatrixElement> &matrix, double max_error)
{
  this->Set(matrix, max_error);
}

template <typename TElement, typename TSIUnit>
template <typename TOtherElement>
tOrientation<2, TElement, TSIUnit>::tOrientation(const tOrientation<2, TOtherElement, TSIUnit> &other) :
  yaw(other.Yaw())
{}

//----------------------------------------------------------------------
// tOrientation2D Set
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit>::Set(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->yaw = yaw;
}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit>::Set(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->yaw = tComponent<>(yaw);
}

template <typename TElement, typename TSIUnit>
template <typename TMatrixElement>
void tOrientation<2, TElement, TSIUnit>::Set(const math::tMatrix<2, 2, TMatrixElement> &matrix, double max_error)
{
  assert(math::IsEqual(matrix.Determinant(), 1.0, max_error));
  this->yaw = tComponent<>(std::atan2(matrix[1][0], matrix[0][0]));
}

//----------------------------------------------------------------------
// tOrientation2D operator +=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TOtherElement>
tOrientation<2, TElement, TSIUnit> &tOrientation<2, TElement, TSIUnit>::operator += (const tOrientation<2, TOtherElement, TSIUnit> &other)
{
  this->yaw += other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation2D operator -=
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TOtherElement>
tOrientation<2, TElement, TSIUnit> &tOrientation<2, TElement, TSIUnit>::operator -= (const tOrientation<2, TOtherElement, TSIUnit> &other)
{
  this->yaw -= other.Yaw();
  return *this;
}

//----------------------------------------------------------------------
// tOrientation2D GetMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
math::tMatrix<2, 2, TElement> tOrientation<2, TElement, TSIUnit>::GetMatrix() const
{
  return math::Get2DRotationMatrix<TElement>(this->yaw.Value());
}

//----------------------------------------------------------------------
// tOrientation2D GetHomogeneousTransformationMatrix
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
math::tMatrix<3, 3, TElement> tOrientation<2, TElement, TSIUnit>::GetHomogeneousTransformationMatrix() const
{
  math::tMatrix<2, 2, TElement> rotation = this->GetMatrix();
  return math::tMatrix<3, 3, TElement>(rotation[0][0], rotation[0][1], 0,
                                       rotation[1][0], rotation[1][1], 0,
                                       0, 0, 1);
}

//----------------------------------------------------------------------
// tOrientation2D Rotate
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit>::Rotate(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->yaw += yaw;
}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
void tOrientation<2, TElement, TSIUnit>::Rotate(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw)
{
  this->Rotate(tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation2D Rotated
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit> tOrientation<2, TElement, TSIUnit>::Rotated(tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return tOrientation(this->yaw + yaw);
}

template <typename TElement, typename TSIUnit>
template <typename TAngleElement, typename TAngleUnitPolicy, typename TAngleAutoWrapPolicy>
tOrientation<2, TElement, TSIUnit> tOrientation<2, TElement, TSIUnit>::Rotated(math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy> yaw) const
{
  return this->Rotated(tComponent<>(yaw));
}

//----------------------------------------------------------------------
// tOrientation2D GetEuclideanNorm
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
const TElement tOrientation<2, TElement, TSIUnit>::GetEuclideanNorm() const
{
  return this->yaw.Value().Value();
}

//----------------------------------------------------------------------
// Numeric equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
bool IsEqual(const tOrientation<2, TElement, TSIUnit> &left, const tOrientation<2, TElement, TSIUnit> &right, float max_error, math::tFloatComparisonMethod method)
{
  return IsEqual(left.Yaw(), right.Yaw(), max_error, method);
}

//----------------------------------------------------------------------
// Equality
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
const bool operator == (const tOrientation<2, TElement, TSIUnit> &left, const tOrientation<2, TElement, TSIUnit> &right)
{
  return left.Yaw() == right.Yaw();
}

//----------------------------------------------------------------------
// Ordering
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
const bool operator < (const tOrientation<2, TElement, TSIUnit> &left, const tOrientation<2, TElement, TSIUnit> &right)
{
  return left.Yaw() < right.Yaw();
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
template <typename TElement, typename TSIUnit>
std::ostream &operator << (std::ostream &stream, const tOrientation<2, TElement, TSIUnit> &orientation)
{
  return stream << math::tAngle<TElement, math::angle::Degree, math::angle::Signed>(orientation.Yaw().Value());
}

template <typename TElement, typename TSIUnit>
std::istream &operator >> (std::istream &stream, tOrientation<2, TElement, TSIUnit> &orientation)
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

template <typename TElement, typename TSIUnit>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tOrientation<2, TElement, TSIUnit> &orientation)
{
  stream << orientation.Yaw();
  return stream;
}

template <typename TElement, typename TSIUnit>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tOrientation<2, TElement, TSIUnit> &orientation)
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
