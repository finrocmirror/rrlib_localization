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
/*!\file    rrlib/localization/orientation/tOrientationBase.cpp
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
namespace orientation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const unsigned int tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::cDIMENSION = Tdimension;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tOrientationBase Reset
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
void tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::Reset()
{
  std::memset(this, 0, sizeof(tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>));
}

//----------------------------------------------------------------------
// tOrientationBase GetMatrix
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::GetMatrix(math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix) const
{
  matrix = reinterpret_cast<tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> *>(this)->GetMatrix();
}

//----------------------------------------------------------------------
// tOrientationBase GetTransformationMatrix
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::GetTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const
{
  matrix = reinterpret_cast<const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> *>(this)->GetTransformationMatrix();
}

//----------------------------------------------------------------------
// tOrientationBase Rotate
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::Rotate(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix)
{
  this->Set(matrix * reinterpret_cast<tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> *>(this)->GetMatrix());
  return *this;
}

//----------------------------------------------------------------------
// tOrientationBase Rotated
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::Rotated(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix) const
{
  tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> temp(*this);
  temp.Rotate(matrix);
  return temp;
}

//----------------------------------------------------------------------
// tOrientationBase IsZero
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
bool tOrientationBase<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::IsZero(double epsilon) const
{
  return IsEqual(*this, tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::Zero(), epsilon);
}

//----------------------------------------------------------------------
// Unary minus
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> operator - (const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  return tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::Zero() - orientation;
}

//----------------------------------------------------------------------
// Addition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tOrientation < Tdimension, decltype(TLeftElement() + TRightElement()), TSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator + (const tOrientation<Tdimension, TLeftElement, TSIUnit, TLeftAutoWrapPolicy> &left, const tOrientation<Tdimension, TRightElement, TSIUnit, TRightAutoWrapPolicy> &right)
{
  tOrientation < Tdimension, decltype(TLeftElement() + TRightElement()), TSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > temp(left);
  temp += right;
  return temp;
}

//----------------------------------------------------------------------
// Subtraction
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tOrientation < Tdimension, decltype(TLeftElement() - TRightElement()), TSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator - (const tOrientation<Tdimension, TLeftElement, TSIUnit, TLeftAutoWrapPolicy> &left, const tOrientation<Tdimension, TRightElement, TSIUnit, TRightAutoWrapPolicy> &right)
{
  tOrientation < Tdimension, decltype(TLeftElement() - TRightElement()), TSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > temp(left);
  temp -= right;
  return temp;
}

//----------------------------------------------------------------------
// Comparison
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
const bool operator != (const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &right)
{
  return !(left == right);
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  std::stringstream s;
  s << orientation;
  stream << s.str();
  return stream;
}

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &orientation)
{
  stream.GetWrappedStringStream() >> orientation;
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
