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
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
const unsigned int tOrientationBase<Tdimension, TElement, TSIUnit>::cDIMENSION = Tdimension;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tOrientationBase Reset
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
void tOrientationBase<Tdimension, TElement, TSIUnit>::Reset()
{
  std::memset(this, 0, sizeof(tOrientation<Tdimension, TElement, TSIUnit>));
}

//----------------------------------------------------------------------
// tOrientationBase GetMatrix
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
template <typename TMatrixElement>
void tOrientationBase<Tdimension, TElement, TSIUnit>::GetMatrix(math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix) const
{
  matrix = reinterpret_cast<tOrientation<Tdimension, TElement, TSIUnit> *>(this)->GetMatrix();
}

//----------------------------------------------------------------------
// tOrientationBase GetHomogeneousTransformationMatrix
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
template <typename TMatrixElement>
void tOrientationBase<Tdimension, TElement, TSIUnit>::GetHomogeneousTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const
{
  matrix = reinterpret_cast<tOrientation<Tdimension, TElement, TSIUnit> *>(this)->GetHomogeneousTransformationMatrix();
}

//----------------------------------------------------------------------
// tOrientationBase Rotate
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
template <typename TMatrixElement>
void tOrientationBase<Tdimension, TElement, TSIUnit>::Rotate(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix)
{
  this->Set(matrix * reinterpret_cast<tOrientation<Tdimension, TElement, TSIUnit> *>(this)->GetMatrix());
  return *this;
}

//----------------------------------------------------------------------
// tOrientationBase Rotated
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
template <typename TMatrixElement>
tOrientation<Tdimension, TElement, TSIUnit> tOrientationBase<Tdimension, TElement, TSIUnit>::Rotated(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix) const
{
  tOrientation<Tdimension, TElement, TSIUnit> temp(*this);
  temp.Rotate(matrix);
  return temp;
}

//----------------------------------------------------------------------
// tOrientationBase IsZero
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
bool tOrientationBase<Tdimension, TElement, TSIUnit>::IsZero(double epsilon) const
{
  return IsEqual(*this, tOrientation<Tdimension, TElement, TSIUnit>::Zero(), epsilon);
}

//----------------------------------------------------------------------
// Unary minus
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
tOrientation<Tdimension, TElement, TSIUnit> operator - (const tOrientation<Tdimension, TElement, TSIUnit> &orientation)
{
  return tOrientation<Tdimension, TElement, TSIUnit>::Zero() - orientation;
}

//----------------------------------------------------------------------
// Addition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TSIUnit>
tOrientation < Tdimension, decltype(TLeftElement() + TRightElement()), TSIUnit > operator + (const tOrientation<Tdimension, TLeftElement, TSIUnit> &left, const tOrientation<Tdimension, TRightElement, TSIUnit> &right)
{
  tOrientation < Tdimension, decltype(TLeftElement() + TRightElement()), TSIUnit > temp(left);
  temp += right;
  return temp;
}

//----------------------------------------------------------------------
// Subtraction
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TSIUnit>
tOrientation < Tdimension, decltype(TLeftElement() - TRightElement()), TSIUnit > operator - (const tOrientation<Tdimension, TLeftElement, TSIUnit> &left, const tOrientation<Tdimension, TRightElement, TSIUnit> &right)
{
  tOrientation < Tdimension, decltype(TLeftElement() - TRightElement()), TSIUnit > temp(left);
  temp -= right;
  return temp;
}

//----------------------------------------------------------------------
// Comparison
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TSIUnit>
const bool operator != (const tOrientation<Tdimension, TElement, TSIUnit> &left, const tOrientation<Tdimension, TElement, TSIUnit> &right)
{
  return !(left == right);
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TSIUnit>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tOrientation<Tdimension, TElement, TSIUnit> &orientation)
{
  std::stringstream s;
  s << orientation;
  stream << s.str();
  return stream;
}

template <unsigned int Tdimension, typename TElement, typename TSIUnit>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tOrientation<Tdimension, TElement, TSIUnit> &orientation)
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
