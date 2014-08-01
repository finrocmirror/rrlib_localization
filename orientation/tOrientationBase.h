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
/*!\file    rrlib/localization/orientation/tOrientationBase.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tOrientationBase
 *
 * \b tOrientationBase
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__orientation__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tOrientation.h" instead.
#endif

#ifndef __rrlib__localization__orientation__tOrientation_h__
#define __rrlib__localization__orientation__tOrientation_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tVector.h"
#include "rrlib/si_units/si_units.h"

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/serialization.h"
#include <sstream>
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
class tOrientation;

namespace orientation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
class tOrientationBase
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const unsigned int cDIMENSION;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tComponent = si_units::tQuantity<TSIUnit, math::tAngle<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>>;

  static const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &Zero()
  {
    static tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> orientation;
    return orientation;
  }

  inline const tComponent<> &operator[](unsigned int i) const
  {
    return const_cast<tOrientationBase &>(*this)[i];
  }
  inline tComponent<> &operator[](unsigned int i)
  {
    if (i > tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::cSIZE - 1)
    {
      std::stringstream stream;
      stream << "Component index (" << i << ") out of bounds [0.." << tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy>::cSIZE - 1 << "].";
      throw std::logic_error(stream.str());
    }
    return reinterpret_cast<tComponent<> *>(this)[i];
  }

  void Reset();

  template <typename TMatrixElement>
  void GetMatrix(math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix) const;

  template <typename TMatrixElement>
  void GetTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const;

  template <typename TMatrixElement>
  void Rotate(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix);

  template <typename TMatrixElement>
  tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> Rotated(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix) const;

  bool IsZero(double epsilon = 1E-6) const;

};

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> operator - (const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tOrientation < Tdimension, decltype(TLeftElement() + TRightElement()), TSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator + (const tOrientation<Tdimension, TLeftElement, TSIUnit, TLeftAutoWrapPolicy> &left, const tOrientation<Tdimension, TRightElement, TSIUnit, TRightAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tOrientation < Tdimension, decltype(TLeftElement() - TRightElement()), TSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator - (const tOrientation<Tdimension, TLeftElement, TSIUnit, TLeftAutoWrapPolicy> &left, const tOrientation<Tdimension, TRightElement, TSIUnit, TRightAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
bool operator != (const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &left, const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &right);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

template <unsigned int Tdimension, typename TElement, typename TSIUnit, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tOrientation<Tdimension, TElement, TSIUnit, TAutoWrapPolicy> &orientation);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/localization/orientation/tOrientationBase.hpp"

#endif
