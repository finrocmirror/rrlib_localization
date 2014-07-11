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
/*!\file    rrlib/localization/pose/tPoseBase.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPoseBase
 *
 * \b tPoseBase
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__pose__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tPose.h" instead.
#endif

#ifndef __rrlib__localization__pose__tPose_h__
#define __rrlib__localization__pose__tPose_h__

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
#include "rrlib/localization/tPosition.h"
#include "rrlib/localization/tOrientation.h"

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

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
class tPose;

namespace pose
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
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
class tPoseBase
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const unsigned int cDIMENSION;

  template <typename TPositionElement = TElement>
  using tPosition = tPosition<Tdimension, TPositionElement, TPositionSIUnit>;

  template <typename TOrientationElement = TElement>
  using tOrientation = tOrientation<Tdimension, TOrientationElement, TOrientationSIUnit>;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = math::angle::NoWrap>
  using tOrientationComponent = typename tOrientation<>::template tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  static inline const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &Zero()
  {
    static tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> pose;
    return pose;
  }

  tPoseBase();

  template <typename TPositionElement, typename TOrientationElement = TElement>
  tPoseBase(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement> &orientation = tOrientation<TElement>::Zero());

  template <typename TMatrixElement>
  tPoseBase(const math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix, double max_error = 1E-6);

  template <typename TOtherElement>
  tPoseBase(const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit> &other);

  inline const tPosition<> &Position() const
  {
    return this->position;
  }
  inline tPosition<> &Position()
  {
    return this->position;
  }

  inline const tOrientation<> &Orientation() const
  {
    return this->orientation;
  }
  inline tOrientation<> &Orientation()
  {
    return this->orientation;
  }

  template <typename TPositionElement>
  void SetPosition(const tPosition<TPositionElement> &position);

  template <typename TOrientationElement>
  void SetOrientation(const tOrientation<TOrientationElement> &orientation);

  template <typename TMatrixElement>
  void SetOrientation(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix, double max_error = 1E-6);

  template <typename TPositionElement, typename TOrientationElement>
  void Set(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement> &orientation);

  void Reset();

  template <typename TOtherElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &operator += (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit> &other);

  template <typename TOtherElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &operator -= (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit> &other);

  math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > GetHomogeneousTransformationMatrix() const;

  template <typename TMatrixElement>
  void GetHomogeneousTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const;

  template <typename TReferenceElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> GetPoseInParentFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit> &reference) const;

  template <typename TReferenceElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> GetPoseInLocalFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit> &reference) const;

  template <typename TTranslationElement>
  void Translate(const math::tVector<Tdimension, TTranslationElement> &translation);

  template <typename TTranslationElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &Translated(const math::tVector<Tdimension, TTranslationElement> &translation);

  template <typename TFactor>
  void Scale(TFactor factor);

  template <typename TFactor>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &Scaled(TFactor factor) const;

  template <typename TTransformationElement>
  void ApplyRelativePoseTransformation(const tPose<Tdimension, TTransformationElement, TPositionSIUnit, TOrientationSIUnit> &relative_transformation);

  bool IsZero(double epsilon) const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tPosition<> position;
  tOrientation<> orientation;

};

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
bool IsEqual(const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &right, float max_error = 1E-6, math::tFloatComparisonMethod method = math::eFCM_ABSOLUTE_ERROR);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> operator - (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit > operator + (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit> &right);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit > operator - (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TFactor>
tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit> operator * (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose, TFactor factor);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TFactor>
tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit> operator * (TFactor factor, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const bool operator == (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const bool operator != (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const bool operator < (const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &right);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/localization/pose/tPoseBase.hpp"

#endif
