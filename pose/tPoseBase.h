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

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
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
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
class tPoseBase
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const unsigned int cDIMENSION;

  template <typename TPositionElement = TElement>
  using tPosition = tPosition<Tdimension, TPositionElement, TPositionSIUnit>;

  template <typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientation = tOrientation<Tdimension, TOrientationElement, TOrientationSIUnit, TOrientationAutoWrapPolicy>;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientationComponent = typename tOrientation<>::template tComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  static inline const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &Zero()
  {
    static tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> pose;
    return pose;
  }

  tPoseBase();

  template <typename TPositionElement, typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  tPoseBase(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation = tOrientation<TElement, TOrientationAutoWrapPolicy>::Zero());

  template <typename TMatrixElement>
  tPoseBase(const math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix, double max_error = 1E-6);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tPoseBase(const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);

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

  template <typename TOrientationElement, typename TOrientationAutoWrapPolicy>
  void SetOrientation(const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation);

  template <typename TMatrixElement>
  void SetOrientation(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix, double max_error = 1E-6);

  template <typename TPositionElement, typename TOrientationElement, typename TOrientationAutoWrapPolicy>
  void Set(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation);

  void Reset();

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &operator += (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);

  template <typename TOtherElement, typename TOtherAutoWrapPolicy>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &operator -= (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other);

  math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > GetHomogeneousTransformationMatrix() const;

  template <typename TMatrixElement>
  void GetHomogeneousTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const;

  template <typename TReferenceElement, typename TReferenceAutoWrapPolicy>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> GetPoseInParentFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit, TReferenceAutoWrapPolicy> &reference) const;

  template <typename TReferenceElement, typename TReferenceAutoWrapPolicy>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> GetPoseInLocalFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit, TReferenceAutoWrapPolicy> &reference) const;

  template <typename TTranslationElement>
  void Translate(const math::tVector<Tdimension, TTranslationElement> &translation);

  template <typename TTranslationElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &Translated(const math::tVector<Tdimension, TTranslationElement> &translation);

  template <typename TRotationElement>
  void Rotate(const math::tMatrix<Tdimension, Tdimension, TRotationElement> &rotation);

  template <typename TRotationElement>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &Rotated(const math::tMatrix<Tdimension, Tdimension, TRotationElement> &rotation);

  template <typename TFactor>
  void Scale(TFactor factor);

  template <typename TFactor>
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &Scaled(TFactor factor) const;

  template <typename TTransformationElement, typename TTransformationAutoWrapPolicy>
  void ApplyRelativePoseTransformation(const tPose<Tdimension, TTransformationElement, TPositionSIUnit, TOrientationSIUnit, TTransformationAutoWrapPolicy> &relative_transformation);

  bool IsZero(double epsilon) const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tPosition<> position;
  tOrientation<> orientation;

};

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
bool IsEqual(const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right, float max_error = 1E-6, math::tFloatComparisonMethod method = math::eFCM_ABSOLUTE_ERROR);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator - (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator + (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator - (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TFactor>
tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator * (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose, TFactor factor);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy, typename TFactor>
tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator * (TFactor factor, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const bool operator == (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const bool operator != (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const bool operator < (const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/localization/pose/tPoseBase.hpp"

#endif
