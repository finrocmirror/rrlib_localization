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
/*!\file    rrlib/localization/pose/tPoseBase.cpp
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
namespace pose
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const unsigned int tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::cDIMENSION = Tdimension;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tPoseBase constructors
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPoseBase()
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TPositionElement, typename TOrientationElement, typename TOrientationAutoWrapPolicy>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPoseBase(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation) :
  position(position),
  orientation(orientation)
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPoseBase(const math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix, double max_error)
{
  reinterpret_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> *>(this)->Set(matrix, max_error);
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::tPoseBase(const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other) :
  position(other.Position()),
  orientation(other.Orientation())
{}

//----------------------------------------------------------------------
// tPoseBase SetPosition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TPositionElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetPosition(const tPosition<TPositionElement> &position)
{
  this->position = position;
}

//----------------------------------------------------------------------
// tPoseBase SetOrientation
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOrientationElement, typename TOrientationAutoWrapPolicy>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetOrientation(const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation)
{
  this->orientation = orientation;
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::SetOrientation(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix, double max_error)
{
  this->orientation.Set(matrix, max_error);
}

//----------------------------------------------------------------------
// tPoseBase Set
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TPositionElement, typename TOrientationElement, typename TOrientationAutoWrapPolicy>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Set(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement, TOrientationAutoWrapPolicy> &orientation)
{
  this->SetPosition(position);
  this->SetOrientation(orientation);
}

//----------------------------------------------------------------------
// tPoseBase Reset
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Reset()
{
  this->position = tPosition<>();
  this->orientation.Reset();
}

//----------------------------------------------------------------------
// tPoseBase Addition assignment
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::operator += (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->position += other.Position();
  this->orientation += other.Orientation();
  return reinterpret_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(*this);
}

//----------------------------------------------------------------------
// tPoseBase Subtraction assignment
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TOtherElement, typename TOtherAutoWrapPolicy>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::operator -= (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit, TOtherAutoWrapPolicy> &other)
{
  this->position -= other.Position();
  this->orientation -= other.Orientation();
  return reinterpret_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &>(*this);
}

//----------------------------------------------------------------------
// tPoseBase GetHomogeneousTransformationMatrix
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::GetHomogeneousTransformationMatrix() const
{
  math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > matrix = this->Orientation.GetHomogeneousTransformationMatrix();
  for (size_t i = 0; i < Tdimension; ++i)
  {
    matrix[i][Tdimension] = this->Position()[i];
  }
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TMatrixElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::GetHomogeneousTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const
{
  matrix = this->GetHomogeneousTransformationMatrix();
}

//----------------------------------------------------------------------
// tPoseBase GetPoseInParentFrame
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TReferenceElement, typename TReferenceAutoWrapPolicy>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::GetPoseInParentFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit, TReferenceAutoWrapPolicy> &reference) const
{
  return tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>(reference.Position() + this->Position().Rotated(reference.Yaw().Value()), reference.Yaw() + this->Yaw());
}

//----------------------------------------------------------------------
// tPoseBase GetPoseInLocalFrame
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TReferenceElement, typename TReferenceAutoWrapPolicy>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::GetPoseInLocalFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit, TReferenceAutoWrapPolicy> &reference) const
{
  return tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>((this->Position() - reference.Position()).Rotated(-reference.Yaw().Value()), this->Yaw() - reference.Yaw());
}

//----------------------------------------------------------------------
// tPoseBase Translate
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TTranslationElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Translate(const math::tVector<Tdimension, TTranslationElement> &translation)
{
  this->position += tPosition<>(translation);
}

//----------------------------------------------------------------------
// tPoseBase Translated
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TTranslationElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Translated(const math::tVector<Tdimension, TTranslationElement> &translation)
{
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> temp(*this);
  temp.Translate(translation);
  return temp;
}

//----------------------------------------------------------------------
// tPoseBase Rotate
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TRotationElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Rotate(const math::tMatrix<Tdimension, Tdimension, TRotationElement> &rotation)
{
  this->SetOrientation(rotation * this->Orientation().GetMatrix());
}

//----------------------------------------------------------------------
// tPoseBase Rotated
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TRotationElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Rotated(const math::tMatrix<Tdimension, Tdimension, TRotationElement> &rotation)
{
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> temp(*this);
  temp.Rotate(rotation);
  return temp;
}

//----------------------------------------------------------------------
// tPoseBase Scale
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TFactor>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Scale(TFactor factor)
{
  this->position *= factor;
}

//----------------------------------------------------------------------
// tPoseBase Scaled
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TFactor>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Scaled(TFactor factor) const
{
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> temp(*this);
  temp.Scale(factor);
  return temp;
}

//----------------------------------------------------------------------
// tPoseBase ApplyRelativePoseTransformation
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
template <typename TTransformationElement, typename TTransformationAutoWrapPolicy>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::ApplyRelativePoseTransformation(const tPose<Tdimension, TTransformationElement, TPositionSIUnit, TOrientationSIUnit, TTransformationAutoWrapPolicy> &relative_transformation)
{
  this->Translate(this->Orientation().GetMatrix() * relative_transformation.Position());
  this->Rotate(relative_transformation.Orientation().GetMatrix());
}

//----------------------------------------------------------------------
// tPoseBase IsZero
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
bool tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::IsZero(double epsilon) const
{
  return IsEqual(*this, tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Zero(), epsilon);
}

//----------------------------------------------------------------------
// Unary minus
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> operator - (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  return tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>::Zero() - pose;
}

//----------------------------------------------------------------------
// Addition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator + (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right)
{
  tPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > temp(left);
  temp += right;
  return temp;
}

//----------------------------------------------------------------------
// Subtraction
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
tPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > operator - (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit, TLeftAutoWrapPolicy> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit, TRightAutoWrapPolicy> &right)
{
  tPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit, typename math::angle::AutoWrapPolicy<TLeftAutoWrapPolicy, TRightAutoWrapPolicy>::tType > temp(left);
  temp -= right;
  return temp;
}

//----------------------------------------------------------------------
// Comparison
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
bool IsEqual(const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right, float max_error, math::tFloatComparisonMethod method)
{
  return IsEqual(left.Position(), right.Position(), max_error, method) && IsEqual(left.Orientation(), right.Orientation(), max_error, method);
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const bool operator == (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right)
{
  return left.Position() == right.Position() && left.Orientation() == right.Orientation();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const bool operator != (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right)
{
  return !(left == right);
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
const bool operator < (const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &left, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &right)
{
  return left.Position() < right.Position || (left.Position() == right.Position() && left.Orientation() < right.Orientation());
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  std::stringstream s;
  s << pose;
  return stream << s.str();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> &pose)
{
  stream.GetWrappedStringStream() >> pose;
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
