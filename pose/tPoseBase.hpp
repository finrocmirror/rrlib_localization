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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tPoseBase constructors
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tPoseBase()
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TPositionElement, typename TOrientationElement>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tPoseBase(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement> &orientation) :
  position(position),
  orientation(orientation)
{}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TMatrixElement>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tPoseBase(const math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix, double max_error)
{
  reinterpret_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> *>(this)->Set(matrix, max_error);
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TOtherElement>
tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::tPoseBase(const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit> &other) :
  position(other.Position()),
  orientation(other.Orientation())
{}

//----------------------------------------------------------------------
// tPoseBase SetPosition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TPositionElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::SetPosition(const tPosition<TPositionElement> &position)
{
  this->position = position;
}

//----------------------------------------------------------------------
// tPoseBase SetOrientation
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TOrientationElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::SetOrientation(const tOrientation<TOrientationElement> &orientation)
{
  this->orientation = orientation;
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TMatrixElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::SetOrientation(const math::tMatrix<Tdimension, Tdimension, TMatrixElement> &matrix, double max_error)
{
  this->orientation.Set(matrix, max_error);
}

//----------------------------------------------------------------------
// tPoseBase Set
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TPositionElement, typename TOrientationElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Set(const tPosition<TPositionElement> &position, const tOrientation<TOrientationElement> &orientation)
{
  this->SetPosition(position);
  this->SetOrientation(orientation);
}

//----------------------------------------------------------------------
// tPoseBase Reset
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Reset()
{
  this->position.Reset();
  this->orientation.Reset();
}

//----------------------------------------------------------------------
// tPoseBase Addition assignment
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TOtherElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::operator += (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit> &other)
{
  this->position += other.Position();
  this->orientation += other.Orientation();
  return reinterpret_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &>(*this);
}

//----------------------------------------------------------------------
// tPoseBase Subtraction assignment
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TOtherElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::operator -= (const tPose<Tdimension, TOtherElement, TPositionSIUnit, TOrientationSIUnit> &other)
{
  this->position -= other.Position();
  this->orientation -= other.Orientation();
  return reinterpret_cast<tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &>(*this);
}

//----------------------------------------------------------------------
// tPoseBase GetHomogeneousTransformationMatrix
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::GetHomogeneousTransformationMatrix() const
{
  math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > matrix = this->Orientation.GetHomogeneousTransformationMatrix();
  for (size_t i = 0; i < Tdimension; ++i)
  {
    matrix[i][Tdimension] = this->Position()[i];
  }
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TMatrixElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::GetHomogeneousTransformationMatrix(math::tMatrix < Tdimension + 1, Tdimension + 1, TMatrixElement > &matrix) const
{
  matrix = this->GetHomogeneousTransformationMatrix();
}

//----------------------------------------------------------------------
// tPoseBase GetPoseInParentFrame
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TReferenceElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::GetPoseInParentFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit> &reference) const
{
  return tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>(reference.Position() + this->Position().Rotated(reference.Yaw().Value()), reference.Yaw() + this->Yaw());
}

//----------------------------------------------------------------------
// tPoseBase GetPoseInLocalFrame
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TReferenceElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::GetPoseInLocalFrame(const tPose<Tdimension, TReferenceElement, TPositionSIUnit, TOrientationSIUnit> &reference) const
{
  return tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>((this->Position() - reference.Position()).Rotated(-reference.Yaw().Value()), this->Yaw() - reference.Yaw());
}

//----------------------------------------------------------------------
// tPoseBase Translate
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TTranslationElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Translate(const math::tVector<Tdimension, TTranslationElement> &translation)
{
  this->position += tPosition<>(translation);
}

//----------------------------------------------------------------------
// tPoseBase Translated
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TTranslationElement>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Translated(const math::tVector<Tdimension, TTranslationElement> &translation)
{
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> temp(*this);
  temp.Translate(translation);
  return temp;
}

//----------------------------------------------------------------------
// tPoseBase Scale
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TFactor>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Scale(TFactor factor)
{
  this->position *= factor;
}

//----------------------------------------------------------------------
// tPoseBase Scaled
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TFactor>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Scaled(TFactor factor) const
{
  tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> temp(*this);
  temp.Scale(factor);
  return temp;
}

//----------------------------------------------------------------------
// tPoseBase ApplyRelativePoseTransformation
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
template <typename TTransformationElement>
void tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::ApplyRelativePoseTransformation(const tPose<Tdimension, TTransformationElement, TPositionSIUnit, TOrientationSIUnit> &relative_transformation)
{
  this->Translate(this->Orientation().GetMatrix() * relative_transformation.Position());
  this->Rotate(relative_transformation.Orientation().GetMatrix());
}

//----------------------------------------------------------------------
// tPoseBase IsZero
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
bool tPoseBase<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::IsZero(double epsilon) const
{
  return IsEqual(*this, tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Zero(), epsilon);
}

//----------------------------------------------------------------------
// Unary minus
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> operator - (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  return tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit>::Zero() - pose;
}

//----------------------------------------------------------------------
// Addition
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit > operator + (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit> &right)
{
  tPose < Tdimension, decltype(TLeftElement() + TRightElement()), TPositionSIUnit, TOrientationSIUnit > temp(left);
  temp += right;
  return temp;
}

//----------------------------------------------------------------------
// Subtraction
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TLeftElement, typename TRightElement, typename TPositionSIUnit, typename TOrientationSIUnit>
tPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit > operator - (const tPose<Tdimension, TLeftElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TRightElement, TPositionSIUnit, TOrientationSIUnit> &right)
{
  tPose < Tdimension, decltype(TLeftElement() - TRightElement()), TPositionSIUnit, TOrientationSIUnit > temp(left);
  temp -= right;
  return temp;
}

//----------------------------------------------------------------------
// Multiplication
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TFactor>
tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit> operator * (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose, TFactor factor)
{
  return tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit>(pose.Position() * factor, pose.Orientation());
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TFactor>
tPose<Tdimension, decltype(TElement() * TFactor()), TPositionSIUnit, TOrientationSIUnit> operator * (TFactor factor, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  return pose * factor;
}

//----------------------------------------------------------------------
// Comparison
//----------------------------------------------------------------------
template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
bool IsEqual(const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &right, float max_error, math::tFloatComparisonMethod method)
{
  return IsEqual(left.Position(), right.Position(), max_error, method) && IsEqual(left.Orientation(), right.Orientation(), max_error, method);
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const bool operator == (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &right)
{
  return left.Position() == right.Position() && left.Orientation() == right.Orientation();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const bool operator != (const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &right)
{
  return !(left == right);
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
const bool operator < (const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &left, const tPose<2, TElement, TPositionSIUnit, TOrientationSIUnit> &right)
{
  return left.Position() < right.Position || (left.Position() == right.Position() && left.Orientation() < right.Orientation());
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
{
  std::stringstream s;
  s << pose;
  return stream << s.str();
}

template <unsigned int Tdimension, typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tPose<Tdimension, TElement, TPositionSIUnit, TOrientationSIUnit> &pose)
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
