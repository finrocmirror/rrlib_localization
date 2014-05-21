//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    rrlib/localization/tPoseBase.hpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-21
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear, const rrlib::math::tVector<Tdimension, TAngular> &angular) : linear(linear), angular(angular)
{}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const TLinear x, const TLinear y, const TAngular yaw)
{
  X() = x;
  Y() = y;
  Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const TLinear x, const TLinear y, const TLinear z,
    const TAngular roll, const TAngular pitch, const TAngular yaw)
{
  X() = x;
  Y() = y;
  Z() = z;
  Roll() = roll;
  Pitch() = pitch;
  Yaw() = yaw;
}



/* constructors added for compatibility with old rrlib::math::tPose */
template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear)
{
  this->LinearVector() = linear;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear, TAngular roll, TAngular pitch, TAngular yaw)
{
  this->LinearVector() = linear;
  this->Roll() = roll;
  this->Pitch() = pitch;
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear, TAngular yaw)
{
  this->LinearVector() = linear;
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const tPoseBase<3, TLinear, TAngular> &pose_3d) : tPoseBase(pose_3d.X(), pose_3d.Y(), pose_3d.Yaw()) {}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const tPoseBase<2, TLinear, TAngular> &pose_2d) : tPoseBase(pose_2d.X(), pose_2d.Y(), TLinear(), TAngular(), TAngular(), pose_2d.Yaw()) {}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const rrlib::math::tMat4x4d &matrix, bool use_second_solution, double max_error)
{
  this->Set(matrix, use_second_solution, max_error);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular>::tPoseBase(const rrlib::math::tMat3x3d &matrix, double max_error)
{
  this->Set(matrix, max_error);
}

/* methods added for compatibility with old rrlib::math::tPose */
template <size_t Tdimension, typename TLinear, typename TAngular>
const tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Zero()
{
  static tPoseBase pose;
  return pose;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
void tPoseBase<Tdimension, TLinear, TAngular>::SetPosition(const rrlib::math::tVector<Tdimension, TLinear> &position)
{
  this->LinearVector() = position;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::SetPosition(const TLinear &x, const TLinear &y)
{
  this->X() = x;
  this->Y() = y;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::SetPosition(const TLinear &x, const TLinear &y, const TLinear &z)
{
  this->X() = x;
  this->Y() = y;
  this->Z() = z;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::SetOrientation(const TAngular &yaw)
{
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::SetOrientation(const TAngular &roll, const TAngular &pitch, const TAngular &yaw)
{
  this->Roll() = roll;
  this->Pitch() = pitch;
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::SetOrientation(const rrlib::math::tMat2x2d &matrix, double max_error)
{
  assert(rrlib::math::IsEqual(matrix.Determinant(), 1.0, max_error));
  this->Yaw() = std::atan2(matrix[1][0], matrix[0][0]);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::SetOrientation(const rrlib::math::tMat3x3d &matrix, bool use_second_solution, double max_error)
{
  matrix.ExtractRollPitchYaw(this->Roll(), this->Pitch(), this->Yaw(), use_second_solution, max_error);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const rrlib::math::tVector<Tdimension, TLinear> &position, const TAngular &yaw)
{
  this->LinearVector() = position;
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const rrlib::math::tVector<Tdimension, TLinear> &position, const TAngular &roll, const TAngular &pitch, const TAngular &yaw)
{
  this->LinearVector() = position;
  this->Roll() = roll;
  this->Pitch() = pitch;
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const rrlib::math::tVector<Tdimension, TLinear> &position)
{
  this->LinearVector() = position;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const TLinear &x, const TLinear &y, const TAngular &yaw)
{
  this->LinearVector().Set(x, y);
  this->Yaw() = yaw;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const TLinear &x, const TLinear &y, const TLinear &z, const TAngular &roll, const TAngular &pitch, const TAngular &yaw)
{
  this->LinearVector().Set(x, y, z);
  this->SetOrientation(roll, pitch, yaw);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const TLinear &x, const TLinear &y, const TLinear &z)
{
  this->LinearVector().Set(x, y, z);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const rrlib::math::tMat3x3d &matrix, double max_error)
{
  assert(rrlib::math::IsEqual(matrix[2][0], 0.0) && rrlib::math::IsEqual(matrix[2][1], 0.0) && rrlib::math::IsEqual(matrix[2][2], 1.0));
  this->LinearVector().Set(matrix[0][2], matrix[1][2]);
  this->SetOrientation(rrlib::math::tMat2x2d(matrix[0][0], matrix[0][1], matrix[1][0], matrix[1][1]), max_error);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::Set(const rrlib::math::tMat4x4d &matrix, bool use_second_solution, double max_error)
{
  assert(rrlib::math::IsEqual(matrix[3][0], 0.0, max_error) && rrlib::math::IsEqual(matrix[3][1], 0.0, max_error) && rrlib::math::IsEqual(matrix[3][2], 0.0, max_error) && rrlib::math::IsEqual(matrix[3][3], 1.0, max_error));
  this->LinearVector().Set(matrix[0][3], matrix[1][3], matrix[2][3]);
  this->SetOrientation(rrlib::math::tMat3x3d(matrix[0][0], matrix[0][1], matrix[0][2],
                       matrix[1][0], matrix[1][1], matrix[1][2],
                       matrix[2][0], matrix[2][1], matrix[2][2]), use_second_solution, max_error);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
void tPoseBase<Tdimension, TLinear, TAngular>::Reset()
{
  this->LinearVector() = rrlib::math::tVector<Tdimension, TLinear>::Zero();
  this->AngularVector() = rrlib::math::tVector<Tdimension, TAngular>::Zero();
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::operator += (const tPoseBase<Tdimension, TLinear, TAngular> &other)
{
  this->LinearVector() += other.LinearVector();
  this->AngularVector() += other.AngularVector();
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::operator -= (const tPoseBase<Tdimension, TLinear, TAngular> &other)
{
  this->LinearVector() -= other.LinearVector();
  this->AngularVector() -= other.AngularVector();
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
const rrlib::math::tMat2x2d tPoseBase<Tdimension, TLinear, TAngular>::GetRotationMatrix() const
{
  return rrlib::math::Get2DRotationMatrix<double>(this->Yaw());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
rrlib::math::tMat3x3d tPoseBase<Tdimension, TLinear, TAngular>::GetRotationMatrix() const
{
  return rrlib::math::Get3DRotationMatrixFromRollPitchYaw<double>(this->Roll(), this->Pitch(), this->Yaw());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::GetRotationMatrix(rrlib::math::tMat2x2d &matrix) const
{
  matrix = this->GetRotationMatrix();
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::GetRotationMatrix(rrlib::math::tMat3x3d &matrix) const
{
  matrix = this->GetRotationMatrix();
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
const rrlib::math::tMat3x3d tPoseBase<Tdimension, TLinear, TAngular>::GetTransformationMatrix() const
{
  rrlib::math::tMat2x2d rotation = this->GetRotationMatrix();
  return rrlib::math::tMat3x3d(rotation[0][0], rotation[0][1], static_cast<double>(this->X()),
                               rotation[1][0], rotation[1][1], static_cast<double>(this->Y()),
                               0, 0, 1);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
const rrlib::math::tMat4x4d tPoseBase<Tdimension, TLinear, TAngular>::GetTransformationMatrix() const
{
  rrlib::math::tMat3x3d rotation = this->GetRotationMatrix();
  return rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], static_cast<double>(this->X()),
                               rotation[1][0], rotation[1][1], rotation[1][2], static_cast<double>(this->Y()),
                               rotation[2][0], rotation[2][1], rotation[2][2], static_cast<double>(this->Z()),
                               0, 0, 0, 1);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::GetTransformationMatrix(rrlib::math::tMat3x3d &matrix) const
{
  matrix = this->GetTransformationMatrix();
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::GetTransformationMatrix(rrlib::math::tMat4x4d &matrix) const
{
  matrix = this->GetTransformationMatrix();
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::GetPoseInParentFrame(const tPoseBase<Tdimension, TLinear, TAngular> &reference) const
{
  return tPoseBase<Tdimension, TLinear, TAngular>(reference.GetTransformationMatrix() * this->GetTransformationMatrix());

}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::GetPoseInLocalFrame(const tPoseBase<Tdimension, TLinear, TAngular> &reference) const
{
  return tPoseBase<Tdimension, TLinear, TAngular>(reference.GetTransformationMatrix().Inverse() * this->GetTransformationMatrix());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template<typename TIterator, size_t U, class>
void tPoseBase<Tdimension, TLinear, TAngular>::TransformCoordinateSystem(TIterator points_begin, TIterator points_end, bool in_local_frame) const
{
  // fixed transformation matrix
  rrlib::math::tMat4x4d reference_transformation_matrix = this->GetTransformationMatrix();
  if (in_local_frame)
  {
    reference_transformation_matrix.Invert();
  }
  // transformation matrix for points of the cloud, rotation matrix is always zero, translation vector is different for each point
  rrlib::math::tMat4x4d point_transformation_matrix = tPoseBase<Tdimension, TLinear, TAngular>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).GetTransformationMatrix();
  //
  rrlib::math::tMat4x4d result_matrix;

  for (auto it = points_begin; it < points_end; ++it)
  {
    // set translation component of the transformation matrix
    point_transformation_matrix[0][3] = static_cast<double>((*it).X());
    point_transformation_matrix[1][3] = static_cast<double>((*it).Y());
    point_transformation_matrix[2][3] = static_cast<double>((*it).Z());

    result_matrix = reference_transformation_matrix * point_transformation_matrix;

    // copy position from result matrix
    (*it).Set(result_matrix[0][3], result_matrix[1][3], result_matrix[2][3]);
  }
}



template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Translate(const rrlib::math::tVector<Tdimension, TLinear> &translation)
{
  this->LinearVector() += translation;
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Translated(const rrlib::math::tVector<Tdimension, TLinear> &translation) const
{
  return tPoseBase(this->LinearVector() + translation, this->AngularVector());
}


template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Rotate(TAngular yaw)
{
  this->Yaw() += yaw;
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Rotate(TAngular roll, TAngular pitch, TAngular yaw)
{
  this->Roll() += roll;
  this->Pitch() += pitch;
  this->Yaw() += yaw;
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Rotate(const rrlib::math::tMat2x2d &matrix)
{
  this->SetOrientation(matrix * this->GetRotationMatrix());
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Rotate(const rrlib::math::tMat3x3d &matrix)
{
  assert(matrix.Determinant() == 1);
  this->SetOrientation(matrix * this->GetRotationMatrix());
  return *this;
}


template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Rotated(TAngular yaw) const
{
  return tPoseBase(this->LinearVector(), this->Yaw() + yaw);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Rotated(TAngular roll, TAngular pitch, TAngular yaw) const
{
  return tPoseBase(this->LinearVector(), this->Roll() + roll, this->Pitch() + pitch, this->Yaw() + yaw);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Rotated(const rrlib::math::tMat2x2d &matrix) const
{
  tPoseBase<Tdimension, TLinear, TAngular> temp(*this);
  temp.Rotate(matrix);
  return temp;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Rotated(const rrlib::math::tMat3x3d &matrix) const
{
  tPoseBase<Tdimension, TLinear, TAngular> temp(*this);
  temp.Rotate(matrix);
  return temp;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::Scale(double factor)
{
  this->LinearVector() *= factor;
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Scaled(double factor) const
{
  return tPoseBase<Tdimension, TLinear, TAngular>(this->LinearVector() * factor, this->AngularVector());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> &tPoseBase<Tdimension, TLinear, TAngular>::ApplyRelativePoseTransformation(const tPoseBase<Tdimension, TLinear, TAngular> &relative_transformation)
{
  this->Set(this->GetTransformationMatrix() * relative_transformation.GetTransformationMatrix());
  return *this;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
void tPoseBase<Tdimension, TLinear, TAngular>::ApplyPose(const tPoseBase<Tdimension, TLinear, TAngular> &relative_transformation)
{
  this->ApplyRelativePoseTransformation(relative_transformation);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
bool tPoseBase<Tdimension, TLinear, TAngular>::IsZero() const
{
  return IsEqual(*this, tPoseBase::Zero());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<2, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::GetPose2D() const
{
  return tPoseBase<2, TLinear, TAngular>(this->X(), this->Y(), this->Yaw());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::Compound(const tPoseBase<Tdimension, TLinear, TAngular> &base, const tPoseBase<Tdimension, TLinear, TAngular> &diff)
{
  return diff.GetPoseInParentFrame(base);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
template <size_t U, class>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::InverseCompound(const tPoseBase<Tdimension, TLinear, TAngular> &target, const tPoseBase<Tdimension, TLinear, TAngular> &base)
{
  return target.GetPoseInLocalFrame(base);
}



// operators and comparisons


template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> tPoseBase<Tdimension, TLinear, TAngular>::operator - (const tPoseBase<Tdimension, TLinear, TAngular> &pose)
{
  return tPoseBase(-pose.LinearVector(), -pose.AngularVector());

}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> operator + (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right)
{
  return tPoseBase<Tdimension, TLinear, TAngular>(left.LinearVector() + right.LinearVector(), left.AngularVector() + right.AngularVector());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> operator - (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right)
{
  return tPoseBase<Tdimension, TLinear, TAngular>(left.LinearVector() - right.LinearVector(), left.AngularVector() - right.AngularVector());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
bool IsEqual(const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right, float max_error, rrlib::math::tFloatComparisonMethod method)
{
  return rrlib::math::IsEqual(left.LinearVector(), right.LinearVector(), max_error, method)
         && rrlib::math::IsEqual(left.AngularVector(), right.AngularVector(), max_error, method);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
bool operator == (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right)
{
  return left.LinearVector() == right.LinearVector() && left.AngularVector() == right.AngularVector();
}

template <size_t Tdimension, typename TLinear, typename TAngular>
bool operator != (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right)
{
  return !(left == right);
}

template <size_t Tdimension, typename TLinear, typename TAngular>
bool operator < (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right)
{
  return left.LinearVector() < right.LinearVector() ||
         (left.LinearVector() == right.LinearVector() && left.AngularVector() < right.AngularVector());
}

template <size_t Tdimension, typename TLinear, typename TAngular>
std::ostream &operator << (std::ostream &stream, const tPoseBase<Tdimension, TLinear, TAngular> &o)
{
  stream << "(";
  stream << o.LinearVector()[0];
  for (size_t i = 1; i < Tdimension; ++i)
  {
    stream << ", " << o.LinearVector()[i];
  }
  for (size_t i = 0; i < Tdimension; ++i)
  {
    stream << ", " << o.AngularVector()[i];
  }

  stream << ")";

  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tPoseBase<Tdimension, TLinear, TAngular>& o)
{
  stream << o.LinearVector() << o.AngularVector();
  return stream;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tPoseBase<Tdimension, TLinear, TAngular>& o)
{
  stream >> o.LinearVector() >> o.AngularVector();
  return stream;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tStringOutputStream& operator << (rrlib::serialization::tStringOutputStream& stream, const tPoseBase<Tdimension, TLinear, TAngular>& o)
{
  std::stringstream s;
  s << o;
  stream << s.str();
  return stream;
}

template <size_t Tdimension, typename TLinear, typename TAngular>
inline rrlib::serialization::tStringInputStream& operator >> (rrlib::serialization::tStringInputStream& stream, tPoseBase<Tdimension, TLinear, TAngular>& o)
{

  char temp(0);
  stream >> temp;
  if (temp == '(')
  {

    for (size_t i = 0; i < Tdimension; ++i)
    {
      stream >> o.LinearVector()[i] >> temp;
    }
    for (size_t i = 0; i < Tdimension; ++i)
    {
      stream >> o.AngularVector()[i] >> temp; // note: the last one deserializes the closing ")" into temp
    }

  }

  return stream;

}
#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
