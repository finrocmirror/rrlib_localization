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
/*!\file    rrlib/localization/tPoseBase.h
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-21
 *
 * \brief   Contains tPoseBase
 *
 * \b tPoseBase
 *
 * Base class for classes related to poses, e.g. the pose itself, its first derivative (the twist) etc.
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__tPoseBase_h__
#define __rrlib__localization__tPoseBase_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
// Class declaration
//----------------------------------------------------------------------
//! Base class for classes related to poses.
/*!
 * Base class for classes related to poses, e.g. the pose itself, its first derivative (the twist) etc.
 */
template <size_t Tdimension, typename TLinear, typename TAngular>
class tPoseBase
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /** the dimension of the free space, this is usually 2 or 3 */
  static const size_t dimension = Tdimension;

  /* this must be ensured for serialization - and otherwise the class does not make much sense */
  static_assert(Tdimension >= 1, "Dimension must be at least 1");

  typedef TLinear tLinear;
  typedef TAngular tAngular;

  //bool operator==(const tPoseBase &other) const;
  tPoseBase &operator= (const tPoseBase &o) = default;

  /* some "standard" constructors */

  tPoseBase() = default;
  tPoseBase(const tPoseBase &o) = default;

  /**
   * Construct a new pose with linear and angular vector
   * @param linear the linear vector
   * @param angular the angular vector
   */
  tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear, const rrlib::math::tVector<Tdimension, TAngular> &angular);

  // TODO: there are unfortunately problems with the default arguments and conflicts if inheriting these c'tors, see also http://stackoverflow.com/questions/9979194/what-is-constructor-inheritance

  /**
   * Construct a 2D pose completely out of scalars
   * @param x the x scalar
   * @param y the y scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase(const TLinear x, const TLinear y, const TAngular yaw = 0);
  /**
   * Construct a 3D pose completely out of scalars
   * @param x the x scalar
   * @param y the y scalar
   * @param z the z scalar
   * @param roll the roll scalar
   * @param pitch the pitch scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  tPoseBase(const TLinear x, const TLinear y, const TLinear z,
            const TAngular roll = 0, const TAngular pitch = 0, const TAngular yaw = 0);



  /* constructors added for compatibility with old rrlib::math::tPose */
  /**
   * Construct a new pose with just the linear vector
   * @param linear the linear vector
   */
  explicit tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear);

  /**
   * Construct a new pose with linear vector and angular scalars (for the 3D case)
   * @param linear the linear vector
   * @param roll the roll scalar
   * @param pitch the pitch scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  explicit tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear, TAngular roll, TAngular pitch, TAngular yaw);

  /**
   * Construct a new pose with linear vector and angular scalars (for the 2D case)
   * @param linear the linear vector
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  explicit tPoseBase(const rrlib::math::tVector<Tdimension, TLinear> &linear, TAngular yaw);

  /**
   * Construct a 2D pose from a 3D pose, this direction is implicit
   * @param pose_3d the 3D pose
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase(const tPoseBase<3, TLinear, TAngular> &pose_3d);

  /**
   * Construct a 3D pose from a 2D pose, this direction is explicit
   * @param pose_2d the 2D pose
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  explicit tPoseBase(const tPoseBase<2, TLinear, TAngular> &pose_2d);

  /**
   * Construct a 3D pose from a 4x4 matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  explicit tPoseBase(const rrlib::math::tMat4x4d &matrix, bool use_second_solution = false, double max_error = 1E-6);

  /**
   * Construct a 2D pose from a 3x3 matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  explicit tPoseBase(const rrlib::math::tMat3x3d &matrix, double max_error = 1E-6);
  /* done */



  template<size_t U = Tdimension, class = typename std::enable_if<U >= 1>::type >
  inline TLinear &X()
  {
    return linear[0];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  inline TLinear &Y()
  {
    return linear[1];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  inline TLinear &Z()
  {
    return linear[2];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  inline TAngular &Roll()
  {
    return angular[0];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  inline TAngular &Pitch()
  {
    return angular[1];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  inline TAngular &Yaw()
  {
    if (Tdimension == 2)
      return angular[0];
    else
      return angular[2];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 1>::type >
  inline const TLinear &X() const
  {
    return linear[0];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  inline const TLinear &Y() const
  {
    return linear[1];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  inline const TLinear &Z() const
  {
    return linear[2];
  }

  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  inline const TAngular &Roll() const
  {
    return angular[0];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 3>::type >
  inline const TAngular &Pitch() const
  {
    return angular[1];
  }
  template<size_t U = Tdimension, class = typename std::enable_if<U >= 2>::type >
  inline const TAngular &Yaw() const
  {
    if (Tdimension == 2)
      return angular[0];
    else
      return angular[2];
  }


  inline rrlib::math::tVector<Tdimension, TLinear> &LinearVector()
  {
    return linear;
  }

  inline rrlib::math::tVector<Tdimension, TAngular> &AngularVector()
  {
    return angular;
  }

  inline const rrlib::math::tVector<Tdimension, TLinear> &LinearVector() const
  {
    return linear;
  }

  inline const rrlib::math::tVector<Tdimension, TAngular> &AngularVector() const
  {
    return angular;
  }


  /* methods added for compatibility with old rrlib::math::tPose */

  static const tPoseBase &Zero();

  inline const rrlib::math::tVector<Tdimension, TLinear> &Position() const
  {
    return this->LinearVector();
  }

  inline rrlib::math::tVector<Tdimension, TLinear> &Position()
  {
    return this->LinearVector();
  }

  /**
   * Set position using a position vector
   * @param position the position vector
   */
  void SetPosition(const rrlib::math::tVector<Tdimension, TLinear> &position);

  /**
   * Set position for a 2D pose using scalars
   * @param x the x scalar
   * @param y the y scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void SetPosition(const TLinear &x, const TLinear &y);

  /**
   * Set position for a 3D pose using scalars
   * @param x the x scalar
   * @param y the y scalar
   * @param z the z scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void SetPosition(const TLinear &x, const TLinear &y, const TLinear &z);

  /**
   * Set orientation for a 2D pose using a scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void SetOrientation(const TAngular &yaw);

  /**
   * Set orientation for a 3D pose using scalars
   * @param roll the roll scalar
   * @param pitch the pitch scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void SetOrientation(const TAngular &roll, const TAngular &pitch, const TAngular &yaw);

  /**
   * Set orientation for a 2D pose using a 2x2 matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void SetOrientation(const rrlib::math::tMat2x2d &matrix, double max_error = 1E-6);

  /**
   * Set orientation for a 3D pose using a 3x3 matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void SetOrientation(const rrlib::math::tMat3x3d &matrix, bool use_second_solution = false, double max_error = 1E-6);

  /**
   * Set values of a 2D pose using a position vector and a scalar for the angular value
   * @param position the position vector
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void Set(const rrlib::math::tVector<Tdimension, TLinear> &position, const TAngular &yaw);

  /**
   * Set values of a 3D pose using a position vector and scalars for the angular values
   * @param position the position vector
   * @param roll the roll scalar
   * @param pitch the pitch scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void Set(const rrlib::math::tVector<Tdimension, TLinear> &position, const TAngular &roll, const TAngular &pitch, const TAngular &yaw);
  /**
   * Set the position of a pose using a position vector
   * @param position the position vector
   */
  void Set(const rrlib::math::tVector<Tdimension, TLinear> &position);


  /**
   * Set values of a 2D pose completely from scalars
   * @param x the x scalar
   * @param y the y scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void Set(const TLinear &x, const TLinear &y, const TAngular &yaw);

  /**
   * Set values of a 3D pose completely from scalars
   * @param x the x scalar
   * @param y the y scalar
   * @param z the z scalar
   * @param roll the roll scalar
   * @param pitch the pitch scalar
   * @param yaw the yaw scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void Set(const TLinear &x, const TLinear &y, const TLinear &z, const TAngular &roll, const TAngular &pitch, const TAngular &yaw);

  /**
   * Set the position of a 3D pose completely from scalars
   * @param x the x scalar
   * @param y the y scalar
   * @param z the z scalar
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void Set(const TLinear &x, const TLinear &y, const TLinear &z);
  /**
   * Set values of a 2D pose using a matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void Set(const rrlib::math::tMat3x3d &matrix, double max_error = 1E-6);

  /**
   * Set values of a 3D pose using a matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void Set(const rrlib::math::tMat4x4d &matrix, bool use_second_solution = false, double max_error = 1E-6);




  void Reset();

  tPoseBase &operator += (const tPoseBase &other);

  tPoseBase &operator -= (const tPoseBase &other);

  /**
   * Get the rotation matrix of a 2D pose
   * @return the rotation matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  const rrlib::math::tMat2x2d GetRotationMatrix() const;

  /**
   * Get the rotation matrix of a 3D pose
   * @return the rotation matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  rrlib::math::tMat3x3d GetRotationMatrix() const;

  /**
   * Get the rotation matrix of a 2D pose
   * @param matrix a reference to the matrix where to store the result
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void GetRotationMatrix(rrlib::math::tMat2x2d &matrix) const;
  /**
   * Get the rotation matrix of a 3D pose
   * @param matrix a reference to the matrix where to store the result
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void GetRotationMatrix(rrlib::math::tMat3x3d &matrix) const;

  /**
   * Get the transformation matrix of a 2D pose
   * @return the transformation matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  const rrlib::math::tMat3x3d GetTransformationMatrix() const;

  /**
   * Get the transformation matrix of a 3D pose
   * @return the transformation matrix
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  const rrlib::math::tMat4x4d GetTransformationMatrix() const;

  /**
   * Get the transformation matrix of a 2D pose
   * @param matrix a reference to the matrix where to store the result
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  void GetTransformationMatrix(rrlib::math::tMat3x3d &matrix) const;

  /**
   * Get the transformation matrix of a 3D pose
   * @param matrix a reference to the matrix where to store the result
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void GetTransformationMatrix(rrlib::math::tMat4x4d &matrix) const;


  tPoseBase GetPoseInParentFrame(const tPoseBase &reference) const;

  tPoseBase GetPoseInLocalFrame(const tPoseBase &reference) const;

  /*! Transform given 3D points to a coordinate system defined by a reference pose
   *
   * \param points_begin    Begin iterator of the points to transform
   * \param points_begin    End iterator of the points to transform
   * \param in_local_frame  Assume that "reference" describes the pose of a coordinate system B in the coordinate system A:
   *                        Choose true if you want to convert points from system A to B, false if you want to transform points from B to A
   */
  template<typename TIterator, size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  void TransformCoordinateSystem(TIterator points_begin, TIterator points_end, bool in_local_frame) const;

  tPoseBase &Translate(const rrlib::math::tVector<Tdimension, TLinear> &translation);

  tPoseBase Translated(const rrlib::math::tVector<Tdimension, TLinear> &translation) const;

  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase &Rotate(TAngular yaw);

  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  tPoseBase &Rotate(TAngular roll, TAngular pitch, TAngular yaw);

  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase &Rotate(const rrlib::math::tMat2x2d &matrix);

  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  tPoseBase &Rotate(const rrlib::math::tMat3x3d &matrix);

  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase Rotated(TAngular yaw) const;

  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  tPoseBase Rotated(TAngular roll, TAngular pitch, TAngular yaw) const;

  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase Rotated(const rrlib::math::tMat2x2d &matrix) const;

  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  tPoseBase Rotated(const rrlib::math::tMat3x3d &matrix) const;

  tPoseBase &Scale(double factor);

  tPoseBase Scaled(double factor) const;

  tPoseBase &ApplyRelativePoseTransformation(const tPoseBase &relative_transformation);

  void ApplyPose(const tPoseBase &relative_transformation) __attribute__((deprecated));

  double GetEuclideanNorm() const;

  bool IsZero() const;

  /**
   * Create a 2D pose out of a 3D pose (ommit values not present in 2D pose)
   * @return the 2D pose
   */
  template<size_t U = Tdimension, class = typename std::enable_if<U == 3>::type>
  tPoseBase<2, TLinear, TAngular> GetPose2D() const;

  tPoseBase operator - (const tPoseBase &pose);

  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase Compound(const tPoseBase &base, const tPoseBase &diff) __attribute__((deprecated));

  template<size_t U = Tdimension, class = typename std::enable_if<U == 2>::type>
  tPoseBase InverseCompound(const tPoseBase &target, const tPoseBase &base) __attribute__((deprecated));


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  rrlib::math::tVector<Tdimension, TLinear> linear;
  rrlib::math::tVector<Tdimension, TAngular> angular;

};

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> operator + (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right);

template <size_t Tdimension, typename TLinear, typename TAngular>
tPoseBase<Tdimension, TLinear, TAngular> operator - (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right);

template <size_t Tdimension, typename TLinear, typename TAngular>
bool IsEqual(const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right, float max_error = 1E-6, rrlib::math::tFloatComparisonMethod method = rrlib::math::tFloatComparisonMethod::eFCM_ABSOLUTE_ERROR);

template <size_t Tdimension, typename TLinear, typename TAngular>
bool operator == (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right);

template <size_t Tdimension, typename TLinear, typename TAngular>
bool operator != (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right);

template <size_t Tdimension, typename TLinear, typename TAngular>
bool operator < (const tPoseBase<Tdimension, TLinear, TAngular> &left, const tPoseBase<Tdimension, TLinear, TAngular> &right);

template <size_t Tdimension, typename TLinear, typename TAngular>
std::ostream &operator << (std::ostream &stream, const tPoseBase<Tdimension, TLinear, TAngular> &o);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TLinear, typename TAngular>
rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tPoseBase<Tdimension, TLinear, TAngular>& o);

template <size_t Tdimension, typename TLinear, typename TAngular>
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tPoseBase<Tdimension, TLinear, TAngular>& o);

template <size_t Tdimension, typename TLinear, typename TAngular>
rrlib::serialization::tStringOutputStream& operator << (rrlib::serialization::tStringOutputStream& stream, const tPoseBase<Tdimension, TLinear, TAngular>& o);

template <size_t Tdimension, typename TLinear, typename TAngular>
rrlib::serialization::tStringInputStream& operator >> (rrlib::serialization::tStringInputStream& stream, tPoseBase<Tdimension, TLinear, TAngular>& o);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/localization/tPoseBase.hpp"

#endif
