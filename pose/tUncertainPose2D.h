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
/*!\file    rrlib/localization/pose/tUncertainPose2D.h
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-21
 *
 * \brief   Contains tUncertainPose2D
 *
 * \b tUncertainPose2D
 *
 * 2D pose with additional uncertainty (covariance matrix)
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__uncertain_pose__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tUncertainPose.h" instead.
#endif

#ifndef __rrlib__localization__tUncertainPose2D_h__
#define __rrlib__localization__tUncertainPose2D_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tMatrix.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/pose/tUncertainPoseBase.h"

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
//! SHORT_DESCRIPTION
/*!
 * 2D pose with additional uncertainty (covariance matrix)
 */
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
class tUncertainPose<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> : public pose::tUncertainPoseBase<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>
{
  typedef pose::tUncertainPoseBase<2, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPoseBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPositionElement = TElement>
  using tPosition = typename tPoseBase::template tPosition<TPositionElement>;

  template <typename TPositionElement = TElement>
  using tPositionComponent = typename tPoseBase::template tPositionComponent<TPositionElement>;

  template <typename TOrientationElement = TElement, typename TOrientationAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientation = typename tPoseBase::template tOrientation<TOrientationElement, TOrientationAutoWrapPolicy>;

  template <typename TAngleElement = TElement, typename TAngleUnitPolicy = math::angle::Radian, typename TAngleAutoWrapPolicy = TAutoWrapPolicy>
  using tOrientationComponent = typename tPoseBase::template tOrientationComponent<TAngleElement, TAngleUnitPolicy, TAngleAutoWrapPolicy>;

  template <typename TCovarianceElement = TElement>
  using tCovarianceMatrix = typename tPoseBase::template tCovarianceMatrix<TCovarianceElement>;

  using tPoseBase::tPoseBase;

  tUncertainPose();

  template <typename TX, typename TY, typename TYaw, typename TCovarianceElement>
  tUncertainPose(TX x, TY y, TYaw yaw, const tCovarianceMatrix<TCovarianceElement> &covariance);

  inline const TElement &CovarianceXX() const
  {
    return this->Covariance()[0][0];
  }
  inline TElement &CovarianceXX()
  {
    return this->Covariance()[0][0];
  }

  inline const TElement &CovarianceXY() const
  {
    return this->Covariance()[0][1];
  }
  inline TElement &CovarianceXY()
  {
    return this->Covariance()[0][1];
  }

  inline const TElement &CovarianceXYaw() const
  {
    return this->Covariance()[0][2];
  }
  inline TElement &CovarianceXYaw()
  {
    return this->Covariance()[0][2];
  }

  inline const TElement &CovarianceYY() const
  {
    return this->Covariance()[1][1];
  }
  inline TElement &CovarianceYY()
  {
    return this->Covariance()[1][1];
  }

  inline const TElement &CovarianceYYaw() const
  {
    return this->Covariance()[1][2];
  }
  inline TElement &CovarianceYYaw()
  {
    return this->Covariance()[1][2];
  }

  inline const TElement &CovarianceYawYaw() const
  {
    return this->Covariance()[2][2];
  }
  inline TElement &CovarianceYawYaw()
  {
    return this->Covariance()[2][2];
  }

  inline const TElement &VarianceX() const
  {
    return this->CovarianceXX();
  }
  inline TElement &VarianceX()
  {
    return this->CovarianceXX();
  }

  inline const TElement &VarianceY() const
  {
    return this->CovarianceYY();
  }
  inline TElement &VarianceY()
  {
    return this->CovarianceYY();
  }

  inline const TElement &VarianceYaw() const
  {
    return this->CovarianceYawYaw();
  }
  inline TElement &VarianceYaw()
  {
    return this->CovarianceYawYaw();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/localization/pose/tUncertainPose2D.hpp"

#endif
