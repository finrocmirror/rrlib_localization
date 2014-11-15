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
/*!\file    rrlib/localization/pose/tUncertainPose3D.h
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2014-04-21
 *
 * \brief   Contains tUncertainPose3D
 */
//----------------------------------------------------------------------
#ifndef __rrlib__localization__uncertain_pose__include_guard__
#error Invalid include directive. Try #include "rrlib/localization/tUncertainPose.h" instead.
#endif

#ifndef __rrlib__localization__tUncertainPose3D_h__
#define __rrlib__localization__tUncertainPose3D_h__

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
//! Definition of an uncertain pose in the three dimensional case.
/*! The pose is defined by a partial specialization of \ref tUncertainPose
 */
template <typename TElement, typename TPositionSIUnit, typename TOrientationSIUnit, typename TAutoWrapPolicy>
class tUncertainPose<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> : public pose::tUncertainPoseBase<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy>
{
  typedef pose::tUncertainPoseBase<3, TElement, TPositionSIUnit, TOrientationSIUnit, TAutoWrapPolicy> tPoseBase;

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

  template <typename TX, typename TY, typename TZ, typename TCovarianceElement>
  tUncertainPose(TX x, TY y, TZ z, const tCovarianceMatrix<TCovarianceElement> &covariance);

  template <typename T1, typename T2, typename T3, typename T4, typename TCovarianceElement>
  tUncertainPose(T1 arg_1, T2 arg_2, T3 arg_3, T4 arg_4, const tCovarianceMatrix<TCovarianceElement> &covariance);

  template <typename TX, typename TY, typename TZ, typename TRoll, typename TPitch, typename TYaw, typename TCovarianceElement>
  tUncertainPose(TX x, TY y, TZ z, TRoll roll, TPitch pitch, TYaw yaw, const tCovarianceMatrix<TCovarianceElement> &covariance);

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

  inline const TElement &CovarianceXZ() const
  {
    return this->Covariance()[0][2];
  }
  inline TElement &CovarianceXZ()
  {
    return this->Covariance()[0][2];
  }

  inline const TElement &CovarianceXRoll() const
  {
    return this->Covariance()[0][3];
  }
  inline TElement &CovarianceXRoll()
  {
    return this->Covariance()[0][3];
  }

  inline const TElement &CovarianceXPitch() const
  {
    return this->Covariance()[0][4];
  }
  inline TElement &CovarianceXPitch()
  {
    return this->Covariance()[0][4];
  }

  inline const TElement &CovarianceXYaw() const
  {
    return this->Covariance()[0][5];
  }
  inline TElement &CovarianceXYaw()
  {
    return this->Covariance()[0][5];
  }

  inline const TElement &CovarianceYY() const
  {
    return this->Covariance()[1][1];
  }
  inline TElement &CovarianceYY()
  {
    return this->Covariance()[1][1];
  }

  inline const TElement &CovarianceYZ() const
  {
    return this->Covariance()[1][2];
  }
  inline TElement &CovarianceYZ()
  {
    return this->Covariance()[1][2];
  }

  inline const TElement &CovarianceYRoll() const
  {
    return this->Covariance()[1][3];
  }
  inline TElement &CovarianceYRoll()
  {
    return this->Covariance()[1][3];
  }

  inline const TElement &CovarianceYPitch() const
  {
    return this->Covariance()[1][4];
  }
  inline TElement &CovarianceYPitch()
  {
    return this->Covariance()[1][4];
  }

  inline const TElement &CovarianceYYaw() const
  {
    return this->Covariance()[1][5];
  }
  inline TElement &CovarianceYYaw()
  {
    return this->Covariance()[1][5];
  }

  inline const TElement &CovarianceZZ() const
  {
    return this->Covariance()[2][2];
  }
  inline TElement &CovarianceZZ()
  {
    return this->Covariance()[2][2];
  }

  inline const TElement &CovarianceZRoll() const
  {
    return this->Covariance()[2][3];
  }
  inline TElement &CovarianceZRoll()
  {
    return this->Covariance()[2][3];
  }

  inline const TElement &CovarianceZPitch() const
  {
    return this->Covariance()[2][4];
  }
  inline TElement &CovarianceZPitch()
  {
    return this->Covariance()[2][4];
  }

  inline const TElement &CovarianceZYaw() const
  {
    return this->Covariance()[2][5];
  }
  inline TElement &CovarianceZYaw()
  {
    return this->Covariance()[2][5];
  }

  inline const TElement &CovarianceRollRoll() const
  {
    return this->Covariance()[3][3];
  }
  inline TElement &CovarianceRollRoll()
  {
    return this->Covariance()[3][3];
  }

  inline const TElement &CovarianceRollPitch() const
  {
    return this->Covariance()[3][4];
  }
  inline TElement &CovarianceRollPitch()
  {
    return this->Covariance()[3][4];
  }

  inline const TElement &CovarianceRollYaw() const
  {
    return this->Covariance()[3][5];
  }
  inline TElement &CovarianceRollYaw()
  {
    return this->Covariance()[3][5];
  }

  inline const TElement &CovariancePitchPitch() const
  {
    return this->Covariance()[4][4];
  }
  inline TElement &CovariancePitchPitch()
  {
    return this->Covariance()[4][4];
  }

  inline const TElement &CovariancePitchYaw() const
  {
    return this->Covariance()[4][5];
  }
  inline TElement &CovariancePitchYaw()
  {
    return this->Covariance()[4][5];
  }

  inline const TElement &CovarianceYawYaw() const
  {
    return this->Covariance()[5][5];
  }
  inline TElement &CovarianceYawYaw()
  {
    return this->Covariance()[5][5];
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

  inline const TElement &VarianceZ() const
  {
    return this->CovarianceZZ();
  }
  inline TElement &VarianceZ()
  {
    return this->CovarianceZZ();
  }

  inline const TElement &VarianceRoll() const
  {
    return this->CovarianceRollRoll();
  }
  inline TElement &VarianceRoll()
  {
    return this->CovarianceRollRoll();
  }

  inline const TElement &VariancePitch() const
  {
    return this->CovariancePitchPitch();
  }
  inline TElement &VariancePitch()
  {
    return this->CovariancePitchPitch();
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

#include "rrlib/localization/pose/tUncertainPose3D.hpp"

#endif
