/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CENTAURO_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CENTAURO_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot Centauro.
 */
class CentauroKinematicModel : public KinematicModel {
public:
  CentauroKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.35;
    const double y_nominal_b = 0.35;
    const double z_nominal_b = -0.668; // 5 cm lower than homing

    nominal_stance_.at(LF) <<  x_nominal_b - 0.1,   y_nominal_b, z_nominal_b;   // CoM 10 cm forward
    nominal_stance_.at(RF) <<  x_nominal_b - 0.1,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b - 0.1,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b - 0.1,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.25, 0.15, 0.15;   //dz = 0.25
    // Forward feet -0.25<dx<0.3, -0.2<dy<0.25, -0.15<dz<0.3
    // Hind feet    -0.3<dx<0.3, -0.25<dy<0.25, -0.15<dz<0.3
  }
};

/**
 * @brief The Dynamics of the quadruped robot Centauro.
 */

class CentauroDynamicModel : public SRBD_const_momentum {
public:
  CentauroDynamicModel()
  : SRBD_const_momentum(95, 4) {}
};
/*
class CentauroDynamicModel : public SingleRigidBodyDynamics {
public:
  CentauroDynamicModel()
  : SingleRigidBodyDynamics(95,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    4) {}
};
*/

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CENTAURO_MODEL_H_ */
