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

#include <towr/initialization/gait_generator.h>
#include <towr_ros/towr_ros_interface.h>


namespace towr {

/**
 * @brief An example application of using TOWR together with ROS.
 *
 * Build your own application with your own formulation using the building
 * blocks provided in TOWR and following the example below.
 */
class TowrRosApp : public TowrRosInterface {
public:
  /**
   * @brief Sets the feet to nominal position on flat ground and base above.
   */
  void SetTowrInitialState() override
  {
    auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();

    double z_ground = 0.0;
    formulation_.initial_ee_W_ =  nominal_stance_B;
    std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
                  [&](Vector3d& p){ p.z() = z_ground; } // feet at 0 height
    );

    formulation_.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;
  }

  /**
   * @brief Sets the parameters required to formulate the TOWR problem.
   */
  Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const override
  {
    Parameters params;

    // Instead of manually defining the initial durations for each foot and
    // step, for convenience we use a GaitGenerator with some predefined gaits
    // for a variety of robots (walk, trot, pace, ...).
    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
    auto id_gait   = static_cast<GaitGenerator::Combos>(msg.gait);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      params.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(msg.total_duration, ee));
      params.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }


    // alternating stance and swing:     ____-----_____
    // this code will create problem for biped, monoped
    // better use GaitGenerator for them
/*    params.ee_phase_durations_.push_back({1.0, 1.0, 7.0});
    params.ee_phase_durations_.push_back({3.0, 1.0, 5.0});
    params.ee_phase_durations_.push_back({5.0, 1.0, 3.0});
    params.ee_phase_durations_.push_back({7.0, 1.0, 1.0});

    params.ee_in_contact_at_start_.push_back(true);
    params.ee_in_contact_at_start_.push_back(true);
    params.ee_in_contact_at_start_.push_back(true);
    params.ee_in_contact_at_start_.push_back(true);

    // constructs optimization variables
    params.duration_base_polynomial_ = 0.3;
    params.force_polynomials_per_stance_phase_ = 3;
    params.ee_polynomials_per_swing_phase_ = 2; // so step can at least lift leg

    // parameters related to specific constraints (only used when it is added as well)
    params.force_limit_in_normal_direction_ = 1000;
    params.dt_constraint_range_of_motion_ = 0.3;
    params.dt_constraint_dynamic_ = 0.3;
    //params_.dt_constraint_base_motion_ = formulation.params_.duration_base_polynomial_/4.; // only for base RoM constraint if added
*/
    // Here you can also add other constraints or change parameters
    // params.constraints_.push_back(Parameters::BaseRom);

    // increases optimization time, but sometimes helps find a solution for
    // more difficult terrain.
    if (msg.optimize_phase_durations)
    {
        //params.bound_phase_duration_ = std::make_pair(5.0, 5.0);
        params.OptimizePhaseDurations();
    }
    return params;
  }

  /**
   * @brief Sets the paramters for IPOPT.
   */
  void SetIpoptParameters(const TowrCommandMsg& msg) override
  {
    solver_->SetOption("print_user_options", "yes");

    // the HA-L solvers are alot faster, so consider installing and using
    solver_->SetOption("linear_solver", "mumps"); // ma27, ma57

    // Analytically defining the derivatives in IFOPT as we do it, makes the
    // problem a lot faster. However, if this becomes too difficult, we can also
    // tell IPOPT to just approximate them using finite differences. However,
    // this uses numerical derivatives for ALL constraints, there doesn't yet
    // exist an option to turn on numerical derivatives for only some constraint
    // sets.
    solver_->SetOption("jacobian_approximation", "exact"); // finite difference-values

    // This is a great to test if the analytical derivatives implemented in are
    // correct. Some derivatives that are correct are still flagged, showing a
    // deviation of 10e-4, which is fine. What to watch out for is deviations > 10e-2.
    // solver_->SetOption("derivative_test_tol", 1.0e-2);
    // solver_->SetOption("derivative_test", "first-order");

    solver_->SetOption("max_cpu_time", 250.0);
    solver_->SetOption("print_level", 5);
    solver_->SetOption("print_timing_statistics", "yes");

    if (msg.play_initialization)
      solver_->SetOption("max_iter", 0);
    else
      solver_->SetOption("max_iter", 3000);
  }
};

} // namespace towr


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "my_towr_ros_app");
  towr::TowrRosApp towr_app;
  ros::spin();

  return 1;
}
