/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_MATH_PID_HH_
#define GZ_MATH_PID_HH_

#include <chrono>
#include <gz/math/Helpers.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::math
{
  // Inline bracket to help doxygen filtering.
  inline namespace GZ_MATH_VERSION_NAMESPACE {
  //
  /// \class PID PID.hh gz/math/PID.hh
  /// \brief Generic PID controller class.
  /// Generic proportional-integral-derivative controller class that
  /// keeps track of PID-error states and control inputs given
  /// the state of a system and a user specified target state.
  /// It includes a user-adjustable command offset term (feed-forward).
  // cppcheck-suppress class_X_Y
  class GZ_MATH_VISIBLE PID
  {
    /// \brief Constructor, zeros out Pid values when created and
    /// initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
    ///
    /// Disable command clamping by setting _cmdMin to a value larger
    /// than _cmdMax. Command clamping is disabled by default.
    ///
    /// Disable integral clamping by setting _iMin to a value larger
    /// than _iMax. Integral clamping is disabled by default.
    ///
    /// \param[in] _p  The proportional gain.
    /// \param[in] _i  The integral gain.
    /// \param[in] _d  The derivative gain.
    /// \param[in] _imax The integral upper limit.
    /// \param[in] _imin The integral lower limit.
    /// \param[in] _cmdMax Output max value.
    /// \param[in] _cmdMin Output min value.
    /// \param[in] _cmdOffset Command offset (feed-forward).
    public: PID(const double _p = 0.0,
                const double _i = 0.0,
                const double _d = 0.0,
                const double _imax = -1.0,
                const double _imin = 0.0,
                const double _cmdMax = -1.0,
                const double _cmdMin = 0.0,
                const double _cmdOffset = 0.0);

    /// \brief Initialize PID-gains and integral term
    ///        limits:[iMax:iMin]-[I1:I2].
    ///
    /// Disable command clamping by setting _cmdMin to a value larger
    /// than _cmdMax. Command clamping is disabled by default.
    ///
    /// Disable integral clamping by setting _iMin to a value larger
    /// than _iMax. Integral clamping is disabled by default.
    ///
    /// \param[in] _p  The proportional gain.
    /// \param[in] _i  The integral gain.
    /// \param[in] _d  The derivative gain.
    /// \param[in] _imax The integral upper limit.
    /// \param[in] _imin The integral lower limit.
    /// \param[in] _cmdMax Output max value.
    /// \param[in] _cmdMin Output min value.
    /// \param[in] _cmdOffset Command offset (feed-forward).
    public: void Init(const double _p = 0.0,
                      const double _i = 0.0,
                      const double _d = 0.0,
                      const double _imax = -1.0,
                      const double _imin = 0.0,
                      const double _cmdMax = -1.0,
                      const double _cmdMin = 0.0,
                      const double _cmdOffset = 0.0);

    /// \brief Set the proportional Gain.
    /// \param[in] _p proportional gain value
    public: void SetPGain(const double _p);

    /// \brief Set the integral Gain.
    /// \param[in] _i integral gain value
    public: void SetIGain(const double _i);

    /// \brief Set the derivative Gain.
    /// \param[in] _d derivative gain value
    public: void SetDGain(const double _d);

    /// \brief Set the integral upper limit.
    /// \param[in] _i integral upper limit value
    public: void SetIMax(const double _i);

    /// \brief Set the integral lower limit.
    /// \param[in] _i integral lower limit value
    public: void SetIMin(const double _i);

    /// \brief Set the maximum value for the command.
    /// \param[in] _c The maximum value
    public: void SetCmdMax(const double _c);

    /// \brief Set the minimum value for the command.
    /// \param[in] _c The minimum value
    public: void SetCmdMin(const double _c);

    /// \brief Set the offset value for the command,
    /// which is added to the result of the PID controller.
    /// \param[in] _c The offset value
    public: void SetCmdOffset(const double _c);

    /// \brief Get the proportional Gain.
    /// \return The proportional gain value
    public: double PGain() const;

    /// \brief Get the integral Gain.
    /// \return The integral gain value
    public: double IGain() const;

    /// \brief Get the derivative Gain.
    /// \return The derivative gain value
    public: double DGain() const;

    /// \brief Get the integral upper limit.
    /// \return The integral upper limit value
    public: double IMax() const;

    /// \brief Get the integral lower limit.
    /// \return The integral lower limit value
    public: double IMin() const;

    /// \brief Get the maximum value for the command.
    /// \return The maximum value
    public: double CmdMax() const;

    /// \brief Get the minimum value for the command.
    /// \return The maximum value
    public: double CmdMin() const;

    /// \brief Get the offset value for the command.
    /// \return The offset value
    public: double CmdOffset() const;

    /// \brief Update the Pid loop with nonuniform time step size.
    /// \param[in] _error  Error since last call (p_state - p_target).
    /// \param[in] _errorRate Estimate of error rate, that can be used
    /// when a smoother estimate is available than the finite difference
    /// used by Update(const double _error,
    /// const std::chrono::duration<double> &_dt)
    /// \param[in] _dt Change in time since last update call.
    /// Normally, this is called at every time step,
    /// The return value is an updated command to be passed
    /// to the object being controlled.
    /// \return the command value
    public: double Update(const double _error,
                          double _errorRate,
                          const std::chrono::duration<double> &_dt);

    /// \brief Update the Pid loop with nonuniform time step size.
    /// \param[in] _error  Error since last call (p_state - p_target).
    /// \param[in] _dt Change in time since last update call.
    /// Normally, this is called at every time step,
    /// The return value is an updated command to be passed
    /// to the object being controlled.
    /// \return the command value
    public: double Update(const double _error,
                          const std::chrono::duration<double> &_dt);

    /// \brief Set current target command for this PID controller.
    /// \param[in] _cmd New command
    public: void SetCmd(const double _cmd);

    /// \brief Return current command for this PID controller.
    /// \return the command value
    public: double Cmd() const;

    /// \brief Return PID error terms for the controller.
    /// \param[in] _pe  The proportional error.
    /// \param[in] _ie  The integral of gain times error.
    /// \param[in] _de  The derivative error.
    public: void Errors(double &_pe, double &_ie, double &_de) const;

    /// \brief Reset the errors and command.
    public: void Reset();

    /// \brief Pointer to private data.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }  // namespace GZ_MATH_VERSION_NAMESPACEH
}  // namespace gz::math
#endif  // GZ_MATH_PID_HH_
