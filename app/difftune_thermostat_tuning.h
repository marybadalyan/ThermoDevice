#pragma once

// DiffTune auto-tuning for the IR-step PID thermostat.
//
// Based on: S. Cheng, M. Kim, L. Song, C. Yang, Y. Jin, S. Wang, N. Hovakimyan,
// "DiffTune: Auto-Tuning through Auto-Differentiation", IEEE T-RO, 2024
// (arXiv:2209.10021).
//
// Idea (paper, Sec. III): treat the closed loop  x_{k+1} = f(x_k, u_k),
// u_k = h(x_k, theta)  as an unrolled computational graph and run gradient
// descent on the controller parameters theta = [kp, ki, kd] to minimise a
// tracking loss  L = sum_k ||r_k - y_k||^2 (+ small effort term).
//
// Because this runs on a *physical* room, we use the paper's "sensitivity
// propagation" form: the real measured trajectory is used for the states and
// only the Jacobians df/dx, df/du come from a model. The model is a first-order
// room/heater model that is identified online with recursive least squares:
//
//   T_{k+1} = T_k + dt * (g * H_k - lambda * T_k + c)       (room temperature)
//   H_{k+1} = H_k + u_k                                     (heater level = sum of IR steps)
//
// Controller states mirror PidThermostatController: integral I and previous
// room temperature Tp. Sensitivity recursion (paper eq. 6):
//
//   dx_{k+1}/dtheta = (df/dx + df/du * dh/dx) dx_k/dtheta + df/du * dh/dtheta
//
// Non-differentiable pieces (rounding to whole IR steps, step clamp, deadband,
// integral clamp) use straight-through / zero-gradient conventions.
// After every trial (N control cycles) theta takes one projected gradient step.

#include <cstdint>

#include "pid_thermostat_controller.h"

class DiffTuneThermostatTuning {
public:
    struct Bounds {
        float min;
        float max;
    };

    struct Config {
        // Control cycles per DiffTune trial (one gradient step per trial).
        uint16_t trialCycles = 30U;
        // Gradient step size, as a fraction of each parameter's allowed range.
        float learningRate = 0.05F;
        // Weight of the actuator-effort term rho * u^2 in the loss.
        float effortWeight = 0.01F;
        // RLS forgetting factor for the room model.
        float rlsForgetting = 0.995F;
        // Model samples required before gradients are trusted.
        uint16_t minModelSamples = 10U;
        Bounds kp{0.3F, 4.0F};
        Bounds ki{0.0F, 0.1F};
        Bounds kd{0.0F, 8.0F};
    };

    struct Stats {
        uint32_t trials = 0U;
        float lastTrialLoss = 0.0F;
        float lastGradKp = 0.0F;
        float lastGradKi = 0.0F;
        float lastGradKd = 0.0F;
        float modelGain = 0.0F;    // g      [C/s per heater step]
        float modelLoss = 0.0F;    // lambda [1/s]
        uint32_t modelSamples = 0U;
    };

    DiffTuneThermostatTuning();
    explicit DiffTuneThermostatTuning(const Config& config);

    // Start from the given gains (e.g. the mode's base tuning).
    void reset(const ThermostatTuning& initial);
    // Drop the running trial (heater turned off, mode changed, PID reset...).
    void resetTrial();

    // Call once for every PID result with ranControlCycle == true.
    // pidConfig must be the config the PID controller is running with.
    // Returns true when a trial finished and the gains were updated.
    bool onControlCycle(float targetC,
                        float roomTempC,
                        const PidThermostatController::Result& result,
                        const PidThermostatController::Config& pidConfig);

    ThermostatTuning tuning() const { return tuning_; }
    const Stats& stats() const { return stats_; }
    bool modelReady() const { return stats_.modelSamples >= config_.minModelSamples; }

private:
    static constexpr int kStates = 4;  // T, H, I, Tp
    static constexpr int kParams = 3;  // kp, ki, kd
    enum { ST_T = 0, ST_H = 1, ST_I = 2, ST_TP = 3 };

    void updateModel(float roomTempC, float dtSeconds);
    void applyGradientStep();
    static float clampFloat(float v, float lo, float hi);

    Config config_{};
    ThermostatTuning tuning_{1.6F, 0.02F, 3.0F, 3};
    Stats stats_{};

    // Sensitivities S[i][j] = d state_i / d theta_j along the trajectory.
    float sens_[kStates][kParams]{};
    float grad_[kParams]{};
    float trialLoss_ = 0.0F;
    uint16_t trialCount_ = 0U;

    // Previous control-cycle sample (for model identification).
    bool havePrev_ = false;
    float prevTempC_ = 0.0F;
    float heaterLevel_ = 0.0F;       // H_k: cumulative IR steps in this session
    float prevHeaterLevel_ = 0.0F;

    // RLS for  dT/dt = w0 * H + w1 * T + w2,  w = [g, -lambda, c].
    float w_[3]{0.0F, 0.0F, 0.0F};
    float P_[3][3]{};
};
