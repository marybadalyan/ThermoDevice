#include "difftune_thermostat_tuning.h"

#include <cmath>

DiffTuneThermostatTuning::DiffTuneThermostatTuning() : DiffTuneThermostatTuning(Config{}) {}

DiffTuneThermostatTuning::DiffTuneThermostatTuning(const Config& config) : config_(config) {
    // Large initial covariance = "know nothing" prior for RLS.
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_[i][j] = (i == j) ? 1000.0F : 0.0F;
        }
    }
}

void DiffTuneThermostatTuning::reset(const ThermostatTuning& initial) {
    tuning_ = initial;
    tuning_.kp = clampFloat(tuning_.kp, config_.kp.min, config_.kp.max);
    tuning_.ki = clampFloat(tuning_.ki, config_.ki.min, config_.ki.max);
    tuning_.kd = clampFloat(tuning_.kd, config_.kd.min, config_.kd.max);
    resetTrial();
}

void DiffTuneThermostatTuning::resetTrial() {
    for (auto& row : sens_) {
        for (float& v : row) {
            v = 0.0F;
        }
    }
    for (float& g : grad_) {
        g = 0.0F;
    }
    trialLoss_ = 0.0F;
    trialCount_ = 0U;
    havePrev_ = false;
    heaterLevel_ = 0.0F;
    prevHeaterLevel_ = 0.0F;
}

void DiffTuneThermostatTuning::updateModel(float roomTempC, float dtSeconds) {
    if (!havePrev_ || dtSeconds <= 0.0F) {
        return;
    }
    // Regression: (T_k - T_{k-1})/dt = g*H_{k-1}^+ - lambda*T_{k-1} + c
    const float phi[3] = {prevHeaterLevel_, prevTempC_, 1.0F};
    const float y = (roomTempC - prevTempC_) / dtSeconds;

    float Pphi[3];
    for (int i = 0; i < 3; ++i) {
        Pphi[i] = P_[i][0] * phi[0] + P_[i][1] * phi[1] + P_[i][2] * phi[2];
    }
    const float denom = config_.rlsForgetting + phi[0] * Pphi[0] + phi[1] * Pphi[1] + phi[2] * Pphi[2];
    if (denom <= 1e-9F) {
        return;
    }
    const float pred = w_[0] * phi[0] + w_[1] * phi[1] + w_[2] * phi[2];
    const float err = y - pred;
    for (int i = 0; i < 3; ++i) {
        w_[i] += (Pphi[i] / denom) * err;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_[i][j] = (P_[i][j] - (Pphi[i] * Pphi[j]) / denom) / config_.rlsForgetting;
        }
    }

    stats_.modelGain = (w_[0] > 0.0F) ? w_[0] : 0.0F;
    stats_.modelLoss = (w_[1] < 0.0F) ? -w_[1] : 0.0F;
    stats_.modelSamples++;
}

bool DiffTuneThermostatTuning::onControlCycle(float targetC,
                                              float roomTempC,
                                              const PidThermostatController::Result& r,
                                              const PidThermostatController::Config& pidConfig) {
    if (!r.ranControlCycle) {
        return false;
    }
    const float dt = static_cast<float>(pidConfig.controlIntervalMs) / 1000.0F;

    // 1) Identify the room model from the real transition (k-1 -> k).
    updateModel(roomTempC, dt);

    const float u = static_cast<float>(r.steps);
    const bool useGradients = modelReady() && havePrev_;

    if (useGradients) {
        float (&S)[kStates][kParams] = sens_;
        const float e = targetC - roomTempC;

        // 2) Loss and its gradient: L += e^2 + rho*u^2,  de/dtheta = -dT/dtheta.
        trialLoss_ += e * e + config_.effortWeight * u * u;

        // 3) Controller Jacobians at this step (straight-through for rounding).
        float dI[kParams];  // d(updated integral)/dtheta
        float dD[kParams];  // d(derivative signal)/dtheta
        float dU[kParams];  // d(u)/dtheta (total)
        const bool integralFrozen = r.inDeadband || r.integralClamped;
        const bool outputFrozen = r.inDeadband || r.outputSaturated;
        const float dParam[kParams] = {r.errorC, r.integral, r.derivative};  // du/dtheta, partial

        for (int j = 0; j < kParams; ++j) {
            dI[j] = integralFrozen ? 0.0F : (S[ST_I][j] - dt * S[ST_T][j]);
            dD[j] = r.derivativeActive ? (-(S[ST_T][j] - S[ST_TP][j]) / dt) : 0.0F;
            dU[j] = outputFrozen ? 0.0F
                                 : (dParam[j] + tuning_.kp * (-S[ST_T][j]) + tuning_.ki * dI[j] +
                                    tuning_.kd * dD[j]);

            grad_[j] += -2.0F * e * S[ST_T][j] + 2.0F * config_.effortWeight * u * dU[j];
        }

        // 4) Propagate sensitivities through the identified model (paper eq. 6).
        const float a = 1.0F - dt * stats_.modelLoss;  // dT+/dT
        const float b = dt * stats_.modelGain;         // dT+/dH+
        for (int j = 0; j < kParams; ++j) {
            const float sT = S[ST_T][j];
            const float sHplus = S[ST_H][j] + dU[j];
            S[ST_T][j] = a * sT + b * sHplus;
            S[ST_H][j] = sHplus;
            S[ST_I][j] = dI[j];
            S[ST_TP][j] = sT;
        }
        trialCount_++;
    }

    // Book-keeping for the next model update.
    heaterLevel_ += u;
    prevHeaterLevel_ = heaterLevel_;
    prevTempC_ = roomTempC;
    havePrev_ = true;

    // 5) End of trial -> one gradient step on (kp, ki, kd).
    if (trialCount_ >= config_.trialCycles) {
        applyGradientStep();
        stats_.trials++;
        stats_.lastTrialLoss = trialLoss_ / static_cast<float>(trialCount_);
        // New trial: sensitivities restart from zero (fresh unroll), model is kept.
        for (auto& row : sens_) {
            for (float& v : row) {
                v = 0.0F;
            }
        }
        for (float& g : grad_) {
            g = 0.0F;
        }
        trialLoss_ = 0.0F;
        trialCount_ = 0U;
        return true;
    }
    return false;
}

void DiffTuneThermostatTuning::applyGradientStep() {
    stats_.lastGradKp = grad_[0];
    stats_.lastGradKi = grad_[1];
    stats_.lastGradKd = grad_[2];

    // Scale each gradient component by its parameter range so kp (~1), ki (~0.02)
    // and kd (~3) move comparably, then normalise the step length. This keeps
    // vanilla projected gradient descent stable without per-room hyperparameters.
    const Bounds* bounds[kParams] = {&config_.kp, &config_.ki, &config_.kd};
    float scaled[kParams];
    float norm = 0.0F;
    for (int j = 0; j < kParams; ++j) {
        const float range = bounds[j]->max - bounds[j]->min;
        scaled[j] = grad_[j] * range;
        norm += scaled[j] * scaled[j];
    }
    norm = std::sqrt(norm);
    if (!(norm > 1e-9F)) {
        return;
    }

    float* params[kParams] = {&tuning_.kp, &tuning_.ki, &tuning_.kd};
    for (int j = 0; j < kParams; ++j) {
        const float range = bounds[j]->max - bounds[j]->min;
        *params[j] -= config_.learningRate * range * (scaled[j] / norm);
        *params[j] = clampFloat(*params[j], bounds[j]->min, bounds[j]->max);  // projection
    }
}

float DiffTuneThermostatTuning::clampFloat(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}
