#include "speed_control.h"
#include "motor.h"
#include "encoder.h"
#include <algorithm>
#include <cmath>

namespace {
double integral[kMaxMotors] = {};
}

// 모든 모터의 적분 누적값을 지우는 함수
void initializeSpeedControl() {
    for (double& value : integral) value = 0.0;
}

// 모드 전환이나 정지 시 해당 모터의 적분을 초기화하는 함수
void resetSpeedController(int motorId) {
    if (motorId >= 0 && static_cast<std::size_t>(motorId) < kMaxMotors) integral[motorId] = 0.0;
}

// 오차에 비례·적분 제어와 포화 시 적분 누적 방지를 적용하는 함수
void updateSpeedControl(uint32_t nowMs, float elapsedSeconds) {
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        const int id = static_cast<int>(i);
        const MotorState& state = *getMotorState(id);
        if (state.mode != ControlMode::Rpm) continue;
        if (!isEncoderFresh(id, nowMs)) {
            latchMotorFault(id, FaultEncoder, nowMs);
            continue;
        }
        if (state.targetRpm == 0.0f) {
            resetSpeedController(id);
            setControlledPower(id, 0.0f);
            continue;
        }
        const MotorConfig& config = getConfig().motors[i];
        const double error = state.targetRpm - getEncoderState(id)->rpm;
        const double limit = config.outputLimitPercent;
        const double low = state.targetRpm > 0.0f ? 0.0 : -limit;
        const double high = state.targetRpm > 0.0f ? limit : 0.0;
        // 새 방향으로 출발하기 전에는 적분을 쌓지 않는다.
        const int direction = state.targetRpm > 0.0f ? 1 : -1;
        const bool reversing = state.lastDirection != 0 && direction != state.lastDirection;
        if (reversing) integral[i] = 0.0;
        const double candidate = std::max(-limit, std::min(limit,
            integral[i] + (reversing ? 0.0 : config.ki * error * elapsedSeconds)));
        const double raw = config.kp * error + candidate;
        if ((raw >= low && raw <= high) || (raw > high && error < 0.0) || (raw < low && error > 0.0))
            integral[i] = candidate;
        const double output = std::max(low, std::min(high, config.kp * error + integral[i]));
        setControlledPower(id, static_cast<float>(output));
    }
}
