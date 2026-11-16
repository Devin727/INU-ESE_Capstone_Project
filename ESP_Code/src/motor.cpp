#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include <algorithm>
#include <cmath>

namespace {
MotorState states[kMaxMotors];
// 활성화한 모터 번호인지 확인하는 함수
bool validId(int id) {
    return id >= 0 && static_cast<std::size_t>(id) < getConfig().motorCount;
}
// 타임아웃 이외의 고정 오류가 없는지 확인하는 함수
bool canCommand(int id) {
    return validId(id) && (states[id].fault & ~static_cast<unsigned>(FaultTimeout)) == 0;
}
// 현재 출력을 목표 방향으로 정해진 크기만큼 이동하는 함수
float approach(float current, float target, float step) {
    return current < target ? std::min(current + step, target) : std::max(current - step, target);
}
}

// 출력 요청의 방향과 크기를 DRV8833의 두 입력 듀티로 변환하는 함수
MotorDuty calculateMotorDuty(int percent, int limitPercent) {
    // 잘못된 입력은 거절한다. 이 결과를 실제 출력에 적용하면 안 된다.
    if (percent < -100 || percent > 100 ||
        limitPercent < 0 || limitPercent > 100) {
        return {false, 0, 0};
    }

    int magnitude = (percent < 0) ? -percent : percent;

    // 요청 크기가 출력 상한보다 크면 상한까지만 허용한다.
    if (magnitude > limitPercent) {
        magnitude = limitPercent;
    }

    if (percent > 0) {
        return {true, magnitude, 0};
    }

    if (percent < 0) {
        return {true, 0, magnitude};
    }

    // 0% 요청은 두 입력 모두 0%인 코스트 조합으로 변환한다.
    return {true, 0, 0};
}

// 유효한 요청만 모터의 목표 출력으로 저장하는 함수
bool setMotorPower(int motorId, int percent) {
    if (!canCommand(motorId) || percent < -100 || percent > 100) {
        return false;
    }
    MotorState& state = states[motorId];
    state.mode = ControlMode::Pwm;
    state.targetPowerPercent = percent;
    state.targetRpm = 0.0f;
    state.requestedPercent = static_cast<float>(percent);
    state.fault &= ~static_cast<unsigned>(FaultTimeout);
    resetSpeedController(motorId);
    return true;
}

// 저장된 목표 출력을 읽고, 잘못된 모터 번호는 거절하는 함수
bool getMotorTargetPower(int motorId, int& percent) {
    if (!validId(motorId)) {
        return false;
    }

    percent = states[motorId].targetPowerPercent;
    return true;
}

// 모든 모터를 출력이 없는 초기 상태로 바꾸는 함수
void initializeMotors(uint32_t nowMs) {
    for (MotorState& state : states) {
        state = MotorState{};
        state.zeroSinceMs = nowMs;
        state.stopMode = StopMode::Coast; // 부팅 직후에는 브레이크도 인가하지 않는다.
    }
}

// 설정과 측정값이 준비된 모터에 RPM 목표를 저장하는 함수
bool setMotorRpm(int motorId, float rpm, uint32_t nowMs) {
    if (!canCommand(motorId) || !std::isfinite(rpm)) return false;
    const MotorConfig& config = getConfig().motors[motorId];
    if (config.countsPerOutputRev <= 0.0 || config.maxRpm <= 0.0f ||
        (config.kp == 0.0f && config.ki == 0.0f) ||
        std::fabs(rpm) > config.maxRpm || !isEncoderFresh(motorId, nowMs)) return false;
    MotorState& state = states[motorId];
    if (state.mode != ControlMode::Rpm || rpm == 0.0f ||
        (state.targetRpm > 0.0f) != (rpm > 0.0f)) resetSpeedController(motorId);
    state.mode = ControlMode::Rpm;
    state.targetRpm = rpm;
    state.targetPowerPercent = 0;
    state.requestedPercent = 0.0f;
    state.fault &= ~static_cast<unsigned>(FaultTimeout);
    return true;
}

// 모터의 목표와 출력을 즉시 지우고 지정한 정지 방식을 적용하는 함수
bool stopMotor(int motorId, StopMode mode, uint32_t nowMs) {
    if (!validId(motorId)) return false;
    MotorState& state = states[motorId];
    if (state.appliedPercent != 0.0f) state.zeroSinceMs = nowMs;
    state.mode = ControlMode::Stopped;
    state.targetPowerPercent = 0;
    state.targetRpm = 0.0f;
    state.requestedPercent = 0.0f;
    state.appliedPercent = 0.0f;
    state.stopMode = mode;
    resetSpeedController(motorId);
    return true;
}

// 활성화한 모든 모터를 즉시 정지시키는 함수
void stopAllMotors(StopMode mode, uint32_t nowMs) {
    for (std::size_t i = 0; i < getConfig().motorCount; ++i)
        stopMotor(static_cast<int>(i), mode, nowMs);
}

// 오류를 보존하면서 해당 모터를 즉시 정지시키는 함수
void latchMotorFault(int motorId, unsigned fault, uint32_t nowMs) {
    if (!validId(motorId)) return;
    states[motorId].fault |= fault;
    stopMotor(motorId, getConfig().stopMode, nowMs);
}

// 저장된 모터 상태를 읽기 전용 포인터로 가져오는 함수
const MotorState* getMotorState(int motorId) {
    return validId(motorId) ? &states[motorId] : nullptr;
}

// PI 제어기가 계산한 출력을 RPM 모드의 목표 듀티로 저장하는 함수
bool setControlledPower(int motorId, float percent) {
    if (!validId(motorId) || !std::isfinite(percent) ||
        states[motorId].mode != ControlMode::Rpm || states[motorId].fault != FaultNone) return false;
    const float limit = static_cast<float>(getConfig().motors[motorId].outputLimitPercent);
    states[motorId].requestedPercent = std::max(-limit, std::min(percent, limit));
    return true;
}

// 현재 출력을 점진적으로 바꾸고 반전 전에 0%에서 기다리는 함수
void updateMotorPower(uint32_t nowMs, float elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0f) return;
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        MotorState& state = states[i];
        if (state.mode == ControlMode::Stopped || state.fault != FaultNone) continue;
        const MotorConfig& config = getConfig().motors[i];
        const float limit = static_cast<float>(config.outputLimitPercent);
        float target = std::max(-limit, std::min(state.requestedPercent, limit));
        const int direction = (target > 0.0f) - (target < 0.0f);
        const bool reversing = direction != 0 && state.lastDirection != 0 &&
                               direction != state.lastDirection;
        if (reversing) {
            if (state.appliedPercent != 0.0f ||
                static_cast<uint32_t>(nowMs - state.zeroSinceMs) < getConfig().reversalPauseMs)
                target = 0.0f;
        }
        const float previous = state.appliedPercent;
        state.appliedPercent = approach(previous, target, config.slewPercentPerSecond * elapsedSeconds);
        if (previous != 0.0f && state.appliedPercent == 0.0f) state.zeroSinceMs = nowMs;
        if (state.appliedPercent != 0.0f)
            state.lastDirection = state.appliedPercent > 0.0f ? 1 : -1;
    }
}

// 모터의 방향 설정과 브레이크 상태를 최종 PWM 듀티에 반영하는 함수
MotorDuty getMotorDuty(int motorId) {
    if (!validId(motorId)) return {false, 0, 0};
    const MotorState& state = states[motorId];
    if (state.mode == ControlMode::Stopped)
        return state.stopMode == StopMode::Brake ? MotorDuty{true, 100, 100} : MotorDuty{true, 0, 0};
    const MotorConfig& config = getConfig().motors[motorId];
    const int percent = static_cast<int>(std::lround(state.appliedPercent)) * config.motorSign;
    return calculateMotorDuty(percent, config.outputLimitPercent);
}
