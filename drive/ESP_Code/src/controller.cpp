#include "controller.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include <cmath>
#include <algorithm>

namespace {
RobotConfig activeConfig;
uint32_t lastControlMs = 0;
uint32_t lastUpdateMs = 0;
bool watching = false;
// uint32_t 시간 차이로 비교 가능한 주기인지 확인하는 함수
bool validPeriod(uint32_t value) { return value > 0 && value < 0x80000000u; }
}

// 설정 범위와 RPM 측정에 필요한 시간 관계를 검사하는 함수
bool validateConfig(const RobotConfig& config) {
    if (config.motorCount == 0 || config.motorCount > kMaxMotors ||
        !validPeriod(config.controlPeriodMs) || config.controlPeriodMs > 1000 ||
        !validPeriod(config.encoderPeriodMs) || !validPeriod(config.statusPeriodMs) ||
        !validPeriod(config.commandTimeoutMs) || !validPeriod(config.encoderTimeoutMs) ||
        config.encoderTimeoutMs <= config.encoderPeriodMs || config.reversalPauseMs >= 0x80000000u ||
        (config.stopMode != StopMode::Coast && config.stopMode != StopMode::Brake)) return false;
    for (std::size_t i = 0; i < config.motorCount; ++i) {
        const MotorConfig& m = config.motors[i];
        if (m.outputLimitPercent < 0 || m.outputLimitPercent > 100 ||
            !std::isfinite(m.slewPercentPerSecond) || m.slewPercentPerSecond <= 0.0f ||
            (m.motorSign != 1 && m.motorSign != -1) || (m.encoderSign != 1 && m.encoderSign != -1) ||
            !std::isfinite(m.countsPerOutputRev) || m.countsPerOutputRev < 0.0 ||
            !std::isfinite(m.maxRpm) || m.maxRpm < 0.0f ||
            !std::isfinite(m.kp) || m.kp < 0.0f || !std::isfinite(m.ki) || m.ki < 0.0f) return false;
    }
    return true;
}

// 시작할 때 검증한 제어 설정을 가져오는 함수
const RobotConfig& getConfig() { return activeConfig; }

// 설정을 검사하고 모든 출력과 측정 이력을 초기화하는 함수
bool initializeRobot(const RobotConfig& config, uint32_t nowMs) {
    const bool valid = validateConfig(config);
    activeConfig = valid ? config : RobotConfig{};
    initializeMotors(nowMs);
    initializeEncoders();
    initializeSpeedControl();
    lastUpdateMs = nowMs;
    lastControlMs = nowMs;
    watching = false;
    if (!valid) {
        for (std::size_t i = 0; i < activeConfig.motorCount; ++i)
            latchMotorFault(static_cast<int>(i), FaultHardware, nowMs);
    }
    return valid;
}

// 유효한 제어 명령 수신 시 통신 감시를 시작하거나 갱신하는 함수
void noteControlCommand(uint32_t nowMs) { lastControlMs = nowMs; watching = true; }

// 이미 제어 중일 때만 heartbeat로 감시 시간을 연장하는 함수
void noteHeartbeat(uint32_t nowMs) { if (watching) lastControlMs = nowMs; }

// 전체 정지 후 새로운 제어 명령을 기다리는 함수
void stopRobot(StopMode mode, uint32_t nowMs) {
    stopAllMotors(mode, nowMs);
    watching = false;
}

// 통신 감시를 먼저 검사하고 일정 주기로 PI와 출력 변화를 실행하는 함수
void updateRobot(uint32_t nowMs) {
    if (watching && static_cast<uint32_t>(nowMs - lastControlMs) >= activeConfig.commandTimeoutMs) {
        for (std::size_t i = 0; i < activeConfig.motorCount; ++i)
            latchMotorFault(static_cast<int>(i), FaultTimeout, nowMs);
        watching = false;
    }
    const uint32_t elapsed = nowMs - lastUpdateMs;
    if (elapsed < activeConfig.controlPeriodMs) return;
    lastUpdateMs = nowMs;
    // 긴 실행 지연 후 한 번에 큰 출력이 적용되지 않도록 증가량을 제한한다.
    const float seconds = std::min(elapsed, activeConfig.controlPeriodMs * 2u) / 1000.0f;
    updateSpeedControl(nowMs, seconds);
    updateMotorPower(nowMs, seconds);
}
