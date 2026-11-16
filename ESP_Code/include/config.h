#pragma once
#include <cstddef>
#include <cstdint>

constexpr std::size_t kMaxMotors = 6;
constexpr std::size_t kCommandCapacity = 96;
enum class StopMode { Coast, Brake };
enum class ControlMode { Stopped, Pwm, Rpm };
enum Fault : unsigned { FaultNone = 0, FaultTimeout = 1, FaultEncoder = 2, FaultHardware = 4 };

struct MotorConfig {
    int outputLimitPercent = 20; // 준비용 초기값. 실물 시험 전에 결정한다.
    float slewPercentPerSecond = 50.0f;
    int motorSign = 1;
    int encoderSign = 1;
    double countsPerOutputRev = 0.0; // 미정. 0이면 RPM 모드를 허용하지 않는다.
    float maxRpm = 0.0f;
    float kp = 0.0f; // %/RPM
    float ki = 0.0f; // %/(RPM*s)
};
struct RobotConfig {
    std::size_t motorCount = 1;
    uint32_t controlPeriodMs = 10;
    uint32_t encoderPeriodMs = 100;
    uint32_t statusPeriodMs = 200;
    uint32_t commandTimeoutMs = 500;
    uint32_t encoderTimeoutMs = 300;
    uint32_t reversalPauseMs = 200;
    StopMode stopMode = StopMode::Coast;
    MotorConfig motors[kMaxMotors];
};
// 제어 설정의 범위와 조합을 검사하는 함수
bool validateConfig(const RobotConfig& config);
// 현재 제어 설정을 읽기 전용으로 가져오는 함수
const RobotConfig& getConfig();
