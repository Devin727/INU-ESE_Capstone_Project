#pragma once
#include "config.h"

// DRV8833의 두 입력에 보낼 PWM 듀티(%). GPIO에 적용하기 전의 계산 결과다.
struct MotorDuty {
    bool valid;      // 입력값이 유효하면 true
    int in1Percent;  // 첫 번째 입력의 듀티: 0~100%
    int in2Percent;  // 두 번째 입력의 듀티: 0~100%
};

// 부호 있는 출력 요청을 출력 상한 이내의 두 입력 듀티로 변환하는 함수
MotorDuty calculateMotorDuty(int percent, int limitPercent);

// 모터 번호와 출력 요청을 확인하고 목표 출력을 저장하는 함수
bool setMotorPower(int motorId, int percent);

// 모터의 목표 출력(%)을 읽는 함수
bool getMotorTargetPower(int motorId, int& percent);

struct MotorState {
    ControlMode mode = ControlMode::Stopped;
    int targetPowerPercent = 0;
    float targetRpm = 0.0f;
    float requestedPercent = 0.0f;
    float appliedPercent = 0.0f;
    StopMode stopMode = StopMode::Coast;
    unsigned fault = FaultNone;
    int lastDirection = 0;
    uint32_t zeroSinceMs = 0;
};
// 모든 모터의 목표와 현재 출력을 0으로 초기화하는 함수
void initializeMotors(uint32_t nowMs);
// 설정과 센서 상태를 확인하고 목표 RPM을 저장하는 함수
bool setMotorRpm(int motorId, float rpm, uint32_t nowMs);
// 지정한 모터를 즉시 코스트 또는 브레이크 상태로 바꾸는 함수
bool stopMotor(int motorId, StopMode mode, uint32_t nowMs);
// 모든 모터를 즉시 정지시키는 함수
void stopAllMotors(StopMode mode, uint32_t nowMs);
// 모터에 오류를 기록하고 출력을 즉시 끄는 함수
void latchMotorFault(int motorId, unsigned fault, uint32_t nowMs);
// 모터의 상태를 읽기 전용으로 가져오는 함수
const MotorState* getMotorState(int motorId);
// 속도 제어기가 계산한 출력을 목표 듀티로 전달하는 함수
bool setControlledPower(int motorId, float percent);
// 출력 변화율과 방향 반전 대기를 적용하는 함수
void updateMotorPower(uint32_t nowMs, float elapsedSeconds);
// 부호 설정과 정지 모드를 반영한 최종 입력 듀티를 가져오는 함수
MotorDuty getMotorDuty(int motorId);

