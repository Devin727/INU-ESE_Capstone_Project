#pragma once
#include "config.h"

struct MotorPins {
    int in1 = -1;
    int in2 = -1;
    bool encoderEnabled = false;
    int encoderA = -1;
    int encoderB = -1;
    bool encoderPullup = false;
};
struct BoardSettings {
    bool hardwareEnabled = false;
    int sleepPin = -1; // 여러 드라이버의 nSLEEP를 함께 제어할 때 공통으로 연결한다.
    bool externalSleepConfirmed = false; // 외부 활성화 회로 확인 후에만 true로 설정한다.
    int faultPin = -1; // 선택: 3.3V 기준의 공통 nFAULT 입력
    uint32_t pwmFrequencyHz = 0;
    uint8_t pwmResolutionBits = 8;
    uint32_t encoderGlitchFilterNs = 0;
    MotorPins motors[kMaxMotors];
};
// 보드 설정과 하드웨어 자원을 확인하고 출력이 꺼진 상태로 초기화하는 함수
bool initializeHardware(const BoardSettings& settings, uint32_t nowMs);
// 하드웨어 카운트를 읽어 공통 인코더 측정 코드에 전달하는 함수
bool readHardwareEncoders(uint32_t nowMs);
// 공통 제어 코드의 듀티를 LEDC 출력에 적용하는 함수
bool applyHardwareOutputs(uint32_t nowMs);
// 모든 PWM을 끄고 가능한 경우 드라이버를 비활성화하는 함수
void shutdownHardware(uint32_t nowMs);
