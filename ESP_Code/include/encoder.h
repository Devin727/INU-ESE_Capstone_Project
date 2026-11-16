#pragma once
#include "config.h"

struct EncoderState {
    int64_t count = 0;
    int64_t previousCount = 0;
    uint32_t previousSampleMs = 0;
    uint32_t lastReadMs = 0;
    double rpm = 0.0;
    bool hasCount = false;
    bool rpmValid = false;
};
// 카운트 차이와 실제 경과 시간으로 RPM을 계산하는 함수
bool calculateRpm(double deltaCount, double countsPerRev, double elapsedSeconds, double& rpm);
// 모든 인코더 측정 상태를 초기화하는 함수
void initializeEncoders();
// 일관되게 읽은 누적 카운트로 RPM 측정 상태를 갱신하는 함수
bool submitEncoderCount(int motorId, int64_t count, uint32_t nowMs);
// 인코더 카운트와 RPM 측정값을 읽기 전용으로 가져오는 함수
const EncoderState* getEncoderState(int motorId);
// RPM 측정값이 유효하고 최근에 갱신됐는지 확인하는 함수
bool isEncoderFresh(int motorId, uint32_t nowMs);
