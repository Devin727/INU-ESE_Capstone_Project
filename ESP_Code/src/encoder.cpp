#include "encoder.h"
#include <cmath>

namespace {
EncoderState encoders[kMaxMotors];
}

// 카운트 변화량을 출력축의 분당 회전수로 변환하는 함수
bool calculateRpm(double deltaCount, double countsPerRev, double elapsedSeconds, double& rpm) {
    if (!std::isfinite(deltaCount) || !std::isfinite(countsPerRev) ||
        !std::isfinite(elapsedSeconds) || countsPerRev <= 0.0 || elapsedSeconds <= 0.0) return false;
    const double result = (deltaCount / countsPerRev) * (60.0 / elapsedSeconds);
    if (!std::isfinite(result)) return false;
    rpm = result;
    return true;
}

// 누적 카운트와 RPM 측정 이력을 초기화하는 함수
void initializeEncoders() {
    for (EncoderState& encoder : encoders) encoder = EncoderState{};
}

// 누적 카운트와 실제 측정 간격으로 모터별 RPM을 갱신하는 함수
bool submitEncoderCount(int motorId, int64_t count, uint32_t nowMs) {
    if (motorId < 0 || static_cast<std::size_t>(motorId) >= getConfig().motorCount) return false;
    EncoderState& encoder = encoders[motorId];
    encoder.count = count;
    encoder.lastReadMs = nowMs;
    if (!encoder.hasCount) {
        encoder.hasCount = true;
        encoder.previousCount = count;
        encoder.previousSampleMs = nowMs;
        return true;
    }
    const uint32_t elapsed = nowMs - encoder.previousSampleMs;
    if (elapsed < getConfig().encoderPeriodMs) return true;
    const MotorConfig& config = getConfig().motors[motorId];
    // unsigned 차이로 오버플로와 큰 누적값의 실수 변환에 따른 정밀도 손실을 피한다.
    const double difference = count >= encoder.previousCount
        ? static_cast<double>(static_cast<uint64_t>(count) - static_cast<uint64_t>(encoder.previousCount))
        : -static_cast<double>(static_cast<uint64_t>(encoder.previousCount) - static_cast<uint64_t>(count));
    const double delta = difference * config.encoderSign;
    encoder.rpmValid = calculateRpm(delta, config.countsPerOutputRev, elapsed / 1000.0, encoder.rpm);
    encoder.previousCount = count;
    encoder.previousSampleMs = nowMs;
    return true;
}

// 지정한 모터의 측정 상태를 가져오는 함수
const EncoderState* getEncoderState(int motorId) {
    return motorId >= 0 && static_cast<std::size_t>(motorId) < getConfig().motorCount ? &encoders[motorId] : nullptr;
}

// 카운트 읽기와 RPM 측정이 모두 최근에 수행됐는지 확인하는 함수
bool isEncoderFresh(int motorId, uint32_t nowMs) {
    const EncoderState* encoder = getEncoderState(motorId);
    return encoder && encoder->rpmValid &&
        static_cast<uint32_t>(nowMs - encoder->lastReadMs) < getConfig().encoderTimeoutMs &&
        static_cast<uint32_t>(nowMs - encoder->previousSampleMs) < getConfig().encoderTimeoutMs;
}
