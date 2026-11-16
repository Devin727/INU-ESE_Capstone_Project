#include "communication.h"
#include "controller.h"
#include "encoder.h"
#include "motor.h"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace {
int checks = 0;
// 검증이 실패하면 위치를 출력하고 실행을 종료하는 함수
void check(bool condition, int line) {
    ++checks;
    if (!condition) { std::printf("FAIL at line %d\n", line); std::exit(1); }
}
#define CHECK(condition) check((condition), __LINE__)

// PC 시험 전용 가상 사양을 만드는 함수
RobotConfig testConfig(std::size_t count = 2) {
    RobotConfig config;
    config.motorCount = count;
    for (std::size_t i = 0; i < count; ++i) {
        config.motors[i].outputLimitPercent = 40;
        config.motors[i].slewPercentPerSecond = 100.0f;
        config.motors[i].countsPerOutputRev = 1000.0;
        config.motors[i].maxRpm = 300.0f;
        config.motors[i].kp = 0.5f;
        config.motors[i].ki = 0.2f;
    }
    return config;
}
// 명령 처리 응답을 문자열에 저장하는 함수
bool collect(const char* text, void* context) {
    static_cast<std::string*>(context)->append(text);
    return true;
}
// 줄바꿈을 포함한 입력을 한 글자씩 전달하는 함수
void feed(Communication& communication, const std::string& text, uint32_t now) {
    for (char byte : text) receiveCommandByte(communication, byte, now);
}
// 정수 출력의 모든 조합과 잘못된 경계값을 검사하는 함수
void testDuty() {
    for (int limit = 0; limit <= 100; ++limit) {
        for (int percent = -100; percent <= 100; ++percent) {
            const MotorDuty duty = calculateMotorDuty(percent, limit);
            CHECK(duty.valid);
            CHECK(duty.in1Percent >= 0 && duty.in1Percent <= limit);
            CHECK(duty.in2Percent >= 0 && duty.in2Percent <= limit);
            CHECK(duty.in1Percent == 0 || duty.in2Percent == 0);
            CHECK(duty.in1Percent + duty.in2Percent == std::min(std::abs(percent), limit));
            CHECK(percent >= 0 ? duty.in2Percent == 0 : duty.in1Percent == 0);
        }
    }
    CHECK(!calculateMotorDuty(101, 20).valid);
    CHECK(!calculateMotorDuty(std::numeric_limits<int>::min(), 20).valid);
    CHECK(!calculateMotorDuty(0, -1).valid);
    CHECK(!calculateMotorDuty(0, 101).valid);
}
// 초기화·잘못된 설정·여러 모터의 독립 제어를 검사하는 함수
void testConfigAndState() {
    RobotConfig config = testConfig(6);
    CHECK(initializeRobot(config, 0));
    for (int i = 0; i < 6; ++i) {
        CHECK(getMotorDuty(i).in1Percent == 0);
        CHECK(setMotorPower(i, i * 10));
    }
    CHECK(!setMotorPower(6, 10));
    CHECK(!setMotorPower(-1, 10));
    CHECK(!setMotorPower(0, 101));
    int target = 42;
    CHECK(!getMotorTargetPower(-1, target) && target == 42);
    CHECK(getMotorTargetPower(5, target) && target == 50);
    updateRobot(100);
    CHECK(getMotorState(0)->appliedPercent == 0.0f);
    CHECK(getMotorState(5)->appliedPercent == 2.0f); // 긴 지연 시 증가량 제한
    stopAllMotors(StopMode::Brake, 100);
    for (int i = 0; i < 6; ++i) {
        CHECK(getMotorDuty(i).in1Percent == 100 && getMotorDuty(i).in2Percent == 100);
        CHECK(getMotorState(i)->targetPowerPercent == 0);
    }
    config.motorCount = 0;
    CHECK(!initializeRobot(config, 0));
    CHECK(!setMotorPower(0, 10));
    config = testConfig();
    config.motors[0].kp = std::numeric_limits<float>::quiet_NaN();
    CHECK(!validateConfig(config));
    config = testConfig(); config.motors[0].motorSign = 0;
    CHECK(!validateConfig(config));
    config = testConfig(); config.encoderTimeoutMs = config.encoderPeriodMs;
    CHECK(!validateConfig(config));
    CHECK(initializeRobot(testConfig(), 0));
    config = testConfig(); config.stopMode = StopMode::Brake;
    CHECK(initializeRobot(config, 0));
    CHECK(getMotorDuty(0).in1Percent == 0 && getMotorDuty(0).in2Percent == 0);
}
// 램프·반전 대기·STOP 직후 반전의 동작을 검사하는 함수
void testRampAndReverse() {
    CHECK(initializeRobot(testConfig(), 0));
    CHECK(setMotorPower(0, 20));
    for (uint32_t now = 10; now <= 200; now += 10) updateRobot(now);
    CHECK(std::fabs(getMotorState(0)->appliedPercent - 20.0f) < 0.001f);
    CHECK(setMotorPower(0, -20));
    for (uint32_t now = 210; now <= 400; now += 10) updateRobot(now);
    CHECK(getMotorState(0)->appliedPercent == 0.0f);
    updateRobot(590);
    CHECK(getMotorState(0)->appliedPercent == 0.0f);
    updateRobot(600);
    CHECK(getMotorState(0)->appliedPercent < 0.0f);
    CHECK(stopMotor(0, StopMode::Coast, 601));
    CHECK(setMotorPower(0, 20));
    updateRobot(610);
    CHECK(getMotorState(0)->appliedPercent == 0.0f);
    updateRobot(810);
    CHECK(getMotorState(0)->appliedPercent > 0.0f);
    RobotConfig config = testConfig(); config.motors[0].motorSign = -1;
    CHECK(initializeRobot(config, 0));
    CHECK(setMotorPower(0, 100));
    for (uint32_t now = 10; now <= 1000; now += 10) updateRobot(now);
    CHECK(getMotorDuty(0).in1Percent == 0 && getMotorDuty(0).in2Percent == 40);
}
// 실제 경과 시간·음의 방향·분해능 미정·시간 래핑을 검사하는 함수
void testEncoder() {
    CHECK(initializeRobot(testConfig(), 0));
    double rpm = 123.0;
    CHECK(calculateRpm(280.0, 2800.0, 0.1, rpm) && std::fabs(rpm - 60.0) < 1e-9);
    CHECK(!calculateRpm(1, 0, 0.1, rpm));
    CHECK(!calculateRpm(1, 1000, 0, rpm));
    CHECK(!calculateRpm(std::numeric_limits<double>::infinity(), 1000, 1, rpm));
    CHECK(submitEncoderCount(0, 0, 0));
    CHECK(!isEncoderFresh(0, 0));
    CHECK(submitEncoderCount(0, 100, 200));
    CHECK(std::fabs(getEncoderState(0)->rpm - 30.0) < 1e-9);
    CHECK(submitEncoderCount(0, 0, 300));
    CHECK(std::fabs(getEncoderState(0)->rpm + 60.0) < 1e-9);
    CHECK(!isEncoderFresh(0, 600));
    CHECK(initializeRobot(testConfig(), 0));
    submitEncoderCount(0, std::numeric_limits<int64_t>::max() - 1, 0);
    submitEncoderCount(0, std::numeric_limits<int64_t>::max(), 100);
    CHECK(std::fabs(getEncoderState(0)->rpm - 0.6) < 1e-9);
    RobotConfig config = testConfig(); config.motors[0].encoderSign = -1;
    CHECK(initializeRobot(config, 0xfffffff0u));
    CHECK(submitEncoderCount(0, 0, 0xfffffff0u));
    CHECK(submitEncoderCount(0, 100, 84)); // 100ms 경과
    CHECK(std::fabs(getEncoderState(0)->rpm + 60.0) < 1e-9);
    CHECK(initializeRobot(RobotConfig{}, 0));
    submitEncoderCount(0, 0, 0); submitEncoderCount(0, 100, 100);
    CHECK(!getEncoderState(0)->rpmValid);
    CHECK(!setMotorRpm(0, 10, 100));
}
// 명령 파싱·입력 복구·타임아웃과 재시작 조건을 검사하는 함수
void testCommunication() {
    CHECK(initializeRobot(testConfig(), 0));
    Communication communication;
    std::string output;
    initializeCommunication(communication, collect, &output);
    feed(communication, "PWM 0 30\r\nPWM 1 -20\n", 0);
    CHECK(output == "OK CONTROL\nOK CONTROL\n");
    output.clear();
    feed(communication, "PWM 0 1x\nPWM 0 2 extra\nRPM 0 nan\nSTATUS extra\nSTOP bad\n", 400);
    CHECK(getMotorState(0)->targetPowerPercent == 30);
    CHECK(output.find("ERR") != std::string::npos);
    feed(communication, "STATUS\n", 499);
    updateRobot(500);
    CHECK(getMotorState(0)->mode == ControlMode::Stopped);
    CHECK(getMotorState(1)->fault == FaultTimeout);
    feed(communication, "PING\nSTATUS\n", 510);
    CHECK(getMotorState(0)->mode == ControlMode::Stopped);
    feed(communication, "PWM 0 10\n", 520);
    CHECK(getMotorState(0)->fault == FaultNone);
    CHECK(getMotorState(1)->mode == ControlMode::Stopped);
    feed(communication, "PING\n", 1000);
    updateRobot(1499);
    CHECK(getMotorState(0)->mode == ControlMode::Pwm);
    updateRobot(1500);
    CHECK(getMotorState(0)->mode == ControlMode::Stopped);
    output.clear();
    feed(communication, std::string(200, 'X') + "STOP\nPWM 0 20\n", 1600);
    CHECK(output == "ERR LINE_TOO_LONG\nOK CONTROL\n");
    output.clear();
    feed(communication, std::string("PWM 0 ") + char(0) + "50\nSTATUS\n", 1610);
    CHECK(output.find("ERR CHARACTER\n") == 0);
    CHECK(getMotorState(0)->targetPowerPercent == 20);
    feed(communication, "STOP BRAKE\n", 1620);
    CHECK(getMotorDuty(0).in1Percent == 100 && getMotorDuty(1).in2Percent == 100);
    output.clear();
    feed(communication, "STATUS\n", 1620);
    CHECK(output.find("count=") != std::string::npos && output.find("fault=") != std::string::npos);
    CHECK(initializeRobot(testConfig(), 0xfffffff0u));
    feed(communication, "PWM 0 20\n", 0xfffffff0u);
    updateRobot(483);
    CHECK(getMotorState(0)->mode == ControlMode::Pwm);
    updateRobot(484);
    CHECK(getMotorState(0)->mode == ControlMode::Stopped);
}
// PI 포화·적분 초기화·센서 갱신 중단과 가상 부하 추종을 검사하는 함수
void testSpeedControl() {
    RobotConfig config = testConfig(1);
    config.motors[0].outputLimitPercent = 80;
    config.motors[0].ki = 1.0f;
    config.motors[0].slewPercentPerSecond = 400.0f;
    CHECK(initializeRobot(config, 0));
    submitEncoderCount(0, 0, 0); submitEncoderCount(0, 0, 100);
    CHECK(!setMotorRpm(0, 301, 100));
    CHECK(setMotorRpm(0, 120, 100));
    double revolutions = 0.0;
    double measuredRpm = 0.0;
    for (uint32_t now = 110; now <= 15000; now += 10) {
        // 가상 모터의 시간 상수와 중간 부하 변화를 넣는다. 실물 모델은 아니다.
        const double gain = now < 7000 ? 3.0 : 2.0;
        measuredRpm += (getMotorState(0)->appliedPercent * gain - measuredRpm) * 0.01 / 0.15;
        revolutions += measuredRpm / 60.0 * 0.01;
        submitEncoderCount(0, static_cast<int64_t>(std::llround(revolutions * 1000.0)), now);
        updateRobot(now);
        CHECK(std::fabs(getMotorState(0)->appliedPercent) <= 80.0f);
    }
    CHECK(std::fabs(measuredRpm - 120.0) < 5.0);
    CHECK(setMotorRpm(0, 0, 15000));
    updateRobot(15010);
    CHECK(getMotorState(0)->requestedPercent == 0.0f);
    CHECK(setMotorRpm(0, 20, 15010));
    updateRobot(15300);
    CHECK(getMotorState(0)->fault == FaultEncoder);
    CHECK(getMotorState(0)->appliedPercent == 0.0f);
    CHECK(!setMotorPower(0, 10)); // 센서 오류는 새 명령으로 해제되지 않는다.
    CHECK(initializeRobot(config, 0));
    submitEncoderCount(0, 0, 0); submitEncoderCount(0, 0, 100);
    CHECK(setMotorRpm(0, 300, 100));
    for (uint32_t now = 110; now <= 2000; now += 10) {
        submitEncoderCount(0, 0, now); updateRobot(now);
    }
    CHECK(getMotorState(0)->requestedPercent == 80.0f);
    CHECK(setMotorRpm(0, 1, 2000));
    updateRobot(2010);
    CHECK(getMotorState(0)->requestedPercent < 5.0f); // 포화 중 적분이 크게 쌓이지 않는다.
}
}

// 전체 공통 로직 검증을 순서대로 실행하는 함수
int main() {
    testDuty(); testConfigAndState(); testRampAndReverse();
    testEncoder(); testCommunication(); testSpeedControl();
    std::printf("PASS: %d checks\n", checks);
    return 0;
}
