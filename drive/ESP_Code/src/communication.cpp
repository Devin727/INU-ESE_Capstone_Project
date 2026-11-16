#include "communication.h"
#include "controller.h"
#include "motor.h"
#include "encoder.h"
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
// 응답 한 줄을 등록된 출력 함수로 전달하는 함수
void reply(Communication& communication, const char* text) {
    if (communication.sink) communication.sink(text, communication.context);
}
// 정수 토큰 전체가 유효한 int 값인지 확인하는 함수
bool parseInt(const char* text, int& value) {
    errno = 0;
    char* end = nullptr;
    const long result = std::strtol(text, &end, 10);
    if (!text[0] || *end || errno == ERANGE || result < INT_MIN || result > INT_MAX) return false;
    value = static_cast<int>(result);
    return true;
}
// RPM 토큰을 유한한 실수로 변환하는 함수
bool parseFloat(const char* text, float& value) {
    errno = 0;
    char* end = nullptr;
    const float result = std::strtof(text, &end);
    if (!text[0] || *end || errno == ERANGE || !std::isfinite(result)) return false;
    value = result;
    return true;
}
// 제어 모드를 상태 전송용 문자열로 변환하는 함수
const char* modeName(ControlMode mode) {
    if (mode == ControlMode::Pwm) return "PWM";
    if (mode == ControlMode::Rpm) return "RPM";
    return "STOP";
}
}

// 수신 중인 줄을 비우고 응답 콜백을 연결하는 함수
void initializeCommunication(Communication& communication, ReplySink sink, void* context) {
    communication = Communication{};
    communication.sink = sink;
    communication.context = context;
}

// 명령을 토큰 단위로 검사한 뒤 유효한 요청만 실행하는 함수
bool executeCommand(Communication& communication, const char* line, uint32_t nowMs) {
    updateRobot(nowMs); // 기한이 지난 명령은 새 입력을 처리하기 전에 정지한다.
    if (!line || std::strlen(line) >= kCommandCapacity) {
        reply(communication, "ERR LINE_TOO_LONG\n");
        return false;
    }
    char copy[kCommandCapacity];
    std::memcpy(copy, line, std::strlen(line) + 1);
    char* tokens[4] = {};
    std::size_t count = 0;
    char* cursor = copy;
    while (*cursor) {
        while (*cursor == ' ' || *cursor == '\t') ++cursor;
        if (!*cursor) break;
        if (count == 4) { reply(communication, "ERR SYNTAX\n"); return false; }
        tokens[count++] = cursor;
        while (*cursor && *cursor != ' ' && *cursor != '\t') {
            if (static_cast<unsigned char>(*cursor) < 32 || static_cast<unsigned char>(*cursor) > 126) {
                reply(communication, "ERR CHARACTER\n"); return false;
            }
            ++cursor;
        }
        if (*cursor) *cursor++ = '\0';
    }
    if (count == 0) return false;
    if (std::strcmp(tokens[0], "STATUS") == 0 && count == 1) {
        sendStatus(communication, nowMs);
        return true;
    }
    if (std::strcmp(tokens[0], "PING") == 0 && count == 1) {
        noteHeartbeat(nowMs);
        reply(communication, "OK PING\n");
        return true;
    }
    if (std::strcmp(tokens[0], "STOP") == 0 && (count == 1 || count == 2)) {
        StopMode mode = getConfig().stopMode;
        if (count == 2) {
            if (std::strcmp(tokens[1], "COAST") == 0) mode = StopMode::Coast;
            else if (std::strcmp(tokens[1], "BRAKE") == 0) mode = StopMode::Brake;
            else { reply(communication, "ERR STOP_MODE\n"); return false; }
        }
        stopRobot(mode, nowMs);
        reply(communication, "OK STOP\n");
        return true;
    }
    if (count == 3 && (std::strcmp(tokens[0], "PWM") == 0 || std::strcmp(tokens[0], "RPM") == 0)) {
        int id = 0;
        if (!parseInt(tokens[1], id)) { reply(communication, "ERR MOTOR_ID\n"); return false; }
        bool accepted = false;
        if (std::strcmp(tokens[0], "PWM") == 0) {
            int percent = 0;
            accepted = parseInt(tokens[2], percent) && setMotorPower(id, percent);
        } else {
            float rpm = 0.0f;
            accepted = parseFloat(tokens[2], rpm) && setMotorRpm(id, rpm, nowMs);
        }
        if (!accepted) { reply(communication, "ERR REJECTED\n"); return false; }
        noteControlCommand(nowMs);
        reply(communication, "OK CONTROL\n");
        return true;
    }
    reply(communication, "ERR SYNTAX\n");
    return false;
}

// CR·LF와 긴 입력을 처리하여 완성된 명령 행만 실행하는 함수
void receiveCommandByte(Communication& communication, char byte, uint32_t nowMs) {
    if (byte == '\n' && communication.previousCr) {
        communication.previousCr = false;
        return;
    }
    communication.previousCr = byte == '\r';
    if (byte == '\r' || byte == '\n') {
        if (!communication.discarding && communication.length > 0) {
            communication.line[communication.length] = '\0';
            executeCommand(communication, communication.line, nowMs);
        }
        communication.length = 0;
        communication.discarding = false;
        return;
    }
    if (communication.discarding) return;
    const unsigned char code = static_cast<unsigned char>(byte);
    if ((code < 32 && byte != '\t') || code > 126) {
        communication.discarding = true;
        reply(communication, "ERR CHARACTER\n");
    } else if (communication.length + 1 >= kCommandCapacity) {
        communication.discarding = true;
        reply(communication, "ERR LINE_TOO_LONG\n");
    } else {
        communication.line[communication.length++] = byte;
    }
}

// 모터별 목표·카운트·RPM·실제 출력·오류를 상태 행으로 전송하는 함수
void sendStatus(Communication& communication, uint32_t nowMs) {
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        const int id = static_cast<int>(i);
        const MotorState& motor = *getMotorState(id);
        const EncoderState& encoder = *getEncoderState(id);
        const MotorDuty duty = getMotorDuty(id);
        char line[384];
        const int size = std::snprintf(line, sizeof(line),
            "STATE id=%u ms=%lu mode=%s target_pwm=%d target_rpm=%.3f count=%lld rpm=%.3f "
            "out=%.3f in1=%d in2=%d fault=%u rpm_valid=%d\n",
            static_cast<unsigned>(i), static_cast<unsigned long>(nowMs), modeName(motor.mode),
            motor.targetPowerPercent, static_cast<double>(motor.targetRpm),
            static_cast<long long>(encoder.count), encoder.rpm,
            static_cast<double>(motor.appliedPercent), duty.in1Percent, duty.in2Percent,
            motor.fault, isEncoderFresh(id, nowMs) ? 1 : 0);
        if (size > 0 && static_cast<std::size_t>(size) < sizeof(line)) reply(communication, line);
        else reply(communication, "ERR STATUS_OVERFLOW\n");
    }
}
