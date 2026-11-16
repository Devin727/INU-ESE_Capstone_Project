#ifdef ARDUINO
#include <Arduino.h>
#include "communication.h"
#include "controller.h"
#include "esp32_hardware.h"
#if __has_include("board_config.local.h")
#include "board_config.local.h"
#else
#include "board_config.h"
#endif
#include <cstring>
#include <cstdio>

namespace {
Communication communication;
bool hardwareReady = false;
uint32_t lastStatusMs = 0;
char transmitBuffer[4096];
std::size_t head = 0;
std::size_t tail = 0;
uint32_t droppedMessages = 0;

// 응답을 고정 크기 큐에 넣어 시리얼 출력이 제어를 막지 않게 하는 함수
bool enqueueReply(const char* text, void*) {
    const std::size_t length = std::strlen(text);
    const std::size_t free = (tail + sizeof(transmitBuffer) - head - 1) % sizeof(transmitBuffer);
    if (length > free) { ++droppedMessages; return false; }
    for (std::size_t i = 0; i < length; ++i) {
        transmitBuffer[head] = text[i]; head = (head + 1) % sizeof(transmitBuffer);
    }
    return true;
}

// 전송 가능한 공간만 사용해 큐의 데이터를 조금씩 내보내는 함수
void flushReplies() {
    int budget = Serial.availableForWrite();
    if (budget > 128) budget = 128;
    while (tail != head && budget-- > 0) {
        if (Serial.write(static_cast<uint8_t>(transmitBuffer[tail])) != 1) break;
        tail = (tail + 1) % sizeof(transmitBuffer);
    }
}
}

// 통신과 제어 상태를 초기화하고 검증된 설정으로 하드웨어를 준비하는 함수
void setup() {
    Serial.begin(115200);
    const uint32_t now = millis();
    initializeCommunication(communication, enqueueReply, nullptr);
    if (initializeRobot(makeBoardConfig(), now)) hardwareReady = initializeHardware(makeBoardSettings(), now);
    enqueueReply(hardwareReady ? "READY\n" : "ERR HARDWARE_NOT_CONFIGURED_OR_FAILED\n", nullptr);
    lastStatusMs = millis();
}

// 수신·제어·상태 전송을 서로 다른 주기와 제한된 작업량으로 실행하는 함수
void loop() {
    uint32_t now = millis();
    if (hardwareReady) hardwareReady = readHardwareEncoders(now);
    updateRobot(now);
    if (hardwareReady) hardwareReady = applyHardwareOutputs(now);
    // 입력이 계속 들어와도 제어 작업으로 돌아올 수 있도록 글자 수를 제한한다.
    for (unsigned budget = 0; budget < 64 && Serial.available() > 0; ++budget) {
        now = millis();
        receiveCommandByte(communication, static_cast<char>(Serial.read()), now);
        if (hardwareReady) hardwareReady = applyHardwareOutputs(now);
    }
    now = millis();
    if (static_cast<uint32_t>(now - lastStatusMs) >= getConfig().statusPeriodMs) {
        lastStatusMs = now;
        sendStatus(communication, now);
        if (droppedMessages) {
            char text[64];
            std::snprintf(text, sizeof(text), "WARN TX_DROPPED=%lu\n", static_cast<unsigned long>(droppedMessages));
            if (enqueueReply(text, nullptr)) droppedMessages = 0;
        }
    }
    flushReplies();
}
#endif
