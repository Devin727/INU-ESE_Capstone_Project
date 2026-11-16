#pragma once
#include "config.h"

using ReplySink = bool (*)(const char* text, void* context);
struct Communication {
    char line[kCommandCapacity] = {};
    std::size_t length = 0;
    bool discarding = false;
    bool previousCr = false;
    ReplySink sink = nullptr;
    void* context = nullptr;
};
// 수신 버퍼와 응답 전달 함수를 초기화하는 함수
void initializeCommunication(Communication& communication, ReplySink sink, void* context);
// 한 글자씩 수신하여 완성된 명령을 처리하는 함수
void receiveCommandByte(Communication& communication, char byte, uint32_t nowMs);
// 완성된 명령 한 줄을 검사하고 실행하는 함수
bool executeCommand(Communication& communication, const char* line, uint32_t nowMs);
// 모든 모터의 시간과 목표값 및 측정 상태를 전송하는 함수
void sendStatus(Communication& communication, uint32_t nowMs);
