#include "communication.h"
#include "controller.h"
#include "encoder.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

// PC에서 응답을 표준 출력으로 전달하는 함수
bool printReply(const char* text, void*) { std::cout << text; return true; }

// 가상 시간과 카운트로 펌웨어 공통 로직을 실행하는 함수
int main() {
    RobotConfig config;
    config.motorCount = 2;
    // 시뮬레이터 전용 임의 사양이다. 실제 모터 설정에 복사하지 않는다.
    for (std::size_t i = 0; i < config.motorCount; ++i) {
        config.motors[i].countsPerOutputRev = 1000.0;
        config.motors[i].maxRpm = 300.0f;
        config.motors[i].kp = 0.5f;
        config.motors[i].ki = 0.2f;
    }
    if (!initializeRobot(config, 0)) return 1;
    Communication communication;
    initializeCommunication(communication, printReply, nullptr);
    uint32_t now = 0;
    int64_t counts[kMaxMotors] = {};
    for (std::size_t i = 0; i < config.motorCount; ++i) submitEncoderCount(static_cast<int>(i), 0, now);
    std::cout << "SIMULATION ONLY: TICK <ms>, COUNT <id> <absolute_count>, PWM/RPM/STOP/STATUS/PING\n";
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command == "TICK") {
            unsigned duration = 0;
            if (!(input >> duration) || duration > 60000) { std::cout << "ERR TICK\n"; continue; }
            for (unsigned elapsed = 0; elapsed < duration; ++elapsed) {
                ++now;
                for (std::size_t i = 0; i < config.motorCount; ++i)
                    submitEncoderCount(static_cast<int>(i), counts[i], now);
                updateRobot(now);
            }
        } else if (command == "COUNT") {
            int id = -1;
            int64_t count = 0;
            if (!(input >> id >> count) || id < 0 || static_cast<std::size_t>(id) >= config.motorCount) {
                std::cout << "ERR COUNT\n"; continue;
            }
            counts[id] = count;
            submitEncoderCount(id, count, now);
        } else {
            for (char byte : line) receiveCommandByte(communication, byte, now);
            receiveCommandByte(communication, '\n', now);
        }
    }
    return 0;
}
