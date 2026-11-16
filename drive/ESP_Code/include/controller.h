#pragma once
#include "config.h"
// 설정을 검증하고 모든 제어 상태를 정지 상태로 초기화하는 함수
bool initializeRobot(const RobotConfig& config, uint32_t nowMs);
// 제어 명령 수신 시 통신 타임아웃 기준 시간을 갱신하는 함수
void noteControlCommand(uint32_t nowMs);
// 유효한 heartbeat 수신 시 활성 상태의 타임아웃을 연장하는 함수
void noteHeartbeat(uint32_t nowMs);
// 모든 모터를 정지하고 통신 감시를 비활성화하는 함수
void stopRobot(StopMode mode, uint32_t nowMs);
// 타임아웃과 제어 주기를 확인하고 출력 상태를 갱신하는 함수
void updateRobot(uint32_t nowMs);
