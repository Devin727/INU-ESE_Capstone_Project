#pragma once
#include "config.h"
// 모든 모터의 PI 제어기 상태를 초기화하는 함수
void initializeSpeedControl();
// 지정한 모터의 누적 적분을 초기화하는 함수
void resetSpeedController(int motorId);
// 측정 RPM과 목표 RPM의 오차로 모터 출력을 계산하는 함수
void updateSpeedControl(uint32_t nowMs, float elapsedSeconds);
