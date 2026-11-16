#pragma once
#include "esp32_hardware.h"

// 부품 사양이 확정된 뒤 사용할 제어 설정을 만드는 함수
inline RobotConfig makeBoardConfig() {
    RobotConfig config;
    // 초기값은 준비용이다. 모터 수·출력 제한·주기·분해능·게인을 확인 후 변경한다.
    // countsPerOutputRev는 현재 PCNT x4 계수 방식의 출력축 1회전당 카운트다.
    return config;
}

// 확인한 GPIO와 전원 회로에 맞는 보드 설정을 만드는 함수
inline BoardSettings makeBoardSettings() {
    BoardSettings settings;
    // 기본 상태는 비활성화다. -1 핀이나 PWM 주파수 0을 실물에 적용하지 않는다.
    // 이 파일을 board_config.local.h로 복사한 뒤 사양을 기록하고 수정한다.
    // settings.hardwareEnabled = true;
    // settings.motors[0].in1 = 확인한_GPIO;
    // settings.motors[0].in2 = 확인한_GPIO;
    // settings.sleepPin = 확인한_GPIO;
    // settings.pwmFrequencyHz = 확인한_주파수;
    return settings;
}
