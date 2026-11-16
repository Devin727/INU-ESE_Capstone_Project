#include <cstdio>

#include "motor.h"

// 듀티 계산과 목표 출력 저장 동작을 확인하는 PC용 예제
int main() {
    struct Example {
        int percent;
        int limitPercent;
        bool valid;
        int in1Percent;
        int in2Percent;
    };

    const Example examples[] = {
        {30, 20, true, 20, 0},
        {-30, 20, true, 0, 20},
        {0, 20, true, 0, 0},
        {30, 0, true, 0, 0},
        {100, 100, true, 100, 0},
        {-100, 100, true, 0, 100},
        {101, 20, false, 0, 0},
        {-101, 20, false, 0, 0},
        {30, -1, false, 0, 0},
        {30, 101, false, 0, 0},
    };

    for (const Example& example : examples) {
        const MotorDuty duty = calculateMotorDuty(example.percent, example.limitPercent);
        if (duty.valid != example.valid ||
            duty.in1Percent != example.in1Percent ||
            duty.in2Percent != example.in2Percent) {
            std::printf("FAIL: request=%d limit=%d\n", example.percent, example.limitPercent);
            return 1;
        }

        std::printf("OK: request=%d limit=%d valid=%d IN1=%d IN2=%d\n",
                    example.percent, example.limitPercent, duty.valid,
                    duty.in1Percent, duty.in2Percent);
    }

    int target = -1;
    if (!getMotorTargetPower(0, target) || target != 0) {
        std::printf("FAIL: initial target must be zero\n");
        return 1;
    }

    struct PowerExample {
        int motorId;
        int percent;
        bool accepted;
        int expectedTarget;
    };

    const PowerExample powerExamples[] = {
        {0, 30, true, 30},
        {1, 50, false, 30},
        {-1, 50, false, 30},
        {0, 101, false, 30},
        {0, -101, false, 30},
        {0, -30, true, -30},
        {0, 100, true, 100},
        {0, -100, true, -100},
        {0, 0, true, 0},
    };

    for (const PowerExample& example : powerExamples) {
        const bool accepted = setMotorPower(example.motorId, example.percent);
        if (accepted != example.accepted ||
            !getMotorTargetPower(0, target) || target != example.expectedTarget) {
            std::printf("FAIL: motor=%d request=%d\n", example.motorId, example.percent);
            return 1;
        }

        std::printf("OK: motor=%d request=%d accepted=%d target=%d\n",
                    example.motorId, example.percent, accepted, target);
    }

    // 읽기에 실패하면 호출한 쪽의 변수가 바뀌지 않는지도 확인한다.
    target = 42;
    if (getMotorTargetPower(1, target) ||
        getMotorTargetPower(-1, target) || target != 42) {
        std::printf("FAIL: invalid read must preserve the destination\n");
        return 1;
    }

    std::printf("OK: initial target and invalid reads\n");
    return 0;
}
