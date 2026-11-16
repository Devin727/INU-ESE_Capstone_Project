#ifdef ARDUINO
#include "esp32_hardware.h"
#include "motor.h"
#include "encoder.h"
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <driver/gpio.h>
#include <soc/soc_caps.h>
#include <limits>
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "This adapter requires Arduino-ESP32 3.x LEDC and ESP-IDF 5 PCNT APIs."
#endif
#if SOC_PCNT_SUPPORTED
#include <driver/pulse_cnt.h>
#endif

namespace {
BoardSettings board;
bool ready = false;
bool sleepConfigured = false;
bool attached[kMaxMotors][2] = {};
int previousDuty[kMaxMotors][2] = {};
#if SOC_PCNT_SUPPORTED
pcnt_unit_handle_t units[kMaxMotors] = {};
pcnt_channel_handle_t channels[kMaxMotors][2] = {};
uint32_t previousRaw[kMaxMotors] = {};
int64_t totalCounts[kMaxMotors] = {};
#endif

// GPIO 방향·중복·필수 설정을 출력 시작 전에 검사하는 함수
bool validateBoard(const BoardSettings& settings) {
    if (!settings.hardwareEnabled || settings.pwmFrequencyHz == 0 ||
        settings.pwmResolutionBits == 0 || settings.pwmResolutionBits > 14 ||
        (settings.sleepPin < 0 && !settings.externalSleepConfirmed)) return false;
    int used[kMaxMotors * 4 + 2];
    std::size_t count = 0;
    if (settings.sleepPin >= 0) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(settings.sleepPin)) return false;
        used[count++] = settings.sleepPin;
    }
    if (settings.faultPin >= 0) {
        if (!GPIO_IS_VALID_GPIO(settings.faultPin)) return false;
        used[count++] = settings.faultPin;
    }
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        const MotorPins& pins = settings.motors[i];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins.in1) || !GPIO_IS_VALID_OUTPUT_GPIO(pins.in2)) return false;
        used[count++] = pins.in1; used[count++] = pins.in2;
        if (getConfig().motors[i].countsPerOutputRev > 0.0 && !pins.encoderEnabled) return false;
        if (pins.encoderEnabled) {
            if (!GPIO_IS_VALID_GPIO(pins.encoderA) || !GPIO_IS_VALID_GPIO(pins.encoderB)) return false;
            used[count++] = pins.encoderA; used[count++] = pins.encoderB;
        }
    }
    for (std::size_t i = 0; i < count; ++i)
        for (std::size_t j = i + 1; j < count; ++j) if (used[i] == used[j]) return false;
    return true;
}

#if SOC_PCNT_SUPPORTED
// A/B 양쪽 에지를 세는 PCNT x4 디코더를 설정하는 함수
bool setupPcnt(std::size_t id) {
    const MotorPins& pins = board.motors[id];
    pcnt_unit_config_t unitConfig = {};
    unitConfig.high_limit = 30000;
    unitConfig.low_limit = -30000;
    unitConfig.flags.accum_count = true;
    if (pcnt_new_unit(&unitConfig, &units[id]) != ESP_OK) return false;
    if (board.encoderGlitchFilterNs > 0) {
        pcnt_glitch_filter_config_t filter = {};
        filter.max_glitch_ns = board.encoderGlitchFilterNs;
        if (pcnt_unit_set_glitch_filter(units[id], &filter) != ESP_OK) return false;
    }
    pcnt_chan_config_t channelConfig = {};
    channelConfig.edge_gpio_num = pins.encoderA;
    channelConfig.level_gpio_num = pins.encoderB;
    if (pcnt_new_channel(units[id], &channelConfig, &channels[id][0]) != ESP_OK) return false;
    channelConfig.edge_gpio_num = pins.encoderB;
    channelConfig.level_gpio_num = pins.encoderA;
    if (pcnt_new_channel(units[id], &channelConfig, &channels[id][1]) != ESP_OK) return false;
    if (pcnt_channel_set_edge_action(channels[id][0], PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                    PCNT_CHANNEL_EDGE_ACTION_INCREASE) != ESP_OK ||
        pcnt_channel_set_level_action(channels[id][0], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                     PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK ||
        pcnt_channel_set_edge_action(channels[id][1], PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                    PCNT_CHANNEL_EDGE_ACTION_DECREASE) != ESP_OK ||
        pcnt_channel_set_level_action(channels[id][1], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                     PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) return false;
    // 하드웨어 카운터가 경계에서 리셋될 때 드라이버 누적 카운트가 보정되도록 한다.
    if (pcnt_unit_add_watch_point(units[id], 30000) != ESP_OK ||
        pcnt_unit_add_watch_point(units[id], -30000) != ESP_OK ||
        pcnt_unit_enable(units[id]) != ESP_OK || pcnt_unit_clear_count(units[id]) != ESP_OK ||
        pcnt_unit_start(units[id]) != ESP_OK) return false;
    return true;
}
#endif
}

// 오류 시 모든 모터를 정지하고 실제 드라이버 출력을 끄는 함수
void shutdownHardware(uint32_t nowMs) {
    ready = false;
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        latchMotorFault(static_cast<int>(i), FaultHardware, nowMs);
        stopMotor(static_cast<int>(i), StopMode::Coast, nowMs);
        if (attached[i][0]) ledcWrite(board.motors[i].in1, 0);
        if (attached[i][1]) ledcWrite(board.motors[i].in2, 0);
    }
    if (sleepConfigured) digitalWrite(board.sleepPin, LOW);
}

// LEDC와 PCNT 자원을 할당하고 드라이버를 활성화하는 함수
bool initializeHardware(const BoardSettings& settings, uint32_t nowMs) {
    // 이 초기화는 부팅 시 한 번 호출한다. 실패 후에는 설정 수정과 재부팅이 필요하다.
    if (!validateBoard(settings)) { shutdownHardware(nowMs); return false; }
    board = settings;
    if (board.sleepPin >= 0) {
        if (gpio_set_level(static_cast<gpio_num_t>(board.sleepPin), 0) != ESP_OK) {
            shutdownHardware(nowMs); return false;
        }
        pinMode(board.sleepPin, OUTPUT);
        digitalWrite(board.sleepPin, LOW);
        sleepConfigured = true;
    }
    if (board.faultPin >= 0) pinMode(board.faultPin, INPUT_PULLUP);
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        const MotorPins& pins = board.motors[i];
        if (gpio_set_level(static_cast<gpio_num_t>(pins.in1), 0) != ESP_OK ||
            gpio_set_level(static_cast<gpio_num_t>(pins.in2), 0) != ESP_OK) {
            shutdownHardware(nowMs); return false;
        }
        pinMode(pins.in1, OUTPUT); pinMode(pins.in2, OUTPUT);
        attached[i][0] = ledcAttach(pins.in1, board.pwmFrequencyHz, board.pwmResolutionBits);
        if (!attached[i][0]) { shutdownHardware(nowMs); return false; }
        attached[i][1] = ledcAttach(pins.in2, board.pwmFrequencyHz, board.pwmResolutionBits);
        if (!attached[i][1] || !ledcWrite(pins.in1, 0) || !ledcWrite(pins.in2, 0)) {
            shutdownHardware(nowMs); return false;
        }
        if (pins.encoderEnabled) {
            pinMode(pins.encoderA, pins.encoderPullup ? INPUT_PULLUP : INPUT);
            pinMode(pins.encoderB, pins.encoderPullup ? INPUT_PULLUP : INPUT);
#if SOC_PCNT_SUPPORTED
            if (!setupPcnt(i)) { shutdownHardware(nowMs); return false; }
#else
            shutdownHardware(nowMs); return false; // PCNT가 없는 칩에서는 다른 백엔드가 필요하다.
#endif
        }
    }
    if (sleepConfigured) { digitalWrite(board.sleepPin, HIGH); delay(2); }
    ready = true;
    return readHardwareEncoders(millis());
}

// PCNT 스냅샷의 32비트 래핑을 확장하여 64비트 누적 카운트로 전달하는 함수
bool readHardwareEncoders(uint32_t nowMs) {
    if (!ready) return false;
    if (board.faultPin >= 0 && digitalRead(board.faultPin) == LOW) {
        shutdownHardware(nowMs); return false;
    }
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        if (!board.motors[i].encoderEnabled) continue;
#if SOC_PCNT_SUPPORTED
        int raw = 0;
        if (pcnt_unit_get_count(units[i], &raw) != ESP_OK) { shutdownHardware(nowMs); return false; }
        const uint32_t bits = static_cast<uint32_t>(raw);
        const uint32_t difference = bits - previousRaw[i];
        const int64_t delta = difference <= 0x7fffffffu ? static_cast<int64_t>(difference)
            : static_cast<int64_t>(difference) - 0x100000000LL;
        if ((delta > 0 && totalCounts[i] > std::numeric_limits<int64_t>::max() - delta) ||
            (delta < 0 && totalCounts[i] < std::numeric_limits<int64_t>::min() - delta)) {
            shutdownHardware(nowMs); return false;
        }
        previousRaw[i] = bits;
        totalCounts[i] += delta;
        submitEncoderCount(static_cast<int>(i), totalCounts[i], nowMs);
#else
        shutdownHardware(nowMs); return false;
#endif
    }
    return true;
}

// 두 입력을 먼저 0으로 만든 뒤 변경된 듀티만 LEDC에 적용하는 함수
bool applyHardwareOutputs(uint32_t nowMs) {
    if (!ready) return false;
    const uint32_t maximum = (1u << board.pwmResolutionBits) - 1;
    for (std::size_t i = 0; i < getConfig().motorCount; ++i) {
        const MotorDuty duty = getMotorDuty(static_cast<int>(i));
        if (!duty.valid) { shutdownHardware(nowMs); return false; }
        if (previousDuty[i][0] == duty.in1Percent && previousDuty[i][1] == duty.in2Percent) continue;
        const MotorPins& pins = board.motors[i];
        if (!ledcWrite(pins.in1, 0) || !ledcWrite(pins.in2, 0) ||
            !ledcWrite(pins.in1, (maximum * static_cast<uint32_t>(duty.in1Percent) + 50u) / 100u) ||
            !ledcWrite(pins.in2, (maximum * static_cast<uint32_t>(duty.in2Percent) + 50u) / 100u)) {
            shutdownHardware(nowMs); return false;
        }
        previousDuty[i][0] = duty.in1Percent; previousDuty[i][1] = duty.in2Percent;
    }
    return true;
}
#endif
