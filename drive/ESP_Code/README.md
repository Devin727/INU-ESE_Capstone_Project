# ESP32 모터·인코더 제어

배관 로봇의 바퀴 모터를 ESP32로 구동하고 인코더로 회전수와 RPM을 측정하는 하위 제어 코드다. 개발 범위는 `ESP32_DEVELOPMENT_PLAN.md`를 따른다.
외부에서 받은 PWM·RPM 명령을 실행하며, 배관 인식·경로 선택·관절 제어를 포함한 자율주행 알고리즘은 별도 범위다.

**PC 공통 로직은 빌드·검증했다. ESP32 보드, 부품 사양, 실제 핀은 미정이며 ESP32 대상 빌드·업로드·실물 시험은 아직 하지 않았다.**
Arduino CLI와 ESP32 코어는 이 작업 환경에서 확인되지 않았다. 부품 PDF도 작업 폴더에 없어 읽지 못했다.

## 폴더 구성

| 폴더 | 용도 | Git 업로드 |
|---|---|---|
| `src/`, `include/` | 제어 코드와 설정·함수 선언 | 포함 |
| `tools/` | PC·ESP32 빌드 스크립트와 시리얼 도구 | 포함 |
| `tests/`, `examples/` | 공통 로직 검증과 실행 예제 | 포함 |
| `docs/` | 실제 부품과 하드웨어 확인 사항 | 포함 |
| `build/` | 컴파일 결과와 Arduino 스케치 임시 복사본 | 제외, 빌드 시 재생성 |

저장소에서는 `drive/ESP_Code/`를 이 제어 코드 프로젝트의 루트로 사용한다. 아래 실행 명령은 모두 해당 폴더 안에서 실행한다.
`.gitignore`는 빌드 결과, Python 캐시, CSV 로그와 개인별 `include/board_config.local.h`를 제외한다.

## 구현 범위

| 파일 | 역할 |
|---|---|
| `include/config.h` | 모터 수, 분해능, 방향 부호, 출력 제한, 주기, PI 게인 |
| `src/motor.cpp` | 목표 저장, 출력 램프, 반전 대기, 코스트·브레이크, 오류 정지 |
| `src/encoder.cpp` | 누적 카운트 저장, 실제 경과 시간 기반 RPM, 측정 유효성 |
| `src/speed_control.cpp` | 모터별 PI 제어, 출력 포화 시 적분 누적 방지 |
| `src/controller.cpp` | 설정 검증, 제어 주기, 통신 타임아웃 |
| `src/communication.cpp` | 명령 해석, CR/LF 처리, 상태 응답 |
| `src/esp32_hardware.cpp` | Arduino-ESP32 3.x LEDC와 PCNT x4 연결 |
| `src/main.cpp` | 초기화, 제한된 시리얼 수신·송신, 주기별 실행 |
| `include/board_config.h` | 미정인 보드 설정 양식. 기본값은 하드웨어 비활성 |
| `docs/HARDWARE_CHECKLIST.md` | 제품 사양과 전원·GPIO·주변장치 자원 확인 기록 |
| `tools/native_sim.cpp` | 가상 시간과 카운트로 명령을 실행하는 PC 시뮬레이터 |
| `tools/serial_test.py` | USB 시리얼 명령 송수신, heartbeat, CSV 로그 |
| `tests/test_core.cpp` | 경계값, 시간 래핑, 반전, 통신, PI 추종 검증 |

서보·IMU·영상 처리·ROS 노드는 이번 바퀴 모터 개발 범위 밖이다. 상위 장치는 아래 명령 프로토콜로 연결할 수 있지만 실제 상위 장치와의 연동 시험은 남아 있다.

## 전체 동작 흐름

1. PC가 `PWM 0 30` 또는 `RPM 0 60`을 보낸다.
2. 통신 모듈이 모터 번호·범위·형식을 검사한다. 유효한 요청만 목표에 저장한다.
3. PWM 모드는 요청한 듀티를, RPM 모드는 PI가 계산한 듀티를 출력 목표로 사용한다.
4. 출력 상한과 변화율을 적용한다. 방향 반전은 먼저 0%로 내리고 대기 후 반대로 올라간다.
5. ESP32 연결부가 두 입력 듀티를 LEDC에 적용한다.
6. PCNT 카운트를 읽고 실제 경과 시간으로 RPM을 계산한다.
7. 상태 전송은 제어와 별도 주기로 실행한다. 유효 제어 명령이 끊기면 즉시 정지한다.

`setMotorPower()`는 목표를 저장하고, `updateRobot()`이 현재 출력을 갱신한다. 실제 핀 변경은 `applyHardwareOutputs()`가 수행한다.
`setMotorRpm()`은 분해능·최대 RPM·게인과 최신 측정값이 모두 준비되어야 성공한다.
반환값이 false인 요청은 기존 목표와 통신 감시 시간을 바꾸지 않는다.
직접 API를 사용할 경우 `updateRobot()`을 반복 호출해야 한다. 명령 타임아웃 감시는 통신 모듈이 `noteControlCommand()`로 시작하므로 직접 호출하는 제어 코드에서도 이 연결을 유지해야 한다.

## 단위와 설정

모터 번호는 `0 .. motorCount-1`이다. 고정 배열은 최대 6개까지 지원하지만 단일 ESP32에서 6개를 실제로 구동할 수 있다는 뜻은 아니다.

| 설정 | 의미 | 기본값 |
|---|---|---|
| `motorCount` | 활성 모터 수 | 1 |
| `outputLimitPercent` | 실제 구동 듀티 상한 | 20% |
| `slewPercentPerSecond` | 초당 출력 변화량 | 50%p/s |
| `controlPeriodMs` | PI·출력 갱신 간격 | 10ms |
| `encoderPeriodMs` | RPM 계산 간격 | 100ms |
| `statusPeriodMs` | 자동 상태 전송 간격 | 200ms |
| `commandTimeoutMs` | 유효 제어 명령 제한 시간 | 500ms |
| `encoderTimeoutMs` | RPM 측정 유효 시간 | 300ms |
| `reversalPauseMs` | 반전 전 0% 대기 시간 | 200ms |
| `stopMode` | STOP 기본값·타임아웃 정지 방식 | Coast |
| `motorSign`, `encoderSign` | 출력 방향·측정 RPM 부호 보정 | +1 |
| `countsPerOutputRev` | 현재 계수 방식의 출력축 1회전당 카운트 | 0: 미정 |
| `maxRpm`, `kp`, `ki` | 허용 목표 RPM, PI 게인 | 0: 미정 |

시간과 출력 관련 기본값은 준비용이며 실물에 대해 검증된 값이 아니다. 하드웨어는 기본적으로 비활성화되어 있다.
2800카운트, 160RPM, GPIO 번호를 실물 확정값으로 넣지 않았다.
PWM 듀티 30%는 최대 RPM의 30%를 의미하지 않는다.
PI 게인 단위는 `kp: %/RPM`, `ki: %/(RPM*s)`다. 미분항은 사용하지 않는다.

## 모터 출력과 정지

드라이버가 활성화됐다는 전제의 조합이다.

| 동작 | IN1 | IN2 |
|---|---|---|
| 양의 방향 | PWM | 0 |
| 음의 방향 | 0 | PWM |
| 코스트 | 0 | 0 |
| 브레이크 | 100% | 100% |

`motorSign`은 구동 방향에만 적용한다. 브레이크에는 구동 듀티 상한을 적용하지 않는다.
부팅 시에는 설정된 정지 방식과 무관하게 코스트로 시작한다.
`PWM 0 0` 또는 `RPM 0 0`은 점진적으로 출력을 낮춘다. `STOP`은 램프를 거치지 않고 전체 출력을 즉시 정지 조합으로 바꾼다.
0% 출력과 0% 대기는 축의 물리적인 정지를 보증하지 않는다.

## 인코더와 속도 제어

RPM은 `카운트 변화 / countsPerOutputRev × 60 / 실제 경과 초`로 계산한다.
처음 읽은 카운트를 기준점으로 잡고, 설정된 측정 간격이 지난 다음부터 RPM을 제공한다.
카운트가 그대로여도 새 스냅샷이 들어오면 유효한 0RPM 측정이다. 단선·정지·스톨을 이것만으로 구별하지 않는다.

PCNT는 A/B 양쪽 에지를 세는 x4 방식이다. 사양의 PPR/CPR·감속비와 축 기준을 확인해 `countsPerOutputRev`를 정해야 한다.
PCNT 하드웨어 경계 보정은 드라이버의 누적 기능과 ±30000 watch point를 사용한다.
스냅샷은 unsigned 차이로 확장해 64비트 누적값에 저장한다. 두 읽기 사이 변화량이 2^31카운트 미만이어야 한다.
실제 드라이버 누적 카운트의 장시간 래핑, 고속 누락, 잡음 필터 효과는 대상 칩에서 별도 시험해야 한다.

PI 제어에는 출력 제한과 적분 누적 방지가 있다. 정지·모드 전환·목표 방향 전환 시 적분을 초기화한다.
목표와 반대 방향의 제어 출력은 생성하지 않으며, 과속 시 해당 방향의 출력을 0까지 줄인다.
RPM 모드에서 카운트 읽기나 RPM 갱신이 중단되면 해당 모터에 센서 오류를 기록하고 정지한다.
RPM 필터링, 스톨 판정, 전류 측정은 구현하지 않았다. 필요한 기준과 센서가 아직 없다.

## 통신 프로토콜

ASCII 대문자 명령이며 공백·탭으로 구분한다. LF, CR, CRLF를 지원한다. 주석을 명령에 함께 보내면 안 된다.

```text
PWM 0 30
RPM 0 -60.5
STOP
STOP COAST
STOP BRAKE
STATUS
PING
```

- PWM은 -100~100 정수, RPM은 설정한 최대 RPM 이내의 유한한 실수다.
- `STOP`은 항상 모든 활성 모터에 적용된다.
- `PWM`, `RPM` 중 성공한 명령만 통신 감시를 시작·갱신한다.
- `PING`은 이미 감시 중일 때만 시간을 연장한다. 정지한 모터를 재시작하지 않는다.
- `STATUS`, 잘못된 명령, 미완성 행은 감시 시간을 연장하지 않는다.
- 감시는 연결 전체에 하나다. 어느 모터의 유효 제어 명령이든 전체 연결의 시간을 갱신한다.
- 줄바꿈 제외 최대 95바이트다. 초과 입력·잘못된 문자는 해당 행 끝까지 버리고 다음 행에서 복구한다.
- 시간 비교는 uint32_t 차이를 사용하므로 `millis()` 래핑을 처리한다. 루프는 계속 실행되어야 한다.

응답은 `OK CONTROL`, `OK STOP`, `OK PING` 또는 `ERR ...` 형식이다.

상태 행 예시:

```text
STATE id=0 ms=200 mode=PWM target_pwm=30 target_rpm=0.000 count=0 rpm=0.000 out=10.000 in1=10 in2=0 fault=0 rpm_valid=0
```

`out`은 방향 보정 전 현재 구동 듀티(%)이고 `in1/in2`는 방향 보정과 정지 방식을 반영한 출력 요청이다.
`count`는 원시 누적 카운트, `rpm`은 `encoderSign`을 반영한 RPM이다.
`rpm_valid=0`이면 표시된 RPM을 현재 측정값으로 사용하면 안 된다.
하드웨어가 비활성인 PC에서는 실제 핀 출력이 아니라 계산 상태를 보여 준다.

오류 비트는 `1=통신 타임아웃`, `2=RPM 센서 갱신 중단`, `4=설정·하드웨어 오류`다.
타임아웃 후에는 새 PWM/RPM 명령을 받아야 재개한다. 센서·하드웨어 오류는 원인 확인 후 재부팅해야 한다.
하드웨어 오류 시에는 코스트 출력과 가능한 nSLEEP 비활성화를 적용한다.

송신 큐는 고정 크기이며 전송이 밀리면 메시지를 버리고 `WARN TX_DROPPED=...`로 알린다.
누락된 응답을 성공으로 간주하지 말고 상태와 연결을 확인한다.

## PC 빌드와 검증

Windows의 Visual Studio 2022 C++ 도구가 필요하다. 프로젝트 폴더에서 실행한다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build_native.ps1
```

공통 모듈을 `/W4 /WX`로 컴파일한 뒤 전체 검증과 기존 예제를 실행하고 `build/native_sim.exe`를 생성한다.
CMake가 설치되어 있다면 `cmake -S . -B build/cmake`, `cmake --build build/cmake --config Debug`, `ctest --test-dir build/cmake -C Debug --output-on-failure`로도 빌드하도록 구성했다. 이 환경에서는 MSVC 스크립트 경로를 검증했다.

시뮬레이터 예시:

```powershell
@'
PWM 0 30
TICK 200
STATUS
PWM 0 -30
TICK 300
PING
TICK 200
STATUS
STOP BRAKE
STATUS
'@ | .\build\native_sim.exe
```

`TICK 200`은 대기하지 않고 가상 시간을 200ms 진행한다. `COUNT 0 100`은 원시 누적 카운트를 100으로 설정한다.
시뮬레이터는 2개 가상 모터, 1000카운트/회전, 300RPM, 임의 게인을 사용한다. 실제 사양이 아니다.
시뮬레이터는 모터가 저절로 회전하는 물리 모델을 포함하지 않는다. 카운트는 직접 주입한다.
PI 부하 추종 시험용 간단한 가상 모델은 `tests/test_core.cpp`에 별도로 있다.

검증 항목은 모든 정수 듀티/상한 조합, 6개 독립 상태, 반전 대기, 즉시 정지, 잘못된 설정·명령,
긴 입력 후 복구, uint32_t 시간 래핑, 큰 누적 카운트에서의 작은 변화, RPM 계산·오래된 측정,
타임아웃 복구 조건, PI 포화와 가상 부하 변화다. 이것은 실물 성능 검증을 대신하지 않는다.

## ESP32 설정과 빌드

모터 1개의 개념 연결은 다음과 같다. 실제 단자 이름은 구매한 모듈에서 확인한다.

| 연결 | 역할 |
|---|---|
| ESP32 IN1/IN2용 GPIO → 드라이버 AIN1/AIN2 | 방향·PWM 제어 |
| 드라이버 AOUT1/AOUT2 → 모터 전원선 2개 | 모터 구동 |
| 적정 모터 전원 → 드라이버 VM/GND | 모터용 전력 공급 |
| ESP32 GND ↔ 드라이버 GND | 신호 기준 전위 |
| 인코더 A/B → 확인한 입력 GPIO | 회전 카운트 |
| 인코더 전원/GND → 제품 사양에 맞는 전원 | 센서 전력 공급 |
| nSLEEP → 공통 제어 GPIO 또는 확인한 외부 활성화 회로 | 드라이버 활성화 |

1. 실제 보드·칩·핀맵과 모터·드라이버·전원 사양을 확인한다.
2. `include/board_config.h`를 `include/board_config.local.h`로 복사한다.
3. `makeBoardConfig()`에서 모터 수, 출력 제한, 부호, 타이밍을 설정한다. RPM을 쓸 경우 실제 x4 출력축 카운트, 최대 RPM과 시험용 PI 게인을 설정한다.
4. `makeBoardSettings()`에서 IN1/IN2, 인코더 핀, PWM 주파수·해상도를 설정한다. nSLEEP GPIO를 지정하거나 실제 외부 활성화 회로를 확인한 뒤 `externalSleepConfirmed=true`로 설정한다.
5. 전원과 회로를 확인한 다음 `hardwareEnabled=true`로 설정한다. 서보·통신·부트·플래시·USB 용도로 예약된 핀을 피하는 것은 보드 핀맵과 함께 검토한다.
6. Arduino CLI와 선택한 버전의 Arduino-ESP32 **3.x** 코어를 설치하고 버전을 기록한다. 이 연결부는 2.x LEDC API를 지원하지 않는다.

```powershell
.\tools\build_esp32.ps1 -Fqbn '실제_보드의_FQBN'
.\tools\build_esp32.ps1 -Fqbn '실제_보드의_FQBN' -Upload -Port COM5
```

빌드 스크립트는 `build/arduino/ESP_Code`에 스케치 복사본을 만든다. 원본은 `src/`, `include/`에서 수정한다.
FQBN과 COM5는 실제 장치에 맞게 지정한다. 코어 버전은 보드 확인 전까지 확정하지 않았다.
설정이 비어 있으면 GPIO를 구동하지 않고 오류 상태로 시작한다. STATUS·STOP·PING 진단만 가능하다.

LEDC는 모터당 2채널, PCNT는 인코더당 1유닛·2채널을 할당한다. 할당 실패 시 출력을 비활성화한다.
PCNT가 없는 칩에서 인코더를 활성화하면 초기화가 실패한다. 그 칩에서는 별도 인터럽트 백엔드가 필요하다.
6개 모터 외에도 서보 7개·센서·통신의 GPIO/PWM 자원을 함께 계산해야 하며, 필요하면 제어 보드를 분리한다.
내부 풀업 사용 여부와 글리치 필터는 인코더 출력 회로·펄스 폭에 맞춰 정한다. 기본 글리치 필터는 비활성이다.
초기화 실패 후 자원 재할당·자동 재시도는 하지 않으며 재부팅을 요구한다.

## USB 시리얼 도구

```powershell
py -m pip install -r .\tools\requirements.txt
py .\tools\serial_test.py --port COM5 --csv run.csv
```

기본값은 heartbeat 없음이다. 따라서 한 번 PWM을 보낸 뒤 명령을 갱신하지 않으면 설정된 시간 후 정지한다.
연속 시험 시에만 `--heartbeat-ms 200`처럼 타임아웃보다 짧은 간격을 명시한다.
명령으로 `QUIT`을 입력하거나 Ctrl+C를 누르면 STOP을 전송하고 종료한다. 로그 파일이 이미 있으면 덮어쓰지 않는다.
CSV의 `pc_monotonic_s`는 PC가 수신한 단조 시간, `ms`는 ESP32의 부팅 이후 시간이다.
실제 시리얼 연결 시험과 pyserial 연결은 아직 수행하지 않았다.

## 남은 실물 확인

| 계획 단계 | 현재 상태 |
|---|---|
| 0: 부품·전원 확인 | 사양 미정, 미완료 |
| 1: 환경 | PC 빌드 완료, ESP32 대상 빌드·업로드 미확인 |
| 2: PWM | 공통 로직 검증, LEDC 연결 작성, 실제 양방향·정지 미확인 |
| 3: 카운트 | PCNT 연결 작성, 실제 부호·분해능·누락·래핑 미확인 |
| 4: RPM | 가상 카운트 계산 검증, 실측 비교 미확인 |
| 5: 통신 | 파서·타임아웃 검증, USB 연결 미확인 |
| 6: 속도 제어 | PI·가상 부하 검증, 실물 게인 조정 미완료 |
| 7: 확장 | 최대 6개 소프트웨어 상태 검증, 동시 전원·자원·속도 미확인 |
| 8: 상위 연동 | ASCII 프로토콜 제공, 상위 장치·통신 확정과 연동 시험 미완료 |

ESP32 GND와 드라이버 GND를 공통 기준으로 연결하고, 모터 전력은 별도 적정 전원에서 드라이버를 통해 공급한다.
인코더 전원·출력 전압이 ESP32 입력 조건에 맞는지 확인한다. 2S 배터리를 정격 6V 모터에 그대로 연결한다고 가정하지 않는다.
낮은 출력에서 방향·코스트·브레이크를 확인하고 전류·발열·전압 강하를 측정해야 한다. 소프트웨어 STOP은 물리적인 전원 차단을 대신하지 않는다.

## 공식 참고 자료

- [TI DRV8833 데이터시트](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [Arduino-ESP32 LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [ESP-IDF PCNT API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html)

보드를 선택한 뒤 설치한 Arduino 코어·ESP-IDF 버전의 문서와 다시 대조해야 한다.
