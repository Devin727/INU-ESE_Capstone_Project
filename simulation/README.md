# 배관 검사 로봇 시뮬레이션

100mm PVC 배관 안을 주행하며 초음파(UT)로 벽 두께와 균열을 검사하는 로봇의 **설계 검증용 시뮬레이션**입니다. 실제 로봇을 만들기 전에 컴퓨터 안에서 먼저 다음을 시험합니다.

- 차체 구조가 배관 엘보(45°, 90°, 위·아래 방향 포함)를 통과하는지
- 바퀴가 벽을 누르는 힘, 스프링, 관절 서보가 적당한지
- 바퀴 미끄러짐과 몸통 비틀림(롤)이 얼마나 되는지
- 엔코더와 IMU로 추정한 위치가 얼마나 정확한지
- 부품이 몸통 안에 실제로 들어가는지

| 항목 | 내용 |
|---|---|
| 프로젝트 | 황제펭귄 살리기 (100mm PVC 배관 검사 로봇) |
| 환경 | Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, Python 3.10 |

---

## 목차

1. [처음 시작하기](#1-처음-시작하기)
2. [무엇을 시뮬레이션하나](#2-무엇을-시뮬레이션하나)
3. [차체 모델](#3-차체-모델)
4. [배관 코스와 엘보](#4-배관-코스와-엘보)
5. [시뮬 실행 방법](#5-시뮬-실행-방법)
6. [노드와 토픽](#6-노드와-토픽)
7. [벤치마크 (설계 비교)](#7-벤치마크-설계-비교)
8. [부품 배치 검사](#8-부품-배치-검사)
9. [폴더 구조](#9-폴더-구조)
10. [현재 상태와 결과](#10-현재-상태와-결과)
11. [알려진 문제](#11-알려진-문제)
12. [새로 추가하기](#12-새로-추가하기)
13. [문서 안내](#13-문서-안내)
14. [환경과 설치](#14-환경과-설치)

---

## 1. 처음 시작하기

**ROS 2나 Gazebo를 처음 보는 분은 [docs/00 입문 문서](docs/00_입문_먼저_읽기.md)부터 읽으세요.** 개념과 도구 사용법을 순서대로 설명합니다.

### 1-1. 최초 한 번: 빌드

```bash
cd ~/Capstone_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 1-2. 실행할 때마다: 터미널 준비

새 터미널을 열 때마다 두 줄을 먼저 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/Capstone_sim/install/setup.bash
```

### 1-3. 시뮬 띄우고 로봇 움직이기

터미널 1 (시뮬 실행, 끌 때는 Ctrl+C):
```bash
ros2 launch pipe_robot_sim sim.launch.py course:=demo3d model:=two_real
```

터미널 2 (전진 명령, 5 cm/s):
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

또는 키보드로 직접 조종:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 1-4. 끝난 뒤 확인

시뮬을 끈 뒤 Gazebo가 남아 있지 않은지 확인합니다. 남아 있으면 다음 실행 결과가 섞입니다.

```bash
ps -eo pid,args | grep "ign gazebo" | grep -v grep
```

---

## 2. 무엇을 시뮬레이션하나

| 영역 | 내용 |
|---|---|
| **배관** | VG1 100A (외경 114mm, 내경 100.8mm), 엘보 프리셋 5종, 코스 9종. 배관 규격은 KS M 3404, 3410, JIS 기준 |
| **물리** | Gazebo Fortress로 중력, 마찰, 충돌을 계산. 바퀴는 모터 토크 (스톨 토크와 역기전력 특성 포함) |
| **센서** | 엔코더 (CPR 기준 카운트), IMU (MPU6050 수준 노이즈), ToF 전방 8×8 멀티존, 측면 ToF 2개 |
| **제어** | 바퀴 속도 PID, 관절 서보 (능동 제어), 좌우 궤도 조향, 꼭짓점 클램프 |
| **추정** | 엔코더와 IMU로 위치를 계산하는 추측항법. Gazebo 정답과 비교해 오차 출력 |
| **평가** | 벤치마크로 모델 × 배관 조합을 자동 주행하고 정렬, 롤, 미끄러짐, 실물 합격 여부 계산 |
| **부품 검사** | 부품이 몸통 안에 들어가는지, 질량과 무게중심, 배치도 그림 |

---

## 3. 차체 모델

모델은 `src/pipe_robot_sim/models/` 아래 폴더로 들어 있습니다. 실행할 때 `model:=폴더이름`으로 고릅니다.

| 모델 | 구조 | 관절 | 구동 | 제어 | 상태 |
|---|---|---|---|---|---|
| `three_wheel` | 앞마디 1개, 3륜 (120°) | 없음 | 3륜 전부 | 없음 | 기준선 (엘보에서 거의 뒤집힘) |
| `three_segment` | 3마디 (90/100/110mm) | 2개, 피치+요 능동 서보 | 앞·가운데 마디 | 센서 (ToF+IMU+엔코더) | 시뮬에서 안정적, 재측정 필요 |
| `two_module` | 2마디 | 2축 수동 | 6륜 | 없음 | 실패 (V자로 접힘), 참고용 |
| `two_active` | 2마디 (90/100mm) | 2축 능동 서보 | 6륜 | 센서 | 13개 배관 통과, 재측정 필요 |
| `two_real` | two_active + 실제 부품 배치 (474g) | 2축 능동 서보 | 6륜 | 센서 | 부품 배치 통과, 아래 90°가 불안정 |
| `two_real_x` | two_real의 바퀴를 X자 4륜으로 | 2축 능동 서보 | 8륜 | 센서 | 좌우 90° 개선, 배터리 배치 불가 |
| `pirate_v` | 뒤집힌 V 두 개 + 가운데 링크 | 꼭짓점 클램프 + C·D·요 | 6륜 | 없음 (일정 토크 또는 토션 스프링) | 토션 스프링 효과 없음, 수직 실패 |
| `tracked` | 몸통 100mm + 궤도 2 + 보조 바퀴 3 | 없음 | 궤도 (바퀴 4+4로 근사) | 좌우 속도차 (oracle) | 엘보 정렬 가장 좋음, 수직 등반 실패 |

자세한 구조와 파라미터는 [docs/02 차체 모델](docs/02_차체_모델.md)을 보세요.

> **용어:** `oracle`은 시뮬이 배관 모양을 미리 아는 상태에서 제어한다는 뜻입니다. 실제 로봇은 이 정보를 알 수 없으므로 oracle 결과는 **상한값**입니다. 실물은 `sensor` 모드로 평가해야 합니다.

---

## 4. 배관 코스와 엘보

### 4-1. 코스 (`course:=이름`)

| 코스 | 구성 | 설명 |
|---|---|---|
| `straight` | 직관 2m | 기본 주행 확인 (기본값) |
| `elbow45` | 직관 → 수평 45° → 직관 | 수평 45° |
| `elbow90` | 직관 → 수평 90° → 직관 | 수평 90° |
| `demo` | 1m → 45° → 0.5m → 90° → 1m | 데모 배관 |
| `elbow45_up` | 직관 → 위로 45° → 직관 | 수직 방향 45° |
| `elbow45_down` | 공중에서 시작 → 아래로 45° | 내리막 45° |
| `elbow90_up` | 직관 → 위로 90° → 수직 0.5m | 수직관 등반 |
| `elbow90_down` | 공중에서 시작 → 아래로 90° | 내리막 90° (가장 어려움) |
| `demo3d` | 위 45° → 수평 → 왼쪽 90° 등 | 수평과 수직이 섞인 배관 |

### 4-2. 엘보 프리셋 (`fitting90:=`, `fitting45:=`)

| 이름 | 종류 | 중심선 반경 R | 비고 |
|---|---|---|---|
| `dv90` | 배수용 DV 90° | 61 mm | 국내 판매 주력, 급한 엘보 |
| `dv_ll90` | 배수용 DV 대곡 90° | 125 mm | 국내 재고 드묾, 통과 쉬움 |
| `ts90` | 수도용 TS 90° | 50 mm (근사) | 안쪽 모서리가 거의 직각 |
| `dv45` | 배수용 DV 45° | 72 mm | |
| `ts45` | 수도용 TS 45° | 50 mm (근사) | |

- R 값은 제조사가 공개하지 않은 **상한값**입니다. 실물 엘보를 재서 `config/courses.yaml`만 고치면 됩니다.
- 규격 자세한 내용은 [docs/03 배관과 규격](docs/03_배관_규격.md).

---

## 5. 시뮬 실행 방법

```bash
ros2 launch pipe_robot_sim sim.launch.py [인자:=값 ...]
```

| 인자 | 기본값 | 가능한 값 | 설명 |
|---|---|---|---|
| `course` | `straight` | 위 코스 9종 | 배관 모양 |
| `model` | `three_wheel` | 위 모델 8종 | 로봇 모델 |
| `fitting90` | (코스 기본) | `dv90`, `dv_ll90`, `ts90` | 코스 안의 90° 엘보 교체 |
| `fitting45` | (코스 기본) | `dv45`, `ts45` | 코스 안의 45° 엘보 교체 |
| `suspension` | `all` | `all`, `top` | 바퀴 스프링 구성 (일부 모델) |
| `imu_mode` | `gyro` | `gyro`, `orientation` | 추정 자세 계산 방식 |
| `gui` | `true` | `true`, `false` | Gazebo 창 (`false`면 화면 없이 빠르게) |
| `rviz` | `true` | `true`, `false` | RViz 창 (로봇 모양 확인용) |
| `fullscreen` | `false` | `true`, `false` | Gazebo 창을 화면 크기로 |

**예시**

```bash
# 가장 기본: 직관에 three_wheel
ros2 launch pipe_robot_sim sim.launch.py

# 3D 데모 배관에 two_real (화면 있음)
ros2 launch pipe_robot_sim sim.launch.py course:=demo3d model:=two_real

# 90° 엘보를 대곡으로 바꿔 보기
ros2 launch pipe_robot_sim sim.launch.py course:=elbow90 model:=two_active fitting90:=dv_ll90

# 화면 없이 빠르게 (서버, 벤치 대신 단발 확인)
ros2 launch pipe_robot_sim sim.launch.py course:=demo model:=two_real gui:=false rviz:=false
```

> **주의:** `tof: true` 모델 (`three_segment`, `two_active`, `two_real`, `two_real_x`)은 ToF 센서 렌더링 때문에 **디스플레이가 있어야** 합니다. 화면 없는 서버에서는 벤치도 `DISPLAY`가 필요합니다.

**전진 명령 보내기**

```bash
# 고정 속도 (Ctrl+C로 중단)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"

# 회전 포함 (좌우 궤도 조향 기능이 있는 tracked 모델에서만 효과가 있음)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.3}}"
```

---

## 6. 노드와 토픽

### 6-1. 주요 노드

| 노드 | 역할 |
|---|---|
| `motor_driver` | `/cmd_vel`을 바퀴 토크로 바꿈. 모드 `speed_pid` (기본), `open_loop`, `diff` |
| `encoder_sim` | 관절 각도를 엔코더 카운트로 바꿈 |
| `odometry` | 엔코더 + IMU로 추정 위치 계산, 정답과 오차 출력 |
| `joint_follow` | 관절 서보 명령. `follow_mode: sensor` (센서만 사용) 또는 `oracle` |
| `oracle_steer` | 좌우 궤도 속도차로 조향 (`tracked`) |
| `clamp_torque` | 꼭짓점 클램프 토크 (`pirate_v`, 일정 토크 모드) |
| `pipe_markers` | RViz에 배관 표시 |

### 6-2. 도구 (명령줄)

| 명령 | 역할 |
|---|---|
| `ros2 run pipe_robot_sim bench` | 모델 × 배관 자동 비교 |
| `ros2 run pipe_robot_sim layout_check` | 부품 배치 검사, 질량·무게중심, 배치도 |
| `ros2 run pipe_robot_sim generate_world` | 배관 SDF 파일만 생성 |

### 6-3. 주요 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/cmd_vel` | Twist | 목표 전진 속도 `linear.x` [m/s], 조향 `angular.z` |
| `/wheel_encoders` | Int64MultiArray | 바퀴별 누적 엔코더 카운트 |
| `/imu` | Imu | 200 Hz, 노이즈 포함 |
| `/joint_states` | JointState | 모든 관절 각도와 속도 |
| `/servo/<관절>` | Float64 | 능동 관절 각도 명령 [rad] |
| `/motor/<관절>/torque` | Float64 | 바퀴 토크 명령 [N·m] |
| `/tof/front/points` | PointCloud2 | 전방 8×8 ToF |
| `/tof/front`, `/tof/left`, `/tof/right` | LaserScan | ToF 거리 |
| `/odom` | Odometry | 추정 위치 (엔코더 + IMU) |
| `/ground_truth/odom` | Odometry | Gazebo 정답 위치 (평가용) |
| `/gz_poses` | TFMessage | 링크별 정답 자세 (마디 정렬 평가용) |

자세한 내용은 [docs/01 구조와 실행](docs/01_구조와_실행.md)을 보세요.

---

## 7. 벤치마크 (설계 비교)

여러 모델을 여러 배관에서 자동으로 주행시키고 점수를 매기는 도구입니다.

```bash
export ROS_DOMAIN_ID=77 IGN_PARTITION=bench
ros2 run pipe_robot_sim bench --models two_real two_active \
    --courses straight elbow90 demo3d --fittings "" dv_ll90 ts90 \
    -o bench/결과이름.csv
```

| 옵션 | 뜻 |
|---|---|
| `--models` | 비교할 모델들 (띄어쓰기로 여러 개) |
| `--courses` | 시험할 배관들 |
| `--fittings` | 엘보 종류 (`""`는 코스 기본) |
| `--speed` | 전진 속도 [m/s], 기본 0.06 |
| `-o` | 결과 파일 (csv). 로그는 `결과이름_logs/`에 저장 |

**평가 항목과 실물 합격 기준**

| 항목 | 기준 | 의미 |
|---|---|---|
| 도착 (PASS) | 끝까지 도착 | 배관을 통과했나 |
| 정렬 (align_max) | 15° 이하 | 코너에서 마디가 배관과 일자인가 |
| 롤 (roll_max) | 10° 이하 | 몸통이 비틀리지 않나 |
| 한계 접촉 (sat_s) | 0.5 s 이하 | 관절이나 스프링이 끝까지 가서 막히나 |
| 미끄러짐 (slip_pct) | 10% 이하 | 바퀴가 헛도나 |
| 정체 (stall_s) | 1 s 이하 | 중간에 걸리나 |

- 모든 기준을 만족하면 `실물OK O`입니다.
- 같은 조건도 결과가 조금씩 달라질 수 있습니다. 중요한 판단은 3회 이상 반복하세요.
- 기준과 해석 방법은 [docs/04 벤치마크 기준](docs/04_벤치마크_기준.md).

---

## 8. 부품 배치 검사

부품이 몸통 안에 실제로 들어가는지 확인합니다. 모델 폴더의 `layout.yaml`을 읽습니다.

```bash
ros2 run pipe_robot_sim layout_check two_real                       # 검사만
ros2 run pipe_robot_sim layout_check two_real --plot 배치도.png     # 배치도 그림
ros2 run pipe_robot_sim layout_check two_real --apply               # 질량·무게중심을 robot.yaml에 반영
```

- 검사 항목: 몸통 안에 있는가, 마디 길이를 넘지 않는가, 바퀴가 움직이는 공간을 침범하지 않는가, 다른 부품과 겹치지 않는가.
- `✗`로 표시된 부품을 `layout.yaml`에서 옮기고 다시 검사하세요. 마지막에 `배치 OK`가 나와야 합니다.
- 배치도 그림에서 빨간 상자가 문제 부품, 회색 띠가 바퀴 공간입니다.

---

## 9. 폴더 구조

```
Capstone_sim/
├── README.md                    이 문서
├── docs/                        상세 문서 (00~09)
├── bench/                       벤치 결과 (csv), 로그 (*_logs/), 스윕 스크립트
├── build/, install/, log/       colcon 빌드 산출물 (직접 고치지 않음)
└── src/pipe_robot_sim/          ROS 2 패키지
    ├── config/
    │   ├── courses.yaml         배관 규격, 엘보 프리셋, 코스 정의
    │   └── sim.rviz             RViz 설정
    ├── launch/
    │   └── sim.launch.py        실행 진입점 (배관, 로봇, 노드를 한 번에 띄움)
    ├── models/<모델이름>/
    │   ├── <모델이름>.urdf.xacro   로봇 모양 (링크, 관절, 바퀴, 센서)
    │   ├── robot.yaml              파라미터 (질량, 스프링, 서보 토크 등)
    │   └── layout.yaml             (선택) 부품 배치
    └── pipe_robot_sim/*.py      노드와 도구 코드
```

---

## 10. 현재 상태와 결과

**가장 먼저 알아 둘 것:** 2026-10-15에 벤치가 실행이 끝난 Gazebo를 완전히 끄지 못해 결과가 섞이는 문제를 찾았습니다. 그 이전의 모든 벤치 결과(`bench/*.csv` 대부분)는 **재측정 대상**입니다. 수치보다는 경향(어떤 설계가 어떤 문제에 약한지)으로 참고하세요.

| 구분 | 상태 |
|---|---|
| 시뮬 기능 (배관, 모델, 센서, 노드) | 구현 완료 |
| 벤치 도구 | 오염 문제 수정 완료 (2026-10-15) |
| 부품 배치 검사 | 완료 (`two_real` 통과) |
| 지금까지 설계 비교 결과 | 재측정 필요 |
| pirate_v 토션 스프링 재측정 | 완료: 효과 없음 (`bench/pv2_*`) |

**잠정 발견 (재측정 전)**
- 좌우 엘보에서 롤(몸통 비틀림)이 위·아래 엘보보다 큼.
- 관절은 능동 제어여야 함 (수동 관절은 엘보에서 V자로 접힘).
- 2마디 (`two_active`)는 미끄러짐과 위치 오차가 작고, 3마디 (`three_segment`)는 롤이 작음.
- 내리막 90°가 가장 어려움. 관절부 50mm, 바퀴 누름 11N, 스프링 스트로크 ±20mm에서 개선.
- 궤도 (`tracked`)는 엘보 정렬이 가장 좋지만 수직 등반은 못 함.

자세한 표와 근거는 [docs/05 결과 요약](docs/05_결과_요약.md), 설계 방향과 미결정 사항은 [docs/07](docs/07_설계_방향과_미결정.md).

---

## 11. 알려진 문제

| 문제 | 상태 |
|---|---|
| 벤치가 이전 실행의 Gazebo를 끄지 못해 결과가 섞임 | 해결 (2026-10-15), 이전 결과 재측정 필요 |
| 화면이 있는 실행은 실시간 대비 0.5~0.9배로 느림 | 설계 한계 (벤치는 헤드리스) |
| ToF 센서는 디스플레이가 필요 | 부분 해결 (벤치가 `:0` 자동 사용) |
| 전체화면 (`fullscreen`)은 창 크기만 맞춤 | 부분 해결 (창 관리 도구 없음) |
| 엘보에서 롤이 큼 | 미해결 (설계 과제) |
| 서보 토크 (0.35 N·m) 등 다수 값이 추정 | 미해결 (실측 필요, [docs/08](docs/08_추정값_목록.md)) |
| 궤도 플러그인이 배관에서 동작하지 않음 | 우회 (바퀴 4개로 근사) |

전체 목록과 원인은 [docs/06 알려진 문제](docs/06_알려진_문제.md).

---

## 12. 새로 추가하기

### 12-1. 새 차체 모델

1. `src/pipe_robot_sim/models/<이름>/` 폴더를 만듭니다.
2. **파일 이름은 폴더 이름과 같게** `<이름>.urdf.xacro`를 만듭니다 (launch가 이 이름으로 찾습니다).
3. `robot.yaml`을 만듭니다. 최소한 다음 항목이 필요합니다.
   - `wheel_joints`: 모터로 돌릴 바퀴 관절 이름 목록
   - `wheel_radius`, `encoder_cpr`, `motor_stall_torque`, `motor_no_load_speed`
   - `bench_spines`: 벤치가 정렬을 재는 링크 (예: `[base_link, mid_link]`)
   - 나머지는 기존 모델의 `robot.yaml`을 복사해 고치는 것이 가장 쉽습니다.
4. (선택) `layout.yaml`로 부품을 배치하고 `layout_check`로 검사합니다.
5. 실행: `ros2 launch pipe_robot_sim sim.launch.py model:=<이름>`

### 12-2. 새 배관 코스

`config/courses.yaml`의 `courses:`에 항목을 추가합니다. 사용할 수 있는 구성 요소는 다음과 같습니다.

```yaml
my_course:
  - {start_z: 0.6}                    # (맨 앞에만) 시작 높이
  - {straight: 1.0}                   # 직관 1m
  - {fitting: dv90, dir: left}        # 엘보 (dir: left, right, up, down)
  - {straight: 0.5}
  - {cap: true}                       # 끝 마개
```

### 12-3. 새 엘보 프리셋

`config/courses.yaml`의 `fittings:`에 항목을 추가합니다.

```yaml
my_elbow: {angle: 90, R: 0.080, Z: 0.070, bore: 0.0990}
```

- `angle`: 각도 [도], `R`: 중심선 반경 [m], `Z`: 엘보 중심에서 소켓 바닥까지 [m], `bore`: 몸통 내경 [m]

### 12-4. 코드를 고쳤을 때

- `yaml`, `xacro`, `layout.yaml`, `courses.yaml` 수정: 빌드 없이 바로 반영됩니다.
- 새 노드나 새 실행 파일 추가, `setup.py` 수정: 다시 빌드합니다.

```bash
cd ~/Capstone_sim && colcon build --symlink-install && source install/setup.bash
```

---

## 13. 문서 안내

| 문서 | 내용 |
|---|---|
| [00 입문: 먼저 읽기](docs/00_입문_먼저_읽기.md) | ROS 2란, 용어, 도구 사용법, 첫 실행 순서 |
| [01 구조와 실행](docs/01_구조와_실행.md) | 폴더, 빌드, 실행 인자, 노드, 토픽 상세 |
| [02 차체 모델](docs/02_차체_모델.md) | 모델 8종 구조와 파라미터, 상태 |
| [03 배관과 규격](docs/03_배관_규격.md) | VG1 치수, 엘보 규격과 출처, 코스 |
| [04 벤치마크 기준](docs/04_벤치마크_기준.md) | 평가 항목, 합격 기준, 해석 방법 |
| [05 결과 요약](docs/05_결과_요약.md) | 지금까지 결과, 재측정 상태 |
| [06 알려진 문제](docs/06_알려진_문제.md) | 해결한 문제와 남은 문제 |
| [07 설계 방향과 미결정](docs/07_설계_방향과_미결정.md) | 잠정 설계, 미결정 사항 |
| [08 추정값 목록](docs/08_추정값_목록.md) | 추정한 값과 실측 방법 |
| [09 참고자료](docs/09_참고자료.md) | 규격, 논문, 제품 자료 출처 |

---

## 14. 환경과 설치

| 항목 | 버전 / 내용 |
|---|---|
| 운영체제 | Ubuntu 22.04 (Jammy) |
| ROS | ROS 2 Humble |
| 시뮬레이터 | Gazebo Fortress (ign 6.18), ros_gz 0.244 |
| 언어 | Python 3.10 |
| 주요 라이브러리 | numpy, pyyaml, matplotlib (배치도) |
| 그래픽 | NVIDIA GTX 1060 6GB (ToF 렌더링에 디스플레이 필요) |
| 빌드 도구 | colcon, xacro |
| 보조 도구 | rviz2, teleop_twist_keyboard |

**설치가 이미 되어 있는 PC에서는 1장의 명령만 따르면 됩니다.** 다른 PC에 설치할 때는 ROS 2 Humble 설치 후 다음 패키지를 추가합니다.

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard ros-humble-rviz2
```

> 패키지 이름과 버전은 설치 환경에 따라 다를 수 있습니다. 설치 후 `ros2 pkg list | grep ros_gz`로 확인하세요.

---

## 참고

- 시뮬 결과는 추정 파라미터 위에서 나온 값입니다. 실측값이 생기면 [docs/08](docs/08_추정값_목록.md)을 기준으로 바꾸고 재측정하세요.
- 자료 출처는 [docs/09 참고자료](docs/09_참고자료.md)에 있습니다.
