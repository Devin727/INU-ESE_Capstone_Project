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

> **현재 상태 (2026-10-19):** 설계를 처음부터 다시 하기 위해 이전 모델과 벤치 결과를 모두 지웠습니다 (이전 기록은 git 커밋 `810a8d1`). 지금은 논문 로봇을 재현한 참고 모델 2종(`vshape100`, `airo71`)만 있고 아직 시험 전입니다. 새 모델은 [docs/02 규약](docs/02_차체_모델.md)에 맞춰 `src/pipe_robot_sim/models/`에 추가합니다.

---

## 1. 처음 시작하기

**ROS 2나 Gazebo를 처음 보는 분은 [docs/00 입문 문서](docs/00_입문_먼저_읽기.md)부터 읽으세요.**

```bash
# 최초 한 번: 빌드
cd ~/INU-ESE_Capstone_Project/simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 새 터미널마다
source /opt/ros/humble/setup.bash
source ~/INU-ESE_Capstone_Project/simulation/install/setup.bash

# 배관만 보기 (모델 없이)
ros2 launch pipe_robot_sim sim.launch.py course:=demo3d

# 모델이 생긴 뒤: 로봇 띄우고 다른 터미널에서 5 cm/s 전진
ros2 launch pipe_robot_sim sim.launch.py course:=demo3d model:=<모델>
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

끈 뒤에는 Gazebo가 남아 있지 않은지 확인합니다. 남아 있으면 다음 실행 결과가 섞입니다.

```bash
ps -eo pid,args | grep "ign gazebo" | grep -v grep
```

---

## 2. 구성

| 영역 | 내용 |
|---|---|
| **배관** | VG1 100A (외경 114mm, 내경 100.8mm), 엘보 프리셋 5종, 코스 9종. KS M 3404, 3410, JIS 기준. 이음부 턱 재현 |
| **물리** | Gazebo Fortress (중력, 마찰, 충돌). 바퀴는 모터 토크 (스톨 토크와 역기전력 특성) |
| **노드** | `motor_driver` (속도 PID → 바퀴 토크), `encoder_sim` (엔코더 카운트), `odometry` (엔코더 + IMU 추측항법, 정답과 오차 비교), `pipe_markers` (RViz 배관) |
| **도구** | `bench` (모델 × 배관 자동 주행 평가), `layout_check` (부품 배치 검사, 질량·무게중심), `generate_world` (배관 SDF만 생성) |
| **차체 모델** | 참고 모델 `vshape100` (V자형 100mm 로봇), `airo71` (AIRo-7.1). 구조와 추정값은 [docs/02](docs/02_차체_모델.md) |

---

## 3. 배관 코스와 엘보

### 코스 (`course:=이름`)

| 코스 | 구성 |
|---|---|
| `straight` | 직관 2m (기본값) |
| `elbow45` | 직관 → 수평 45° → 직관 |
| `elbow90` | 직관 → 수평 90° → 직관 |
| `demo` | 1m → 45° → 0.5m → 90° → 1m (노션 데모 배관, T자 없음) |
| `elbow45_up` / `elbow45_down` | 위로 / 아래로 45° |
| `elbow90_up` / `elbow90_down` | 수직관 등반 / 내리막 90° |
| `demo3d` | 위 45° → 수평 → 왼쪽 90° (수평·수직 혼합) |

### 엘보 프리셋 (`fitting90:=`, `fitting45:=`)

| 이름 | 종류 | 중심선 반경 R | 비고 |
|---|---|---|---|
| `dv90` | 배수용 DV 90° | 61 mm | 국내 판매 주력, 급한 엘보 |
| `dv_ll90` | 배수용 DV 대곡 90° | 125 mm | 국내 재고 드묾 |
| `ts90` | 수도용 TS 90° | 50 mm (근사) | 안쪽 모서리가 거의 직각 |
| `dv45` | 배수용 DV 45° | 72 mm | |
| `ts45` | 수도용 TS 45° | 50 mm (근사) | |

- R은 제조사가 공개하지 않은 **상한값**입니다. 실물 엘보를 재서 `config/courses.yaml`만 고치면 됩니다.
- 새 코스·엘보 추가 방법과 규격 근거는 [docs/03](docs/03_배관_규격.md).

---

## 4. 실행 인자

```bash
ros2 launch pipe_robot_sim sim.launch.py [인자:=값 ...]
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `course` | `straight` | 배관 코스 |
| `model` | (비어 있음) | `models/` 아래 폴더 이름. 비우면 배관만 |
| `fitting90` / `fitting45` | (코스 기본) | 코스 안의 90° / 45° 엘보 교체 |
| `suspension` | `all` | xacro로 넘기는 바퀴 스프링 구성 (모델이 쓰는 경우만) |
| `imu_mode` | `gyro` | `gyro` (자이로 적분) / `orientation` (Gazebo 자세 그대로) |
| `gui` / `rviz` | `true` | Gazebo 창 / RViz 창 |
| `fullscreen` | `false` | Gazebo 창을 화면 크기로 |

노드와 토픽 상세는 [docs/01](docs/01_구조와_실행.md).

---

## 5. 벤치마크와 부품 배치 검사

```bash
# 모델 × 배관 자동 비교 (헤드리스)
export ROS_DOMAIN_ID=77 IGN_PARTITION=bench
ros2 run pipe_robot_sim bench --models <모델> --courses straight elbow90 demo3d \
    --fittings "" dv_ll90 ts90 -o bench/<결과이름>.csv

# 부품 배치 검사 (models/<모델>/layout.yaml)
ros2 run pipe_robot_sim layout_check <모델> --plot 배치도.png
```

| 평가 항목 | 실물 합격 기준 |
|---|---|
| 도착 | PASS |
| 정렬 (`align_max`) | 15° 이하 |
| 롤 (`roll_max`) | 10° 이하 |
| 한계 접촉 (`sat_s`) | 0.5 s 이하 |
| 미끄러짐 (`slip_pct`) | 10% 이하 |
| 정체 (`stall_s`) | 1 s 이하 |

기준과 해석은 [docs/04](docs/04_벤치마크_기준.md), 결과 기록은 [docs/05](docs/05_결과_요약.md).

---

## 6. 폴더 구조

```
simulation/
├── README.md                    이 문서
├── docs/                        상세 문서 (00~09)
├── bench/                       벤치 결과 (처음 실행 때 생김)
├── build/, install/, log/       colcon 빌드 산출물 (커밋하지 않음)
└── src/pipe_robot_sim/          ROS 2 패키지
    ├── config/
    │   ├── courses.yaml         배관 규격, 엘보 프리셋, 코스 정의
    │   └── sim.rviz             RViz 설정
    ├── launch/sim.launch.py     실행 진입점
    ├── models/                  차체 모델 (common/ = 공통 매크로)
    └── pipe_robot_sim/*.py      노드와 도구
```

- 이미 있는 `yaml`, `xacro` 수정은 빌드 없이 바로 반영됩니다. 새 파일(모델, 노드)을 추가했거나 `setup.py`를 고쳤으면 다시 빌드합니다.

---

## 7. 문서 안내

| 문서 | 내용 |
|---|---|
| [00 입문](docs/00_입문_먼저_읽기.md) | ROS 2란, 용어, 도구 사용법, 첫 실행 순서 |
| [01 구조와 실행](docs/01_구조와_실행.md) | 폴더, 빌드, 실행 인자, 노드, 토픽 |
| [02 차체 모델](docs/02_차체_모델.md) | 새 모델 규약 (토픽, 플러그인, robot.yaml, layout.yaml) |
| [03 배관과 규격](docs/03_배관_규격.md) | VG1 치수, 엘보 규격과 출처, 코스 |
| [04 벤치마크 기준](docs/04_벤치마크_기준.md) | 평가 항목, 합격 기준 |
| [05 결과 요약](docs/05_결과_요약.md) | 결과 기록 (비어 있음) |
| [06 알려진 문제](docs/06_알려진_문제.md) | 시뮬 기반의 문제와 주의 사항 |
| [07 설계 방향과 미결정](docs/07_설계_방향과_미결정.md) | 팀이 정할 것, 결정 기록 |
| [08 추정값 목록](docs/08_추정값_목록.md) | 추정한 값과 실측 방법 |
| [09 참고자료](docs/09_참고자료.md) | 규격, 논문, 제품 자료 출처 |

---

## 8. 환경과 설치

| 항목 | 버전 / 내용 |
|---|---|
| 운영체제 | Ubuntu 22.04 (Jammy) |
| ROS | ROS 2 Humble |
| 시뮬레이터 | Gazebo Fortress (ign 6.18), ros_gz 0.244 |
| 언어 | Python 3.10 (`/usr/bin/python3`) |
| 주요 라이브러리 | numpy, pyyaml, matplotlib (배치도) |
| 그래픽 | ToF 센서 렌더링에 디스플레이 필요 |

다른 PC에 설치할 때는 ROS 2 Humble 설치 후:

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard ros-humble-rviz2
```
