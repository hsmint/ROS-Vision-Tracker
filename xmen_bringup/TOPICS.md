# 노드 수신·송신 (토픽·액션)

Xmen 저장소의 ROS 노드(인지 `xmen_vision`, 제어 `xmen_control`, 통합 `xmen_bringup`)가 무엇을 받고 무엇을 내는지
정리한 문서다. 토픽 이름·타입·QoS는 모두 `xmen_bringup/tracking_common/tracking_common/interface.py` 한 곳에서 정의하고, 노드는 그 값을 가져다 쓴다.
**이름·QoS를 바꿀 때는 interface.py만 고친다** — 노드 코드에 토픽 이름을 직접 쓰지 않는다.

```python
from tracking_common import interface as I
self.create_subscription(PointStamped, I.TOPICS['target']['name'], self.on_target, I.TARGET_QOS)   # 수신
self.pub = self.create_publisher(Twist, I.TOPICS['gimbal_cmd']['name'], I.CMD_QOS)                # 송신
```

---

## 1. 전체 흐름

카메라 입력은 두 방식이 있고, **RPi(보드)는 현재 방식 B**로 실행한다. detector 뒤는 두 방식이 같다.

```
방식 B (RPi, source:=ros)
[D435] ══USB══▶ realsense2_camera ──/camera/camera/rgbd──▶ target_detector ──▶ (아래와 같음)

방식 A (PC, source:=realsense, 기본)
[D435] ══USB══▶ target_detector   (pyrealsense2로 직접 연다. 영상 토픽 없음)

공통
target_detector ──/target──▶ tracking_controller ──/cmd_vel──▶ motor_driver ──X── (하드웨어 미연결)
      │                            │      ▲
      └─/perception_status         │      └── /search (액션 요청)
                                   └─/tracking_status
```

| 단계 | 노드 | 패키지 | 받는 것 → 내는 것 |
|---|---|---|---|
| 카메라 (방식 B) | `camera` (realsense2_camera) | realsense2_camera (apt) | USB → 컬러·정렬 뎁스·camera_info 묶음 |
| 인지 | `target_detector` | target_perception | 카메라 영상 → 목표의 화면 중심 오차 |
| 제어 | `tracking_controller` | target_control | 오차 → 팬·틸트 속도 명령, 추적 상태 |
| 구동 | `motor_driver` | target_control | 속도 명령 → 하드웨어 (현재 출력 OFF, 로그만) |

---

## 2. 토픽

| 토픽 | 타입 | 송신 | 수신 | QoS | 주기 |
|---|---|---|---|---|---|
| `/camera/camera/rgbd` (방식 B) | `realsense2_camera_msgs/RGBD` | realsense2_camera | target_detector (`source:=ros`), camera_viewer | 발행 reliable keep_last 1 / 구독 reliable keep_last 5 | 30 Hz |
| `/target` | `geometry_msgs/PointStamped` | target_detector | tracking_controller | best-effort, keep_last 1 | 영상마다 (약 30 Hz) |
| `/perception_status` | `std_msgs/String` | target_detector | (모니터링) | reliable, keep_last 1 | 상태 변화 시 + 1 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | tracking_controller | motor_driver | reliable, keep_last 1 | 20 Hz 고정 |
| `/tracking_status` | `std_msgs/String` | tracking_controller | (모니터링·기록) | reliable, keep_last 1 | 20 Hz |

### `/target` 필드

| 필드 | 값 |
|---|---|
| `header.stamp` | 원본 영상 **촬영 시각** (RealSense global_time. 아니면 수신 시각 — detector 로그에 기준 표시) |
| `header.frame_id` | `camera_color_optical_frame` |
| `point.x` | ex = (cx − W/2)/(W/2), **오른쪽 +**, [−1, 1] |
| `point.y` | ey = (cy − H/2)/(H/2), **아래쪽 +**, [−1, 1] |
| `point.z` | 면적비 contour_area/(W×H). **0 = 미검출** — 이때 x·y는 의미 없음(0) |

- 정상 영상에서 목표가 없어도 **z=0으로 발행**한다(미검출).
- 새 영상이 없으면 **발행하지 않는다**(통신 중단). 받는 쪽은 "z=0"과 "토픽 침묵"을 구분한다.

### `/cmd_vel` 필드

| 필드 | 값 | 부호 (REP-103) |
|---|---|---|
| `angular.z` | 팬(좌우) 각속도 [rad/s] | **+ = 왼쪽**(반시계). ex>0(목표 오른쪽) → 음수 |
| `angular.y` | 틸트(상하) 각속도 [rad/s] | **+ = 아래**. ey>0(목표 아래) → 양수. *실제 모터로 부호 확인 필요* |
| 나머지 | 0 | 사용하지 않음 |

정지 명령 = 두 축 0.0을 **계속 발행**(발행을 멈추는 것이 아니다).

### 상태 값

| `/perception_status` | 뜻 |
|---|---|
| `OK` | 목표 검출 |
| `NO_TARGET` | 영상은 정상, 목표 없음 |
| `CAMERA_STALL` | 0.3 s 동안 새 영상 없음 — `/target` 발행 중단 |

| `/tracking_status` | 뜻 | 명령 |
|---|---|---|
| `WAITING` | 시작 후 아직 신선한 입력 없음 | 0, 0 |
| `TRACKING` | 검출, 한 축 이상 데드밴드 밖 | 축마다 clamp(sign × Kp × 오차) |
| `CENTERED` | 검출, 두 축 모두 데드밴드 안 | 0, 0 |
| `LOST` | 신선한 입력의 z=0 | 0, 0 (x·y 안 봄) |
| `TIMEOUT` | 마지막 신선한 입력 후 0.5 s 초과 | 0, 0 |
| `SEARCHING` | `/search` 실행 중 | 팬만 회전, 틸트 0 |

---

## 3. 액션

| 이름 | 타입 | 서버 | 클라이언트 |
|---|---|---|---|
| `/search` | `tracking_interfaces/action/Search` | tracking_controller | search_test(시험) / 상위 임무 노드 |

| 구분 | 필드 |
|---|---|
| 요청 | `direction` (+1 왼쪽 / −1 오른쪽), `speed` [rad/s] > 0, `timeout` [s] > 0 |
| 결과 | `found`, `reason` (`FOUND` \| `TIMEOUT` \| `CANCELED` \| `NO_INPUT` \| `INVALID_GOAL`), `elapsed` [s] |
| 진행 | `elapsed`, `angular_z`, `state` (`SEARCHING`) |

---

`/camera/camera/rgbd`가 실행 중일 때는 `/camera/camera/color/image_raw`, `aligned_depth_to_color/image_raw` 등
realsense2_camera의 개별 영상 토픽도 함께 나온다. 우리 노드는 이것들을 구독하지 않는다(RGBD 한 토픽만 사용).

## 4. 노드별 수신·송신

### realsense2_camera (방식 B, xmen_vision/target_perception/launch/camera.launch.py)

| 구분 | 내용 |
|---|---|
| 수신 | D435 USB |
| 송신 | `/camera/camera/rgbd` (컬러 + 컬러에 정렬한 뎁스 + camera_info, 같은 프레임끼리 묶음) |
| 설정 | 640×360@30, `align_depth`, `enable_sync`, `enable_rgbd`, `initial_reset`, 적외선·IMU·점구름 끔 |
| 실행 | `ros2 launch target_perception camera.launch.py viewer:=false` |

### target_detector (xmen_vision/target_perception/target_perception/detector_node.py)

| 구분 | 내용 |
|---|---|
| 수신 | 방식 B(RPi): `/camera/camera/rgbd` 구독 (`source:=ros`). 방식 A(PC): 카메라 USB 직접 (pyrealsense2, `source:=realsense`, 기본) |
| 송신 | `/target`, `/perception_status` |
| 동작 | B: RGBD 수신 → cv_bridge로 변환 → 검출. A: 전용 스레드가 `wait_for_frames`로 새 프레임 대기 → 뎁스를 컬러에 정렬 → 검출. 검출은 HSV → 컨투어 → 필터 → 뎁스 중앙값 검증 → 선택 → 중심 |
| 거르는 것 | 같은 영상은 발행 안 함 (B: header.stamp가 이전 이하, A: 프레임 번호가 이전 이하) |
| 실행 | B: `ros2 run target_perception detector --ros-args -p source:=ros` / A: `ros2 run target_perception detector` |
| 정지 판단 | 0.3 s 새 영상 없음 → `CAMERA_STALL`, 발행 중단 |
| 주요 파라미터 | `config`(detector.yaml), `show`(검출 화면), `target_topic`·`status_topic`(재처리 시 `/target_replay`·`/perception_status_replay`) |

### tracking_controller (xmen_control/target_control/target_control/controller_node.py)

| 구분 | 내용 |
|---|---|
| 수신 | `/target` |
| 송신 | `/cmd_vel`, `/tracking_status`, `/search` 서버 |
| 동작 | `/target` 수신 시 신선도만 판정해 저장 → **20 Hz 타이머**가 상태 결정·명령 계산·발행(입력 주기와 무관하게 일정 주기) |
| 거르는 것 | NaN·범위 밖 값(`invalid`), 같은·이전 stamp(`old_or_duplicate_stamp`), 촬영 시각이 0.5 s보다 오래됨(`too_old`) |
| 정지 판단 | z=0 첫 프레임부터 `LOST`(0), 마지막 신선한 입력 후 0.5 s → `TIMEOUT`(0) |
| 파라미터 | `xmen_bringup/target_bringup/config/tracking.yaml` — kp·cmd_sign·deadband·max_speed (팬), *_tilt (틸트), rate_hz, timeout, max_input_age, tilt_enabled |

### motor_driver (xmen_control/target_control/target_control/motor_driver_node.py)

| 구분 | 내용 |
|---|---|
| 수신 | `/cmd_vel` |
| 송신 | 없음 (하드웨어 출력 지점. 현재 `output_enabled: false` → 로그만) |
| 동작 | 축별 상한(`hw_max_speed`, `hw_max_speed_tilt`)으로 다시 제한 → 값이 바뀔 때 로그 `[OFF] pan=… tilt=… rad/s` |
| 정지 판단 | 0.2 s 명령 없음 → 두 축 0 (`명령 끊김(watchdog)`) |
| 미구현 | OpenCR 시리얼 전송, 현재 각도(`/joint_states`) 발행 |

### 보조·검증 노드 (xmen_bringup/target_bringup, 모터 출력 OFF)

| 노드 | 수신 | 송신 | 용도 |
|---|---|---|---|
| `camera_viewer` (target_perception) | `/camera/camera/rgbd` | 없음 | realsense2_camera 방식일 때 영상 확인 |
| `input_test` | `/cmd_vel`, `/tracking_status` | `/target` (모의) | 모의 입력 11단계 → PASS/FAIL, CSV |
| `gimbal_sim` | `/cmd_vel` | `/target` (모의) | 명령을 적분한 가상 짐벌로 부호 시험 |
| `search_test` | `/cmd_vel`, `/tracking_status` | `/target` (모의), `/search` 요청 | 액션 5가지 경우 |
| `interface_check` | 위 토픽 4개 전부 | 없음 | 타입·QoS·주기·호환성 검사 + 기록 |

주의: 시험 노드는 `/target`을 **모의로 발행**한다. detector와 동시에 띄우면 값이 섞인다 —
같이 켜야 하면 `export ROS_DOMAIN_ID=<다른 번호>`로 분리한다.

---

## 5. 정지 규칙 (세 겹)

| 끊기는 곳 | 감지 | 결과 |
|---|---|---|
| 카메라 → detector | detector: 0.3 s 새 영상 없음 (B: RGBD 수신 없음, A: 프레임 없음) | `CAMERA_STALL`, `/target` 발행 중단 |
| detector → controller | controller: 0.5 s 신선한 입력 없음 | `TIMEOUT`, 두 축 0을 계속 발행 |
| controller → motor_driver | motor_driver: 0.2 s 명령 없음 | 두 축 0 |
| RPi → OpenCR | OpenCR 펌웨어 워치독 | **미구현** — 제어 프로그램이 죽어도 모터가 서야 한다(발제 필수) |

---

## 6. QoS를 이렇게 정한 이유

| QoS | 토픽 | 이유 |
|---|---|---|
| best-effort, keep_last 1 | `/target` | 최신 값만 의미가 있다. 늦은 재전송보다 다음 프레임이 낫다 |
| reliable, keep_last 1 | 상태·명령 | 유실되면 안 되는 값. 최신 1개만 |
| reliable, keep_last 5 | 영상(`/camera/camera/rgbd`) 구독 | 큰 영상(≈1 MB)을 best-effort로 받으면 조각 유실로 대부분 버려졌다(163장 중 7장, PC 실측) |

reliable로 구독하는 노드는 best-effort로 발행되는 `/target`을 받지 못한다 — `/target` 구독은 반드시 best-effort.
확인: `ros2 topic echo /target --qos-reliability best_effort`

---

## 7. 확인 명령

```bash
ros2 topic list
ros2 node info /target_detector                           # 노드가 받는·내는 토픽
ros2 topic info -v /target                                # 발행·구독 노드와 QoS
ros2 topic hz /camera/camera/rgbd                         # 방식 B, 약 30 Hz
ros2 topic hz /target --qos-reliability best_effort       # 약 30 Hz (RPi 29.7)
ros2 topic hz /cmd_vel                             # 20 Hz (RPi 20.0)
ros2 topic echo /target --qos-reliability best_effort
ros2 topic echo /cmd_vel
ros2 topic echo /tracking_status
ros2 topic echo /perception_status
ros2 action send_goal /search tracking_interfaces/action/Search "{direction: 1.0, speed: 0.2, timeout: 3.0}" --feedback
```
