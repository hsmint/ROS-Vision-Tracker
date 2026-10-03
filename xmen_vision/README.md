# xmen_vision — 비전 기반 객체 추적 (인지·통합)

ROS 2 Lyrical · Python(rclpy) · Intel RealSense D435 · 목표: 파란 3×3×6 cm 직육면체 (근거리 ~ 1 m)
RPi에서 검출·제어 계산 → OpenCR(Dynamixel 팬·틸트)로 명령.

```
[D435] ──▶ target_detector ─/target─▶ tracking_controller ─/gimbal/cmd_vel─▶ motor_driver ─(시리얼, 미구현)─▶ OpenCR
               └─/perception_status        ├─/tracking_status                (출력 OFF, 로그만)
                                           └─/search (액션)
```
노드별 수신·송신, 메시지 필드, QoS, 정지 규칙 → **[TOPICS.md](TOPICS.md)**

## 패키지

| 패키지 | 실행 파일 | 내용 |
|---|---|---|
| `tracking_interfaces` | — | `/search` 액션 정의 (`action/Search.action`) |
| `tracking_common` | — | `interface.py` — 토픽 이름·타입·QoS·상태 값 단일 정의 |
| `target_perception` | `detector`, `camera_viewer` | 검출(`detector.py`), 설정(`config/detector.yaml`), `launch/perception.launch.py`, `launch/camera.launch.py` |
| `target_control` | `controller`, `motor_driver` | 팬·틸트 P 제어·상태·`/search` 서버, 하드웨어 출력(기본 OFF) |
| `target_bringup` | `input_test`, `gimbal_sim`, `search_test`, `interface_check` | 전체 실행 launch, `config/tracking.yaml`(제어·구동 파라미터), 검증 launch |

## 카메라 입력 방식 (detector `source`)

| 방식 | 실행 | 필요 | CPU (PC / RPi) |
|---|---|---|---|
| **A. 직접** (`source:=realsense`, 기본) | `perception.launch.py` 또는 `bringup.launch.py` | 시스템 python3의 `pyrealsense2` | 23% / 미측정 |
| **B. realsense2_camera** (`source:=ros`) | `camera.launch.py` + `detector source:=ros` | `ros-lyrical-realsense2-camera`, `ros-lyrical-cv-bridge` | 37% / 154% |

- RPi(aarch64, Python 3.14)는 pip용 pyrealsense2가 없어 **현재 B로 실행**한다. A는 librealsense 소스 빌드가 필요하다.
- 카메라는 한 프로세스만 연다 — A와 B, `realsense-viewer`를 동시에 띄우지 않는다.

## 빌드

```bash
cd ~/ws                                   # ws/src/Xmen 에 이 저장소
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to target_bringup   # xmen_vision 5개만
source install/setup.bash                 # 새 터미널마다
```
- `build/ install/ log/`는 경로·CPU 종류에 묶여 있다. **다른 PC·RPi로는 소스만 옮기고 다시 빌드한다.** ws 위치를 옮겼으면 `rm -rf build install log` 후 빌드.
- 필요 패키지: `python3-opencv python3-numpy python3-yaml python3-colcon-common-extensions`

## 실행 — PC (방식 A)

```bash
ros2 launch target_bringup bringup.launch.py              # 카메라·검출 → 제어 → 구동(OFF)
ros2 launch target_bringup bringup.launch.py show:=true   # + 검출 화면
ros2 launch target_perception perception.launch.py        # 인지만
```

## 실행 — Raspberry Pi (방식 B)

**1. 소스 복사 (PC에서)** — 커밋 전 파일도 옮기려면 rsync
```bash
rsync -av --exclude .git ~/git/opencv/opencv/ws/src/Xmen <user>@<RPi IP>:~/ws/src/
```

**2. 설치·빌드 (RPi)**
```bash
sudo apt install python3-opencv python3-numpy python3-yaml python3-colcon-common-extensions \
                 ros-lyrical-realsense2-camera ros-lyrical-cv-bridge
cd ~/ws && source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to target_bringup
```

**3. 실행 (터미널 4개, 각각 `source /opt/ros/lyrical/setup.bash && source ~/ws/install/setup.bash`)**
```bash
ros2 launch target_perception camera.launch.py viewer:=false           # 1 카메라
ros2 run target_perception detector --ros-args -p source:=ros           # 2 검출
ros2 run target_control controller --ros-args \
  --params-file ~/ws/src/Xmen/xmen_vision/target_bringup/config/tracking.yaml   # 3 제어
ros2 run target_control motor_driver --ros-args \
  --params-file ~/ws/src/Xmen/xmen_vision/target_bringup/config/tracking.yaml   # 4 구동(OFF)
```
`bringup.launch.py`는 방식 A로 detector를 띄우므로 RPi(B)에서는 쓰지 않는다.
카메라는 USB3(파란) 포트에 꽂는다.

## 확인

```bash
ros2 topic hz /target --qos-reliability best_effort    # 약 30 Hz
ros2 topic hz /gimbal/cmd_vel                           # 20 Hz
ros2 topic echo /tracking_status                        # 물체를 비추면 TRACKING
ros2 node info /tracking_controller                     # 받는·내는 토픽
top -p $(pgrep -d, -x detector),$(pgrep -d, -f realsense2_camera_node)   # CPU (100% = 코어 1개)
vcgencmd measure_temp; vcgencmd get_throttled           # RPi 온도·클럭 저하(0x0 정상)
```
`ros2 topic hz`도 CPU를 쓰므로 CPU는 따로 잰다.

## 검증 (모터 출력 OFF, 모의 입력)

| 실행 | 내용 |
|---|---|
| `ros2 launch target_bringup test_inputs.launch.py [tilt:=false]` | 모의 `/target` 11단계 → PASS/FAIL |
| `ros2 launch target_bringup sign_demo.launch.py [cmd_sign:=1.0] [cmd_sign_tilt:=-1.0]` | 가상 짐벌로 부호 정상·반대 |
| `ros2 launch target_bringup search_test.launch.py` | `/search` 5가지 경우 |
| `ros2 launch target_bringup e2e_check.launch.py [duration:=30.0 tag:=…]` | 실제 카메라(방식 A) + 인터페이스 검사 |

시험 노드는 `/target`을 모의로 발행한다 — 실제 detector와 섞이지 않게 `export ROS_DOMAIN_ID=<다른 번호>`.
결과는 `<ws>/results/`.

## 실측

| 항목 | PC (방식 A) | RPi (방식 B) |
|---|---|---|
| `/target` | 29.96 Hz | 29.7 Hz (간격 17~54 ms) |
| `/gimbal/cmd_vel` | 20.0 Hz | 20.0 Hz (간격 43~54 ms) |
| CPU detector / 카메라 노드 | 23% / — | 94% / 61% (보드 전체 약 50%) |
| 메모리 | — | 약 1.0 GB / 3.7 GB |
| 검증 | test_inputs 11/11, search_test 5/5, sign_demo 통과 | — |

RPi detector가 코어 1개를 거의 다 쓴다(Python은 사실상 1코어). 100%에 닿으면 `/target` Hz가 떨어지고
controller가 오래된 입력을 버려 `TIMEOUT`(정지)으로 간다
