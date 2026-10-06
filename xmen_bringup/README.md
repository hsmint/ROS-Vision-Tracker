# xmen_bringup — 비전 기반 객체 추적 통합 (실행·인터페이스·검증)

ROS 2 Lyrical · Python(rclpy) · Intel RealSense D435 · 목표: 파란 3×3×6 cm 직육면체 (근거리 ~ 1 m)
RPi에서 검출·제어 계산 → OpenCR(Dynamixel 팬·틸트)로 명령.

```
[D435] ──▶ target_detector ─/target─▶ tracking_controller ─/cmd_vel─▶ motor_driver ─(시리얼, 기본 OFF)─▶ OpenCR
               └─/perception_status        ├─/tracking_status         (출력 OFF, 로그만)
                                           └─/search (액션)
```
노드별 수신·송신, 메시지 필드, QoS, 정지 규칙 → **[TOPICS.md](TOPICS.md)**

## 패키지

| 폴더 (담당) | 패키지 | 실행 파일 | 내용 |
|---|---|---|---|
| `xmen_bringup` (통합) | `tracking_interfaces` | — | `/search` 액션 정의 (`action/Search.action`) |
| `xmen_bringup` (통합) | `tracking_common` | — | `interface.py` — 토픽 이름·타입·QoS·상태 값 단일 정의 |
| `xmen_vision` (인지) | `target_perception` | `detector`, `camera_viewer` | 검출(`detector.py`), 설정(`config/detector.yaml`), `launch/perception.launch.py`, `launch/camera.launch.py` |
| `xmen_control` (제어) | `target_control` | `controller`, `motor_driver` | 팬·틸트 P 제어·상태·`/search` 서버, 하드웨어 출력(기본 OFF) |
| `xmen_bringup` (통합) | `target_bringup` | `input_test`, `gimbal_sim`, `search_test`, `interface_check` | 전체 실행 launch, `config/tracking.yaml`(제어·구동 파라미터), 검증 launch |

## 카메라 입력 방식 (detector `source`)

| 방식 | 실행 | 필요 | CPU (PC / RPi) |
|---|---|---|---|
| **A. 직접** (`source:=realsense`, 기본) | `perception.launch.py` 또는 `bringup.launch.py` | 시스템 python3의 `pyrealsense2` | 23% / 117% |
| **B. realsense2_camera** (`source:=ros`) | `bringup.launch.py source:=ros` 또는 `perception.launch.py source:=ros` | `ros-lyrical-realsense2-camera`, `ros-lyrical-cv-bridge` | 37% / 155% |

- **추적·시연·성능 측정은 A, bag 녹화는 B**(A는 영상 토픽을 내지 않는다). detector 뒤 로직은 같다.
- RPi(aarch64, Python 3.14)는 pip용 pyrealsense2가 없다 → A를 쓰려면 librealsense 소스 빌드(아래 "RPi에서 방식 A 준비").
- 카메라는 한 프로세스만 연다 — A와 B, `realsense-viewer`를 동시에 띄우지 않는다.

## 빌드

```bash
cd ~/ws                                   # ws/src/Xmen 에 이 저장소
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to target_bringup   # 위 5개만 (xmen_vision·xmen_control·xmen_bringup)
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

## 실행 — Raspberry Pi

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

**3. 실행 (RPi)**
```bash
source /opt/ros/lyrical/setup.bash && source ~/ws/install/setup.bash
ros2 launch target_bringup bringup.launch.py                        # 방식 A: 추적·시연 (pyrealsense2 소스 빌드 필요, 아래)
ros2 launch target_bringup bringup.launch.py source:=ros            # 방식 B: bag 녹화용 (영상 토픽 발행)
ros2 launch target_perception perception.launch.py [source:=ros]    # 인지만
```
pyrealsense2가 없는 상태에서 `source:=ros` 없이 실행하면 detector가 `No module named 'pyrealsense2'`로 종료된다.
노드를 따로 띄울 때는 `camera.launch.py viewer:=false`, `detector --ros-args -p source:=ros`,
`controller`·`motor_driver --ros-args --params-file ~/ws/src/Xmen/xmen_bringup/target_bringup/config/tracking.yaml`.
카메라는 USB3(파란) 포트에 꽂는다.

### RPi에서 방식 A 준비 (pyrealsense2 소스 빌드, 1회)

```bash
# 메모리 3.7 GB·swap 0 → 컴파일 중 메모리 부족(cc1plus Killed) 방지용 swap. 빌드 후 지워도 된다
sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile

sudo apt install git cmake build-essential pkg-config libssl-dev libusb-1.0-0-dev libudev-dev python3-dev
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git ~/librealsense
cd ~/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir build && cd build                          # 반드시 ~/librealsense/build 안에서
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3) \
         -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DFORCE_RSUSB_BACKEND=ON
make -j2                                         # easyloggingpp 경고(warning)는 무시해도 된다
sudo make install && sudo ldconfig

echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.14/dist-packages' >> ~/.bashrc
source ~/.bashrc && cd ~
python3 -c "import pyrealsense2 as rs; print(rs.__version__, len(rs.context().query_devices()))"   # 버전, 1
```
- `FORCE_RSUSB_BACKEND=ON`: 커널 패치 없이 USB로 카메라를 연다.
- `/usr/local`에 설치되므로 apt의 `ros-lyrical-realsense2-camera`(방식 B)와 함께 둘 수 있다.
- 설치 후 실행: `ros2 launch target_bringup bringup.launch.py` (`source:=ros` 없이). PYTHONPATH가 적용된 터미널에서 실행한다.

## 확인

```bash
ros2 topic hz /target --qos-reliability best_effort    # 약 30 Hz
ros2 topic hz /cmd_vel                           # 20 Hz
ros2 topic echo /tracking_status                        # 물체를 비추면 TRACKING
ros2 node info /tracking_controller                     # 받는·내는 토픽
top -p $(pgrep -d, -x detector),$(pgrep -d, -f realsense2_camera_node)   # CPU (100% = 코어 1개)
top -H -p $(pgrep -x detector)                          # 스레드별 CPU
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

| 항목 | PC (방식 A) | RPi (방식 B) | RPi (방식 A) |
|---|---|---|---|
| `/target` | 29.96 Hz | 29.7 Hz (간격 17~54 ms) | 미측정 |
| `/cmd_vel` | 20.0 Hz | 20.0 Hz (간격 43~54 ms) | 미측정 |
| CPU 프로세스 | detector 23% | detector 94% + 카메라 노드 61% = **155%** | detector **117%** |
| CPU Python 처리 스레드 | — | 94% (detector 전체) | **약 70%** (`realsense_loop`) |
| 보드 전체 유휴 | — | 약 50% | 약 60% |
| 메모리 | — | 약 1.0 GB / 3.7 GB | 약 0.8 GB / 3.7 GB |
| 검증 | test_inputs 11/11, search_test 5/5, sign_demo 통과 | 상태 TRACKING ↔ LOST 전환 확인 | — |

CPU 100% = 코어 1개, RPi는 4코어(최대 400%).

**해석**
- Python 노드는 GIL 때문에 사실상 코어 1개로 처리한다. 이 스레드가 100%에 닿으면 `/target` Hz가 떨어지고
  controller가 오래된 입력(`too_old`)을 버려 `TIMEOUT`(정지)으로 간다 — 위험하게 움직이지는 않지만 추적이 끊긴다.
- 방식 B는 detector가 94%로 한계에 가까웠다. 영상을 ROS 메시지로 만들고(카메라 노드) 다시 푸는(cv_bridge) 비용이 있다.
- 방식 A는 프로세스 합계가 117%로 B(155%)보다 약 25% 적다. 100%를 넘는 것은 librealsense의 C++ 스레드
  (USB 수신·디코딩·정렬 보조, `top -H`에서 `detector`·`Thread-1 (reals…` 11%×3 등)가 같은 프로세스 안에서 여러 코어로
  돌기 때문이며, 1코어 제한을 받는 Python 처리 스레드는 약 70% → **약 30% 여유**.
- 처리 스레드 70% 중 큰 부분은 매 프레임 화면 전체의 뎁스 정렬(`rs.align`)이다. 더 줄여야 하면 색 후보가 없는 프레임에서
  정렬을 건너뛰는 방법이 있다(미적용).
