# 문제 5 — bag 재현과 팀 협업

## 1. 구현 내용

| 항목 | 내용 | 위치 |
|---|---|---|
| 기록 | 대표 성공 장면·소실/복귀 장면을 영상·뎁스·목표·명령·관절 토픽과 함께 기록 | `~/bags/p5_success`, `~/bags/p5_loss_return` |
| 메타데이터 | bag마다 `metadata.yaml` + `bag_info.txt`(해상도·토픽·메시지 수·기간) + 실행 당시 설정 3종 + `uncommitted.diff` | 각 bag 폴더 |
| 입력 재처리 | bag 영상만 재생 → `tracker_node`로 다시 검출 → `/target_replay` 등 별도 토픽으로 기록 | `~/bags/p5_*_replay` |
| 결과 재분석 | 저장된 `/target`·`/cmd_vel`에서 FPS·검출 출력 비율·RMSE·소실 이벤트 재계산, 재처리 결과와 프레임별 대조 | `replay_info.txt`, `compare.csv` |
| 다른 팀원 실행 | 다른 PC에서 `p5_success` 입력 재처리 + RViz 확인 | 4절, `pic/third_party_replay_rviz.png` |

## 2. 기록 (bag)

### 2.1 기록 조건

| 항목 | 값 |
|---|---|
| 해상도 | 640 × 360 (컬러·정렬 뎁스) |
| 카메라 설정 FPS | 30 |
| 저장 형식 | mcap |
| 검출 설정 | `detector.yaml` (파란 30×30×60 mm 직육면체, H 104~120 · S 200~255 · V 8~255) |
| 추적 설정 | `tracker.yaml` — `tracker_type: fusion`, Kp 1.5 (팬) / 1.2 (틸트), 데드밴드 0.05, 속도 상한 0.9 / 0.6 rad/s, 처리 15 Hz, 명령 20 Hz, 타임아웃 0.5 s |
| 구동 설정 | `control.yaml` — `control_lite`, 115200 baud, 시작 시 홈 복귀 |
| 실제 구동 | 녹화 중 모터 출력 켬(폐루프). `/joint_states` 약 48 Hz. 팬 −0.37~−0.06 rad · 틸트 −0.17~0.20 rad (`p5_success`), 팬 −0.03~0.06 rad · 틸트 −0.06~0.37 rad (`p5_loss_return`) |

기록 명령 (보드):

```bash
ros2 bag record -s mcap -o ~/bags/p5_success \
  /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info \
  /target /cmd_vel /perception_status /joint_states
```
(확인 필요 — 실제 사용한 명령으로 교체)

### 2.2 bag 메타데이터

| bag | 장면 | 기간 [s] | 크기 | 메시지 수 | SHA-256 (mcap) |
|---|---|---|---|---|---|
| `p5_success` | 대표 성공 | 47.81 | 912.0 MiB | 4864 | `f2d9a2c3…8b7594638` |
| `p5_loss_return` | 소실·복귀 | 29.48 | 575.0 MiB | 3101 | `34fc72b6…1e63938` |

<details><summary>SHA-256 전체</summary>

```
f2d9a2c33547641fc7f84f8fb68dab871acf54decc5ad8fee0d890e072835fdb  p5_success/0_p5_success_2026_10_08-10_13_52.mcap
34fc72b612c7951e1482b66602929717cd66bacc1768d0ef6c9fc4c8b7594638  p5_loss_return/0_p5_loss_return_2026_10_08-10_24_02.mcap
4e73e3c181ef73643a27947f4c27bcf80aef67b797c31f2d99673df3a8744ffc  p5_success_replay/0_p5_success_replay_2026_10_08-11_56_18.mcap
481a5b2b612f663c9b8f0e337e6ce3368b89c8b27f5075fc82806bb4f1e63938  p5_loss_return_replay/0_p5_loss_return_replay_2026_10_08-11_58_16.mcap
```
</details>

| 토픽 | 타입 | `p5_success` | `p5_loss_return` |
|---|---|---|---|
| `/camera/color/image_raw` | sensor_msgs/Image | 594 | 374 |
| `/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 591 | 373 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 592 | 374 |
| `/target` | geometry_msgs/PointStamped | 353 | 247 |
| `/cmd_vel` | geometry_msgs/Twist | 418 | 326 |
| `/perception_status` | std_msgs/String | 42 | 54 |
| `/joint_states` | sensor_msgs/JointState | 2274 | 1353 |


## 3. 재현

두 재현 모두 **모터 출력 없이** 수행했다(PC에서 control 미실행, 보드 연결 없음). 원본 bag은 실제 모터를 구동하며 기록한 것이고(2.1), 재현은 오프라인이다.

| 재현 | 수행 내용 | 확인 |
|---|---|---|
| 입력 재처리 | bag의 영상 3토픽만 재생 → `tracker_node`가 다시 검출, 출력은 `/target_replay`·`/cmd_vel_replay`·`/perception_status_replay` | 같은 설정에서 검출·오차 경향이 재현되는가 |
| 결과 재분석 | bag에 저장된 `/target`·`/cmd_vel`만으로 지표 재계산 | 기존 성능표와 같은가 |

### 3.1 입력 재처리 명령

```bash
# 터미널 1 — 검출기 (원본 /target과 섞이지 않게 remap, bag 시간 사용)
ros2 run xmen_tracker tracker_node --ros-args \
  --params-file ~/bags/p5_success/tracker.yaml \
  -p detector_config:=$HOME/bags/p5_success/detector.yaml \
  -p use_sim_time:=true \
  -r /target:=/target_replay -r /cmd_vel:=/cmd_vel_replay \
  -r /perception_status:=/perception_status_replay

# 터미널 2 — 영상만 재생, 촬영 시각을 /clock으로
ros2 bag play ~/bags/p5_success --clock-topics /camera/color/image_raw \
  --qos-profile-overrides-path ~/bags/replay_qos.yaml \
  --topics /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info

# 터미널 3 — 재처리 결과 기록
ros2 bag record -s mcap -o ~/bags/p5_success_replay \
  /target_replay /cmd_vel_replay /perception_status_replay
```

- 저장된 `/target`·`/cmd_vel`은 `--topics`로 재생하지 않으므로 새 결과와 같은 토픽에 섞이지 않는다.
- `--clock-topics`로 촬영 시각이 `/clock`이 되고 tracker는 `use_sim_time:=true`이므로, 타임아웃·지연에 현재 벽시계가 섞이지 않는다.

### 3.2 결과 — 대표 성공 (`p5_success`)

| 지표 | 원본 `/target` (결과 재분석) | 재처리 `/target_replay` (입력 재처리) |
|---|---|---|
| 프레임 수 | 353 (47.13 s) | 382 (47.75 s) |
| 처리 FPS | 7.47 | 7.98 |
| 검출 출력 비율 | 87.8% (310 / 353) | 92.1% (352 / 382) |
| 수평 RMSE (ex) | 0.079 (검출 310) | 0.080 (검출 352) |
| 수직 RMSE (ey) | 0.110 | 0.128 |
| 최대 \|ex\| | 0.355 | 0.357 |
| 소실 | 8회 | 6회 |

같은 촬영 시각 프레임 대조 (`compare.csv`):

| 항목 | 값 |
|---|---|
| 짝 프레임 | 222 |
| 검출 여부 일치 | 95.0% (211 / 222). 불일치 11개는 모두 원본 미검출 → 재처리 검출 |
| \|Δex\| 평균 / 최대 (둘 다 검출 196개) | 0.0006 / 0.0054 |

### 3.3 결과 — 소실·복귀 (`p5_loss_return`)

| 지표 | 원본 `/target` (결과 재분석) | 재처리 `/target_replay` (입력 재처리) |
|---|---|---|
| 프레임 수 | 247 (29.08 s) | 242 (27.62 s) |
| 처리 FPS | 8.46 | 8.72 |
| 검출 출력 비율 | 40.5% (100 / 247) | 42.1% (102 / 242) |
| 수평 RMSE (ex) | 0.124 (검출 100) | 0.078 (검출 102) |
| 수직 RMSE (ey) | 0.356 | 0.297 |
| 최대 \|ex\| | 0.985 | 0.130 |
| 소실 | 18회 (마지막 1회 재검출 없음) | 8회 (마지막 1회 재검출 없음) |
| 소실 후 정지까지 | 0.001~0.027 s | 0.000 s |

같은 촬영 시각 프레임 대조:

| 항목 | 값 |
|---|---|
| 짝 프레임 | 143 |
| 검출 여부 일치 | 81.8% (117 / 143). 불일치 26개 = 원본만 검출 7 · 재처리만 검출 19 |
| \|Δex\| 평균 / 최대 (둘 다 검출 41개) | 0.0010 / 0.0042 |

## 4. 다른 팀원 실행 기록

### 4.1 실행 명령 (원본 로그: [third_party_log.txt](third_party_log.txt))

```bash
# 저장소: ~/Documents/temp/ROS-Vision-Tracker, bag: ~/bags/p5/p5_success
ros2 run xmen_tracker tracker_node --ros-args \
  --params-file $HOME/bags/p5/p5_success/tracker.yaml \
  -p detector_config:=$HOME/bags/p5/p5_success/detector.yaml \
  -p use_sim_time:=true -p debug_timing:=true \
  -r /target:=/target_replay -r /cmd_vel:=/cmd_vel_replay \
  -r /perception_status:=/perception_status_replay

ros2 bag play ~/bags/p5/p5_success --clock
```

### 4.2 결과

![다른 PC 재처리 — RViz](pic/third_party_replay_rviz.png)

*다른 PC에서 `p5_success` 재처리 중인 RViz. 왼쪽 위 Tracking Image에 큐브 검출 박스, 오른쪽에 로봇 모델과 이미지 평면*

| 항목 | 값 |
|---|---|
| 검출 설정 로드 | HSV [104,200,8]~[120,255,255], min_area 57 px, depth 0.1~1.1 m — `detector.yaml`과 같음 |
| sim time | `now`가 bag 시각(1791422034~046 s)을 따라감, `stale 0` |
| 처리량 (2 s마다) | 동기화 40~45쌍 중 24~30개 처리 → 약 12~15 FPS (보드 원본 약 13.7 FPS, 기록 공백 제외) |

## 5. 해석

1. **검출기 자체는 재현된다.** 두 bag 모두 원본·재처리가 함께 검출한 프레임에서 |Δex| 최대 0.0054로, 같은 영상·같은 설정이면 중심 오차는 사실상 같다.
2. **차이는 "어떤 프레임을 처리했는가"에서 나온다.** 짝 프레임이 원본 프레임의 63%(222/353)·58%(143/247)뿐이다. tracker가 15 Hz로 가장 최근 프레임만 처리하므로(`tracking_hz: 15`) 보드와 PC에서 건너뛴 프레임이 달라진다. fusion 추적기는 KCF·SIFT 상태를 이어 쓰므로, 처리한 프레임이 다르면 다음 프레임의 검출 여부도 달라질 수 있다. 소실·복귀 bag에서 불일치(18.2%)가 큰 것은 목표가 가려지고 나타나는 경계 프레임이 많기 때문으로 보인다. (가설 — 확인 필요)
3. 소실·복귀 bag의 최대 |ex|가 0.985 → 0.130으로 크게 다른 것은 원본의 한 프레임(8.49 s) 때문이다. 화면 왼쪽 끝 모니터 가장자리를 면적비 0.0005로 잘못 검출한 배경 오검출이다(3.4 이미지). 이 프레임은 재처리에서 짝이 없어(그 촬영 시각을 처리하지 않음) 재처리 쪽 결과는 알 수 없다. |ex| > 0.3인 원본 검출은 이 1프레임뿐이다.
4. 처리 FPS가 7.5~8.7로 문제 4(29.98)보다 낮다. 다만 이 FPS는 기록 공백을 포함한 전체 시간으로 나눈 값이다. 문제 4는 `target_detector`, 문제 5는 `tracker_node`(fusion, 뎁스·SIFT 포함)로 검출기가 다르다. 문제 4 성능표와 수치를 직접 비교하지 않는다.
5. 명령 수가 원본 418 vs 재처리 961로 다르다. 원본은 기록 공백(약 21.4 s) 동안 명령이 거의 없었고(5개), 공백을 빼면 약 16 Hz(명령 간격 중앙값 0.062 s)였다. 재처리는 PC에서 sim time 기준 20 Hz 타이머가 그대로 돌았다. 공백을 뺀 나머지 차이(16 vs 20 Hz)는 보드 부하로 타이머가 밀린 것으로 보인다. (가설)
6. 원본 `/target`의 수신 − 촬영 시각은 중앙값 0.25 s / 0.17 s, 최대 0.41 s / 0.47 s였다(`p5_success` / `p5_loss_return`). 입력 신선도 한계 `max_input_age` 0.5 s에 가까워, 보드 부하가 조금만 늘어도 신선한 입력이 TIMEOUT으로 처리될 수 있다.
7. 소실 중 정지는 실제 모터로 확인된다. `p5_loss_return`의 의도한 가림(2.46~8.49 s, 6.0 s) 동안 마지막 명령으로 틸트가 0.006 rad 더 움직인 뒤(2.46~2.52 s), 나머지 구간은 팬·틸트 모두 0.003 rad 이내로 멈춰 있었다. `p5_success`의 미검출 구간(32.1~33.8 s)도 팬 0.002 rad, 틸트 0 rad 변화였다(`/joint_states` 48 Hz 기준). 

## 6. 입력 재처리 vs 결과 재분석

| | 입력 재처리 | 결과 재분석 |
|---|---|---|
| 입력 | bag의 **영상**(컬러·뎁스·camera_info) | bag의 **저장된 출력**(`/target`·`/cmd_vel`) |
| 검출기 실행 | 함 (현재 코드·설정) | 안 함 |
| 확인하는 것 | 검출·제어 코드가 같은 입력에 같은 결과를 내는가 (코드 회귀) | 당시 기록으로 계산한 지표가 성능표와 같은가 (계산 재현) |


## 7. 원본 파일

| 파일 | 내용 |
|---|---|
| `~/bags/p5_*/metadata.yaml`, `bag_info.txt` | bag 메타데이터 |
| `~/bags/p5_*/{detector,tracker,control}.yaml` | 녹화 당시 설정 |
| `~/bags/p5_*_replay/replay_info.txt` | 재처리 명령·원본/재처리 지표 |
| `~/bags/p5_*_replay/compare.csv` | 프레임별 대조 (`stamp_ns, det_orig, ex_orig, ey_orig, det_replay, ex_replay, ey_replay`) |
| `report/problem5/third_party_log.txt` | 다른 PC 재처리 터미널 로그 (bag play·tracker_node) |
| `report/problem5/pic/third_party_replay_rviz.png` | 다른 PC 재처리 RViz 화면 |
