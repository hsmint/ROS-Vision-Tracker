# param — 설정 파일 설명

`hardware_launch.py`가 읽는 설정 파일 네 개를 정리한다. 튜닝 값은 코드가 아니라 이 YAML에 두므로, 재빌드 없이 파일만 바꿔서 조정한다.

| 파일 | 읽는 쪽 | 런치 인자 | 내용 |
| --- | --- | --- | --- |
| [`tracker.yaml`](tracker.yaml) | `tracker_node` | `params_file` | 추종 제어 게인 · 주기 · 타임아웃 |
| [`detector.yaml`](detector.yaml) | `tracker_node` (`detector_config`) | `detector_config` | 큐브 검출(HSV · 크기 · 모양 · 뎁스) |
| [`control.yaml`](control.yaml) | `control_lite` | `control_params_file` | 모터 구동 노드 설정 |
| [`bag_qos.yaml`](bag_qos.yaml) | `ros2 bag play` | (`bag_path`를 줄 때 자동 적용) | 재생할 영상 토픽의 QoS |

다른 파일을 쓰려면 런치 인자로 경로를 넘긴다.

```bash
ros2 launch xmen_bringup hardware_launch.py detector_config:=/path/to/my_detector.yaml
```

---

## tracker.yaml

`tracker_node`의 ROS 파라미터. 화면 중심 오차(ex, ey ∈ [-1, 1])를 팬 · 틸트 각속도(`/cmd_vel`)로 바꾼다.
명령 = `clamp(sign × kp × 오차, ±max_speed)`, |오차| < `deadband`면 0.

### 팬(좌우, `angular.z`)

| 파라미터 | 값 | 단위 | 의미 |
| --- | --- | --- | --- |
| `kp` | 1.5 | (rad/s) / 정규화 오차 | 비례 게인. ex = 0.4 → 0.6 rad/s |
| `cmd_sign` | -1.0 | — | 명령 부호. REP-103에서 `angular.z` + = 왼쪽이므로, 목표가 오른쪽(ex > 0)이면 음수 명령 |
| `deadband` | 0.05 | 정규화 오차 | \|ex\| < 0.05면 정지 — 중심 근처 떨림 방지 |
| `max_speed` | 0.9 | rad/s | 명령 상한(포화) |

### 틸트(상하, `angular.y`)

| 파라미터 | 값 | 단위 | 의미 |
| --- | --- | --- | --- |
| `tilt_enabled` | true | — | false면 팬 1축만 제어 |
| `kp_tilt` | 1.2 | (rad/s) / 정규화 오차 | 세로 화각이 좁아(ey = 1 ≈ 21°, 팬 ex = 1 ≈ 35°) 팬보다 작게 |
| `cmd_sign_tilt` | 1.0 | — | REP-103에서 `angular.y` + = 아래. 목표가 아래(ey > 0)면 양수 명령 |
| `deadband_tilt` | 0.05 | 정규화 오차 | \|ey\| < 0.05면 정지 |
| `max_speed_tilt` | 0.6 | rad/s | 중력이 걸리는 축이라 팬보다 낮게 |

### 주기 · 타임아웃

| 파라미터 | 값 | 단위 | 의미 |
| --- | --- | --- | --- |
| `tracking_hz` | 20.0 | Hz | 검출 주기. 가장 최근의 동기화된 RGB · 뎁스 쌍만 처리 |
| `rate_hz` | 20.0 | Hz | `/cmd_vel` · 상태 발행 주기. 입력이 없어도 0을 계속 보낸다 |
| `timeout` | 0.5 | s | 마지막 입력을 **받은 시각**에서 이만큼 지나면 명령 0 (30 fps에서 약 15프레임) |
| `max_input_age` | 0.5 | s | 영상의 `header.stamp`가 이보다 오래되면 신선하지 않은 입력으로 보고 명령 0 |
| `stall_timeout` | 0.5 | s | 이 시간 동안 영상이 없으면 상태를 `CAMERA_STALL`로 발행 |

---

## detector.yaml

`tracker_node`가 `detector_config`로 읽는 검출 설정. 목표는 파란 30×30×60 mm 직육면체, 거리 0.1~1.0 m, 어두움~밝음 조명이다. 검출은 색 마스크 → 면적 → 모양 → 상자 판정 → 뎁스 검증 순서로 진행된다.

### target — 무엇을 찾는가

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `name` | blue_cube | 사진 · 결과 JSON에 기록되는 목표 이름 |
| `hsv_ranges` | lower [104, 200, 8]<br>upper [120, 255, 255] | 목표 색의 OpenCV HSV 범위(H 0~179, S · V 0~255). 큐브 H 112 ± 8. S 하한 200은 배경(보라 의자 · 청회색 카펫, S 최대 약 163)을 거르는 값. 범위를 여러 개 적으면 합집합 |
| `min_chroma` | 5 | 색 세기(max − min) 하한. 어두운 픽셀에서 잡음으로 커진 S를 버린다 |
| `size_mm` | [30, 30, 60] | 목표 크기(가로 · 세로 · 높이, mm). 면적 기준 자동 계산에 쓴다 |
| `distance_range_m` | [0.1, 1.0] | 목표가 있을 거리 범위(m). 면적 기준 자동 계산에 쓴다 |

### camera — 카메라 설정

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `backend` | realsense | `realsense`(pyrealsense2) 또는 `v4l2`(OpenCV, 예비용) |
| `device` | 6 | v4l2 장치 번호. `backend: v4l2`일 때만 사용 |
| `fps` | 30 | 프레임률 |
| `warmup_frames` | 30 | 자동 노출 · 화이트밸런스 안정화를 위해 처음에 버리는 프레임 수 |
| `exposure` | null | 컬러 노출(1~10000). null = 자동 — 조명 변화를 카메라가 흡수 |
| `white_balance` | 4600 | 화이트밸런스(2800~6500 K). 고정해야 색상 H가 흔들리지 않는다. 4600 K는 실험실에서 잰 자동값 |
| `options.auto_exposure_priority` | 1 | 1 = 어두우면 프레임률을 낮춰서라도 노출을 늘림(30 → 15 fps 가능) |
| `options.backlight_compensation` | 0 | 역광 보정 끔 |
| `fx_px` | 460.0 | 컬러 초점거리(px, `fx_width` 기준). 640×360은 약 460. RealSense 실행 시 실제 값으로 덮어쓴다 |
| `fx_width` | 640 | `fx_px`의 기준 영상 폭(px) |

### processing — 전처리

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `width` | 640 | 처리 해상도 폭. 다른 크기 입력은 이 크기로 바꿔 처리 |
| `height` | 360 | 처리 해상도 높이. 비율이 다르면(640×480) 가운데를 16:9로 잘라 쓴다 |
| `blur_ksize` | 5 | 가우시안 블러 커널 크기(홀수, 0이면 생략) |
| `morph_kernel` | 5 | 모폴로지 커널 크기 |
| `open_iterations` | 1 | 열림 횟수 — 작은 점 잡음 제거 |
| `close_iterations` | 2 | 닫힘 횟수 — 물체 내부 구멍 · 틈 메우기 |

### selection — 후보 거르기와 고르기

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `min_area` | auto | 뎁스가 없을 때의 최소 면적(px). auto = `min_area_factor` × (1 m에서 30×30 면의 넓이) |
| `min_area_factor` | 0.3 | 회전 · 가림 · 원근 여유. 640×360 · fx 460 · 1 m에서 약 57 px |
| `max_area_ratio` | auto | 화면 대비 최대 면적 비. auto = 0.1 m에서 30×60 면 기준(0.6~0.95) |
| `max_aspect` | 3.0 | 외접 사각형 긴 변 / 짧은 변 상한. 옆면은 2:1 — 긴 줄무늬 · 막대 제거 |
| `min_extent` | 0.60 | 면적 / 최소 외접 사각형 면적 하한. 홈이 있거나 구불구불한 덩어리 제거(큐브 0.84~0.99, 파란 브래킷 0.52) |
| `min_solidity` | 0.75 | 면적 / 볼록껍질 면적 하한. 오목한 물체 제거(큐브 0.91~1.00, 브래킷 0.67) |
| `tie_ratio` | 0.1 | 후보가 여럿이면 가장 큰 것을 고르되, 면적 차이가 10% 이내면 화면 중심에 가까운 것 |
| `center_method` | hull | 중심 계산 방식. 볼록껍질 중심 — 하이라이트 · 그늘에 덜 끌려간다 |

#### selection.box_fit — 상자 모양 판정

실루엣(볼록껍질)을 꼭짓점 `max_vertices` 이하 다각형으로 근사했을 때, 평균 틈이 `base + pixel / √넓이` 이하여야 상자로 본다. 원 · 타원 · 고리를 거른다.

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `enabled` | true | 상자 판정 사용 여부 |
| `max_vertices` | 6 | 근사 다각형 꼭짓점 상한. 상자 실루엣은 4(정면)~6(비스듬)각형 |
| `base` | 0.02 | 허용 틈의 기본값 |
| `pixel` | 0.7 | 작은 실루엣일수록 틈을 더 허용하는 계수(500 px → 0.051, 5,000 px → 0.030) |
| `min_px` | 500 | 이보다 작은 실루엣(약 0.7 m보다 멂)은 해상도가 부족해 검사하지 않는다 |

#### selection.partial — 일부만 보이는 큐브

모양 · 넓이 기준에 걸려도 그 이유가 화면 가장자리에 잘림(border)이나 앞 물체에 가림(occluded)으로 설명되면 기준을 완화한다. 크기 상한은 그대로 둔다.

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `enabled` | true | 일부 보임 완화 사용 여부 |
| `border_px` | 2 | 화면 가장자리에서 이 픽셀 이내면 잘린 것으로 본다 |
| `border_max_angle_deg` | 30 | 길쭉한 덩어리는 긴 변이 가장자리와 30° 이내로 나란해야 잘린 것(가로 띠 오검출 방지) |
| `max_aspect` | 8.0 | 완화된 장단비 상한(큐브 폭의 1/3만 보이면 약 7.6) |
| `min_extent` | 0.35 | 가림으로 설명될 때의 채움비 하한 |
| `min_solidity` | 0.5 | 가림으로 설명될 때의 solidity 하한 |
| `front_margin_m` | 0.01 | 큐브보다 max(1 cm, 거리 × `front_margin_ratio`) 이상 가까우면 '앞 물체' |
| `front_margin_ratio` | 0.015 | 위 기준의 거리 비례 항(D435 뎁스 오차보다 크고 손가락 두께보다 작게) |
| `front_ratio` | 0.6 | 오목부 뎁스의 60% 이상이 앞 물체면 가림으로 본다 |
| `ring_px` | 4 | 둘레 검사 폭(px) |
| `ring_front_ratio` | 0.2 | 바로 바깥 둘레의 20% 이상이 앞 물체면 가림(곧게 잘린 가림) |
| `occluder_exclude_ranges` | lower [95, 60, 20]<br>upper [130, 255, 255] | 이 HSV 범위(파란 계열)는 가린 물체로 치지 않는다 — 청바지 주름 오검출 방지 |
| `min_area_m2` | 0.0001 | 일부만 보일 때 실제 넓이 하한 1 cm²(평소 3 cm²) |

### depth — 뎁스 검증

컨투어 안 뎁스의 중앙값 Z로 실제 거리와 크기를 계산해 후보를 검증한다(RealSense만). 실제 길이 = 픽셀 길이 × Z / f.

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `enabled` | true | 뎁스 검증 사용 여부 |
| `min_m` | 0.1 | 최소 거리(m). 이보다 가까워 뎁스가 0이면 색 · 면적 규칙으로 판단 |
| `max_m` | 1.1 | 최대 거리(m). 목표 1.0 m + 여유(1.0 m 실측 Z 1.01~1.03) |
| `min_valid_ratio` | 0.3 | 컨투어 안 유효(0이 아닌) 뎁스 픽셀 비율 하한 |
| `erode` | 1 | 경계 픽셀을 빼기 위한 침식 횟수(3×3) |
| `min_area_px` | 60 | 뎁스가 있을 때의 최소 면적(px, 잡음 바닥). 1 m 실측 최소 211 px |
| `no_depth_min_area` | 800 | 뎁스 측정 실패(너무 가까움 · 반사) 시 색만으로 믿을 최소 면적(px) |
| `max_short_m` | 0.06 | 실제 짧은 변 상한(m). 모서리가 카메라를 향한 자세(5.0~5.3 cm)까지 허용 |
| `max_long_m` | 0.08 | 실제 긴 변 상한(m) |
| `area_m2` | [0.0003, 0.0030] | 실제 넓이 범위 3~30 cm² |
| `split` | true | 같은 색 물체와 겹쳐 크기가 안 맞는 덩어리를 뎁스 경계에서 나눠 다시 검증 |
| `split_jump_m` | 0.02 | 이웃 픽셀 뎁스 차가 이보다 크면 다른 물체로 본다(m) |
| `split_jump_ratio` | 0.03 | 먼 거리 잡음 대비. 기준 = max(`split_jump_m`, 거리 × 이 값) |
| `split_min_valid` | 0.6 | 덩어리 안 유효 뎁스 비율이 이보다 낮으면 나누지 않는다 |

---

## control.yaml

`control_lite`(모터 구동 노드)의 ROS 파라미터.

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `use_sim_time` | false | 시스템 시계를 쓴다(실제 하드웨어 구동) |

파일에 없는 설정:

- 시리얼 포트는 런치 인자 `port`로 넘긴다(기본 `/dev/ttyACM0`).
- 통신 속도는 115200 baud로 고정이다.
- 시작할 때 항상 한 번 원점(홈)을 잡는다.

---

## bag_qos.yaml

`ros2 bag play`로 녹화 영상을 재생할 때 토픽의 QoS를 덮어쓴다. 발행자와 구독자의 QoS가 맞지 않으면 연결되지 않으므로, `tracker_node`의 영상 구독 설정(reliable)에 맞춘다. `hardware_launch.py`에 `bag_path`를 주면 자동으로 적용된다.

적용 토픽(두 토픽 모두 같은 값):

- `/camera/color/image_raw` — 컬러 영상
- `/camera/aligned_depth_to_color/image_raw` — 컬러에 정렬된 뎁스 영상

| 파라미터 | 값 | 의미 | 반대 값 | 이 값을 쓰는 이유 |
| --- | --- | --- | --- | --- |
| `reliability` | `reliable` | 유실되면 재전송해 반드시 전달 | `best_effort` — 유실돼도 재전송 안 함 | tracker 구독 설정과 맞춰야 연결됨 |
| `durability` | `volatile` | 늦게 접속한 구독자에게 지난 메시지를 안 보냄 | `transient_local` — 마지막 메시지를 보관해 전달 | 영상은 지난 프레임이 필요 없음 |
| `history` | `keep_last` | 큐에 최근 N개만 보관 | `keep_all` — 전부 보관 | 메모리가 끝없이 쌓이지 않음 |
| `depth` | `5` | `keep_last`의 N. 최근 5프레임까지 보관 | — | 처리가 밀리면 오래된 프레임부터 버림 |
