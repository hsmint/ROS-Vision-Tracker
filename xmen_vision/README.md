# xmen_vision — 인지 (target_perception)

목표: 파란 30×30×60 mm 직육면체, 어두움~밝음, 0.1~1 m. D435 컬러·뎁스 640×360.

| 파일 | 내용 |
|---|---|
| `target_perception/detector.py` | 검출: HSV(엄격 + 연결된 완화 범위) → 컨투어 → 면적·모양 → 뎁스 거리·실제 크기 → 선택 → 볼록껍질 중심 |
| `target_perception/detector_node.py` | `detector` 노드 → `/target`, `/perception_status` |
| `target_perception/tuning.py` | `tuning` — HSV 튜닝, 장면 기록, 조건별 검출률 측정, 평가 데이터 수집 |
| `target_perception/evaluate.py` | `evaluate` — 조명×거리 인식률 평가, 설정 비교 |
| `config/detector.yaml` | 검출 설정(노드·도구 공용) |
| `data/` | `evaluate --sim` 기준 실제 영상(큐브 있음·없음) |

## 노드 실행
```bash
ros2 launch target_perception perception.launch.py show:=true   # 검출 화면(오버레이·mask)
```

## 튜닝·평가 도구
카메라는 한 프로세스만 연다 — **detector 노드(perception·bringup launch)를 끄고** 실행한다.
기본 설정은 패키지 `config/detector.yaml`(`--symlink-install`이면 저장소 파일을 바로 고친다),
결과는 `<ws>/results/perception/`.

```bash
ros2 run target_perception tuning tune --camera       # 드래그→HSV 제안, 트랙바, f=노출 고정, w=설정 기록
ros2 run target_perception tuning live --camera       # 실시간 확인, s=증거 저장
ros2 run target_perception tuning scenes --camera     # 보임 → 없음 → 가림, TP/TN 판정
ros2 run target_perception tuning bench --camera --label dark_1.0m        # 조건별 100프레임 검출률·Z·처리 시간
ros2 run target_perception tuning collect --camera --label dark_1.0m --light dark --distance 1.0
ros2 run target_perception tuning collect --camera --label dark_empty --light dark --absent   # 오검출 측정용
ros2 run target_perception evaluate --dataset ~/ws/results/perception/dataset --config A.yaml B.yaml
ros2 run target_perception evaluate --sim --ablation  # 실제 영상 1장 변형(모의) — 빠른 비교용
```

- `collect`는 큐브를 **받침대에 고정**하고, 정지 화면에서 큐브를 드래그한 뒤 Enter → 30장(컬러 + 뎁스) 저장.
- `evaluate`는 저장된 뎁스로 노드와 같은 검증(거리·실제 크기·겹침 분리)까지 한다. `--no-depth`는 색·모양만.
- 결과: `eval/eval_dataset.md`(조명×거리 인식률·오검출률·중심 오차), `eval/montage_*.png`(조건별 검출 화면).

권장 조건: 조명 {어두움, 보통, 밝음} × 거리 {0.1, 0.2, 0.5, 1.0 m} + 조명마다 큐브 없는 장면.

## 결과를 보고 고칠 곳 (`config/detector.yaml`)

| 증상 | 확인 | 고칠 값 |
|---|---|---|
| 어두울 때 놓침 | mask에 큐브가 안 나옴 | `hsv_ranges` V 하한, `min_chroma`, 또는 조명 |
| 밝을 때 큐브가 조각남 | 하이라이트가 mask에서 빠짐 | `hsv_ranges` S 하한, `relaxed_ranges` S 하한 |
| 1 m에서 놓침 | 탈락 `too_small` | `depth.min_area_px`, `selection.min_area_factor` |
| 0.1~0.2 m에서 놓침 | 탈락 `no_depth`·`size_mismatch` | `depth.no_depth_min_area`, `depth.max_short_m`·`max_long_m` |
| 다른 파란 물체 오검출 | 목표 없는 장면의 검출 | `hsv_ranges` S 하한↑, `max_aspect`·`min_extent` 엄격히 |

설정을 고친 뒤 노드만 다시 켜면 적용된다(재빌드 불필요).
