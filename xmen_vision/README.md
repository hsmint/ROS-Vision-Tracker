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
| 다른 파란 물체 오검출 | 목표 없는 장면의 검출 | `hsv_ranges` S 하한↑, `min_extent`·`min_solidity` 엄격히 |
| 사각 면이 아닌 파란 물체(원·타원·고리) 오검출 | 탈락 사유에 `not_box`가 없음 | `selection.box_fit` (`base`, `pixel` 낮추면 엄격, `min_px` 낮추면 먼 거리까지 검사) |
| 비스듬한·모서리 방향 큐브를 놓침 | 탈락 `not_box`·`size_mismatch` | `box_fit.base`·`pixel`↑, `depth.max_short_m`(이론 최대 5.4 cm) |
| 화면 가장자리·손에 가려 일부만 보일 때 놓침 | 탈락 `bad_aspect`·`low_extent`·`size_mismatch` | `selection.partial` (`max_aspect`, `front_margin_*`, `front_ratio`, `ring_front_ratio`, `min_area_m2`) |

사각 면으로 된 상자인지(`selection.box_fit`): 상자 실루엣은 어느 방향에서도 꼭짓점 4~6개 볼록 다각형이다.
볼록껍질을 6각형 이하로 근사했을 때의 평균 틈(넓이 차 ÷ 둘레 ÷ √넓이)이 크면 곡선 테두리(원·타원·고리)로 보고 `not_box`.
실루엣이 `min_px`(500 px, 약 0.7 m)보다 작으면 해상도가 부족해 검사하지 않는다. 반원처럼 직선 변이 있는 곡선 물체는 통과한다.

일부만 보이는 큐브(`selection.partial`): 모양·넓이 하한에 걸려도 **화면 가장자리에 잘렸거나**(border)
**뎁스로 본 오목부·둘레가 큐브보다 가까울 때**(손가락 등 앞 물체, occluded)만 기준을 완화한다. 크기 상한은 그대로.
홈·구멍이 있는 다른 물체는 오목부로 뒤쪽 배경이 보여 완화되지 않는다. 검출 화면 둘째 줄에 `PARTIAL(border|occluded)`, 기록 JSON에 `partial`.

설정을 고친 뒤 노드만 다시 켜면 적용된다(재빌드 불필요).
