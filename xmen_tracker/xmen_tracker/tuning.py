"""tuning — 검출(detector.py) 튜닝·확인 도구. detector 노드와 같은 설정·같은 코드로 동작한다.

  ros2 run xmen_tracker tuning <모드> --camera [--config <yaml>] [--out <폴더>]

모드
  tune    HSV 트랙바 튜닝. w = 설정 파일에 HSV 범위 기록 + 근거(<out>/tune) 저장
  live    실시간 검출. s = 증거 저장(원본·mask·overlay·뎁스·JSON), q/ESC = 종료
  scenes  보임·없음·가림 세 장면을 같은 설정으로 검출하고 TP/FP/FN/TN 판정
  bench   조건 하나(조명·거리)마다 N프레임 검출률·거리·처리 속도 측정 (--label)
  collect 인식률 평가 데이터 수집(--label --light --distance [--absent]) → evaluate --dataset
입력
  --camera        설정의 camera(RealSense 컬러 + 뎁스) 사용
  --image A B ..  저장 영상. scenes 모드에서는 보임·없음·가림 순서로 3장
  (생략)          합성 영상 — 실제 카메라 촬영이 아님
기본값: --config = 패키지 config/detector.yaml(--symlink-install이면 저장소 파일), --out = <cwd>/results/perception
카메라는 한 프로세스만 연다 — realsense_node를 끄고 실행한다.
"""
import argparse
import json
import shutil
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np

from xmen_tracker import detector

SCENES = ['visible', 'absent', 'occluded']


def synthetic(scene):
    """examples/common.synthetic과 같은 구도 + 선택 규칙 확인용 방해 요소."""
    image = np.full((480, 640, 3), 220, np.uint8)
    cv2.rectangle(image, (60, 60), (70, 70), (255, 0, 0), -1)        # 작은 잡음 → too_small
    if scene != 'absent':
        cv2.rectangle(image, (240, 150), (400, 310), (255, 0, 0), -1)    # 목표
        cv2.rectangle(image, (480, 330), (530, 370), (255, 0, 0), -1)    # 작은 후보 → 면적 비교에서 짐
    if scene == 'occluded':
        cv2.rectangle(image, (325, 140), (420, 320), (220, 220, 220), -1)
    cv2.putText(image, 'SYNTHETIC - NOT A CAMERA CAPTURE', (10, 465),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (30, 30, 30), 1)
    return image


def apply_realsense_options(rs, sensor, c):
    """exposure/white_balance 숫자 = 해당 자동 기능을 끄고 고정. camera.options = 그 밖의 컬러 옵션 이름: 값."""
    for auto, key, opt in [(rs.option.enable_auto_exposure, 'exposure', rs.option.exposure),
                           (rs.option.enable_auto_white_balance, 'white_balance', rs.option.white_balance)]:
        if c.get(key) is not None:
            sensor.set_option(auto, 0)
            sensor.set_option(opt, float(c[key]))
    for name, value in (c.get('options') or {}).items():
        opt = getattr(rs.option, name, None)
        if opt is None or not sensor.supports(opt):
            print(f'경고: 컬러 센서가 {name} 옵션을 지원하지 않는다 — 무시')
            continue
        r = sensor.get_option_range(opt)
        v = min(max(float(value), r.min), r.max)
        sensor.set_option(opt, v)
        print(f'camera option {name} = {v} (범위 {r.min}~{r.max})')


@contextmanager
def camera(cfg):
    c = cfg['camera']
    if c['backend'] == 'realsense':
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        conf = rs.config()
        W, H = cfg['processing']['width'], cfg['processing']['height']
        conf.enable_stream(rs.stream.color, W, H, rs.format.bgr8, c.get('fps', 30))
        use_depth = detector.depth_cfg(cfg) is not None
        if use_depth:
            conf.enable_stream(rs.stream.depth, W, H, rs.format.z16, c.get('fps', 30))
        try:
            profile = pipe.start(conf)
        except RuntimeError as e:
            raise RuntimeError(f'RealSense 시작 실패: {e} — USB3 연결·다른 프로그램 점유를 확인한다.') from e
        sensor = profile.get_device().first_color_sensor()
        apply_realsense_options(rs, sensor, c)
        # 실제 컬러 초점거리로 크기 기반 면적 기준(min_area: auto 등)을 다시 계산한다(설정의 fx_px는 근사값)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        c['fx_px'], c['fx_width'] = float(intr.fx), int(intr.width)
        detector.resolve_size_limits(cfg)
        print(f"RealSense {intr.width}x{intr.height} fx={intr.fx:.1f}px → min_area={cfg['selection']['min_area']} "
              f"max_area_ratio={cfg['selection']['max_area_ratio']}")
        if use_depth:
            align = rs.align(rs.stream.color)      # 뎁스를 컬러 픽셀 좌표로 맞춘다(CPU 사용 가장 큼)
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        last = {'depth': None}
        for auto, key, opt in [(rs.option.enable_auto_exposure, 'exposure', rs.option.exposure),
                               (rs.option.enable_auto_white_balance, 'white_balance', rs.option.white_balance)]:
            if c.get(key) is not None:
                sensor.set_option(auto, 0)
                sensor.set_option(opt, float(c[key]))

        def read():
            frames = pipe.wait_for_frames()
            if use_depth:
                frames = align.process(frames)
                d = frames.get_depth_frame()
                last['depth'] = detector.DepthFrame(np.asanyarray(d.get_data()).copy(), depth_scale,
                                                    intr.fx, intr.fy) if d else None
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError('컬러 프레임이 없다.')
            return np.asanyarray(color.get_data()).copy()
        def settings():
##            return {k: sensor.get_option(o) for k, o in
##                    [('exposure', rs.option.exposure), ('white_balance', rs.option.white_balance)]}
            out = {}
            for k in ('exposure', 'white_balance', 'gain', 'enable_auto_exposure',
                      'enable_auto_white_balance', 'auto_exposure_priority'):
                o = getattr(rs.option, k, None)
                if o is not None and sensor.supports(o):
                    out[k] = sensor.get_option(o)
            return out
        read.settings = settings
        read.depth = lambda: last['depth']      # 직전 read()와 같은 시점의 정렬된 뎁스
        try:
            for _ in range(c['warmup_frames']):
                read()
            yield read
        finally:
            pipe.stop()
    else:
        cap = cv2.VideoCapture(c['device'], cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"/dev/video{c['device']}를 열 수 없다. 번호·USB·점유를 확인한다.")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg['processing']['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg['processing']['height'])

        def read():
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('카메라 프레임을 읽지 못했다.')
            return frame
        try:
            for _ in range(c['warmup_frames']):
                read()
            yield read
        finally:
            cap.release()


def imread(path):
    frame = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError(f'이미지를 읽을 수 없다: {path}')
    return frame


def save_png(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise RuntimeError('PNG 인코딩 실패')
    path.write_bytes(data.tobytes())


def process(raw, cfg, label='', depth=None):
    """반환 (처리 영상, Detection, mask, overlay, 처리 시간 ms). 시간은 리사이즈~중심 계산(표시 제외).
    depth는 raw와 같은 크기의 DepthFrame 또는 None."""
    t0 = time.perf_counter()
    frame, scale = detector.preprocess(raw, cfg)
    det, mask = detector.detect(frame, cfg, scale, detector.preprocess_depth(depth, scale))
    ms = (time.perf_counter() - t0) * 1000
    return frame, det, mask, detector.draw(frame, det, label), ms


def camera_settings(read):
    return read.settings() if hasattr(read, 'settings') else None


def depth_of(read):
    """직전 프레임의 정렬된 뎁스. 뎁스가 없는 입력(v4l2·이미지·합성)이면 None."""
    return read.depth() if hasattr(read, 'depth') else None


def record(det, raw, cfg, source, **extra):
    return dict(target=cfg['target']['name'], source=source,
                processing_size=[cfg['processing']['width'], cfg['processing']['height']],
                hsv_ranges=cfg['target']['hsv_ranges'],
                **extra, **det.summary())


def save_evidence(out, name, raw, mask, overlay, info, depth=None):
    save_png(out / f'{name}_original.png', raw)
    if depth is not None:
        save_png(out / f'{name}_depth.png', depth.data)   # uint16 원값, × depth_scale = m
    save_png(out / f'{name}_mask.png', mask)
    save_png(out / f'{name}_overlay.png', overlay)
    (out / f'{name}.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')


def run_live(a, cfg):
    if not a.camera and not a.image:
        raise SystemExit('live 모드는 --camera 또는 --image가 필요하다.')
    with (camera(cfg) if a.camera else nullreader(a.image[0])) as read:
        while True:
            raw = read()
            depth = depth_of(read)
            frame, det, mask, overlay, ms = process(raw, cfg, depth=depth)
            if a.no_gui:
                save_evidence(a.out, 'live', raw, mask, overlay,
                              record(det, raw, cfg, 'camera' if a.camera else str(a.image[0]),
                                     camera_settings=camera_settings(read), process_ms=ms), depth)
                print(json.dumps(det.summary(), ensure_ascii=False))
                return
            cv2.imshow('overlay', overlay)
            cv2.imshow('mask', mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                name = f'live_{time.strftime("%Y%m%d_%H%M%S")}'
                save_evidence(a.out, name, raw, mask, overlay,
                              record(det, raw, cfg, 'camera' if a.camera else str(a.image[0]),
                                     camera_settings=camera_settings(read), process_ms=ms), depth)
                print('저장:', (a.out / name).resolve())
            elif key in (ord('q'), 27):
                break
    cv2.destroyAllWindows()


@contextmanager
def nullreader(path):
    frame = imread(path)
    yield lambda: frame.copy()


def run_scenes(a, cfg):
    if a.image and len(a.image) != 3:
        raise SystemExit('scenes 모드의 --image는 보임·없음·가림 순서로 3장이어야 한다.')
    rows = []
    cam = camera(cfg) if a.camera else None
    read = cam.__enter__() if cam else None
    try:
        for i, scene in enumerate(SCENES):
            if a.camera:
                input(f'[{scene}] 장면을 준비하고 Enter: ')
                for _ in range(5):
                    raw = read()          # 버퍼에 남은 이전 장면 프레임을 버린다
                src = 'camera'
            elif a.image:
                raw, src = imread(a.image[i]), str(a.image[i])
            else:
                raw, src = synthetic(scene), 'synthetic'
            depth = depth_of(read) if read else None
            frame, det, mask, overlay, ms = process(raw, cfg, label=scene.upper(), depth=depth)
            expected = scene != 'absent'
            verdict = ('TP' if expected else 'FP') if det.detected else ('FN' if expected else 'TN')
            info = record(det, raw, cfg, src, scene=scene, expected_visible=expected, verdict=verdict,
                          camera_settings=camera_settings(read) if read else None, process_ms=ms)
            save_evidence(a.out, f'scene_{i}_{scene}', raw, mask, overlay, info, depth)
            rows.append(info)
            print(f"{scene:9s} {verdict}  detected={det.detected} error={det.error} "
                  f"area_ratio={det.area_ratio} depth_m={det.depth_m} candidates={det.num_candidates} "
                  f"rejected={info['rejected']}")
    finally:
        if cam:
            cam.__exit__(None, None, None)
    shutil.copy(a.config, a.out / 'config_used.yaml')   # 세 장면에 실제로 쓴 설정
    (a.out / 'scenes.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    sheet = np.hstack([imread(a.out / f'scene_{i}_{s}_overlay.png') for i, s in enumerate(SCENES)])
    save_png(a.out / 'scenes_overlay.png', sheet)
    print('저장 위치:', a.out.resolve())
    if not a.no_gui:
        cv2.imshow('scenes', sheet)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return rows


def run_bench(a, cfg):
    """심화: 조건(조명·거리) 하나를 바꿔 같은 설정으로 N프레임 검출률·처리 속도를 잰다.
    목표가 보이는 상태에서 실행한다 — 검출률 = 검출 프레임 / 전체 프레임."""
    if a.exposure is not None:
        cfg['camera']['exposure'] = a.exposure
    out = a.out / 'bench'
    if a.camera:
        cam = camera(cfg)
        read = cam.__enter__()
        input(f'[{a.label}] 조건을 준비하고(목표가 보이게) Enter: ')
        frames = None
    else:
        cam, read = None, None
        base = imread(a.image[0]) if a.image else synthetic('visible')
        frames = [base] * a.frames
    rows, ms_list, cap_ms, sample = [], [], [], None
    try:
        settings = camera_settings(read) if read else None
        for i in range(a.frames):
            t0 = time.perf_counter()
            raw = read() if read else frames[i]
            depth = depth_of(read) if read else None
            cap_ms.append((time.perf_counter() - t0) * 1000)   # RealSense 뎁스 정렬(align) 시간 포함
            if a.gain != 1.0:
                raw = cv2.convertScaleAbs(raw, alpha=a.gain)   # 조명 변화 모의(실제 조명과 다름)
            frame, det, mask, overlay, ms = process(raw, cfg, label=a.label, depth=depth)
            ms_list.append(ms)
            rows.append(det.summary())
            if i == a.frames // 2:
                sample = (raw, mask, overlay, depth)
    finally:
        if cam:
            cam.__exit__(None, None, None)
    det_rows = [r for r in rows if r['detected']]
    zs = [r['depth_m'] for r in det_rows if r['depth_m'] is not None]
    errs = np.array([r['error'] for r in det_rows]) if det_rows else np.zeros((0, 2))
    ms_arr = np.array(ms_list)
    summary = dict(
        label=a.label, source='camera' if a.camera else (str(a.image[0]) if a.image else 'synthetic'),
        gain=a.gain, camera_settings=settings, frames=a.frames,
        detected_frames=len(det_rows), detection_rate=len(det_rows) / a.frames,
        error_mean=errs.mean(0).tolist() if len(errs) else None,
        error_std=errs.std(0).tolist() if len(errs) else None,
        area_ratio_mean=float(np.mean([r['area_ratio'] for r in det_rows])) if det_rows else None,
        depth_m_median=float(np.median(zs)) if zs else None,
        process_ms_mean=float(ms_arr.mean()), process_ms_p95=float(np.percentile(ms_arr, 95)),
        process_fps=float(1000 / ms_arr.mean()),
        capture_ms_mean=float(np.mean(cap_ms)) if read else None,
        rejected_reasons={k: sum(r['rejected'].count(k) for r in rows)
                          for k in ('too_small', 'too_large', 'bad_aspect', 'low_extent', 'low_solidity',
                                    'zero_moment', 'no_depth', 'depth_range', 'size_mismatch')},
        hsv_ranges=cfg['target']['hsv_ranges'], min_area=cfg['selection']['min_area'])
    out.mkdir(parents=True, exist_ok=True)
    raw, mask, overlay, depth = sample
    save_evidence(out, f'{a.label}_sample', raw, mask, overlay, summary, depth)
    (out / f'{a.label}.json').write_text(json.dumps(dict(summary=summary, frames=rows),
                                                    ensure_ascii=False, indent=2), encoding='utf-8')
    print(f"[{a.label}] 검출률 {summary['detection_rate']:.1%} ({len(det_rows)}/{a.frames})  "
          f"처리 {summary['process_ms_mean']:.2f} ms (p95 {summary['process_ms_p95']:.2f}, "
          f"{summary['process_fps']:.0f} fps)  면적비 {summary['area_ratio_mean']}  "
          f"Z 중앙값 {summary['depth_m_median']}")
    print('저장:', (out / f'{a.label}.json').resolve())


def run_collect(a, cfg):
    """실제 인식률 평가용 데이터 수집: 조건 하나(조명·거리)마다 N프레임 + 정답 상자.
    큐브를 받침대에 고정한다(손에 들면 정답 상자가 어긋난다). --absent는 큐브 없는 장면(오검출 측정)."""
    if not a.camera or a.no_gui:
        raise SystemExit('collect는 --camera와 화면이 필요하다.')
    out = a.out / 'dataset' / a.label
    out.mkdir(parents=True, exist_ok=True)
    with camera(cfg) as read:
        print(f'[{a.label}] 미리보기 — 장면을 준비하고 space(정지) → '
              + ('Enter로 수집' if a.absent else '큐브를 드래그 후 Enter') + ', q=취소')
        while True:
            frame = read()
            _, det, _, overlay, _ = process(frame, cfg, a.label, depth=depth_of(read))
            cv2.imshow('collect', overlay)
            k = cv2.waitKey(1) & 0xFF
            if k == ord(' '):
                break
            if k in (ord('q'), 27):
                cv2.destroyAllWindows()
                return
        gt = None
        if not a.absent:
            x, y, w, h = cv2.selectROI('collect', frame, showCrosshair=False)
            if w == 0 or h == 0:
                raise SystemExit('정답 상자를 그리지 않았다.')
            gt = [int(x), int(y), int(w), int(h)]
        else:
            cv2.waitKey(0)
        cv2.destroyAllWindows()
        settings = camera_settings(read)
        depth_info = None
        for i in range(a.frames):
            save_png(out / f'frame_{i:03d}.png', read())
            depth = depth_of(read)        # 같은 시점의 정렬된 뎁스(뎁스 검증을 켠 경우)
            if depth is not None:
                save_png(out / f'depth_{i:03d}.png', depth.data)   # uint16 원값, × scale = m
                depth_info = dict(scale=depth.scale, fx=depth.fx, fy=depth.fy)
    meta = dict(label=a.label, light=a.light, distance_m=a.distance, gt=gt, frames=a.frames,
                depth=depth_info,
                camera_settings=settings, time=time.strftime('%Y-%m-%d %H:%M:%S'),
                note='gt는 정지 화면 기준. 수집 중 큐브가 움직이지 않아야 한다.')
    (out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'저장: {out}  ({a.frames}장, gt={gt}, 카메라={settings})')


TUNE_HELP = """[tune] 키
  마우스 드래그(overlay 쪽) = 목표 영역 선택 → HSV 범위 자동 제안
  클릭 = 그 픽셀의 HSV 출력      space = 화면 정지/재개
  f = 현재 자동 노출·화이트밸런스 값을 고정값으로 설정   w = 설정 파일(detector.yaml)에 기록   q/ESC = 종료
  H low > H high 로 두면 빨강처럼 0/179를 넘는 범위가 된다."""


def run_tune(a, cfg):
    if a.no_gui:
        raise SystemExit('tune 모드는 GUI가 필요하다.')
    src = camera(cfg) if a.camera else nullreader(a.image[0]) if a.image else None
    if src is None:
        raise SystemExit('tune 모드는 --camera 또는 --image가 필요하다.')
    print(TUNE_HELP)
    names = ['H low', 'S low', 'V low', 'H high', 'S high', 'V high']
    lo, hi = detector.trackbars_from_hsv_ranges(cfg['target']['hsv_ranges'])
    cv2.namedWindow('HSV controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('HSV controls', 500, 330)
    for n, v, m in zip(names, lo + hi, [179, 255, 255] * 2):
        cv2.createTrackbar(n, 'HSV controls', v, m, lambda _: None)
    cv2.createTrackbar('min_area', 'HSV controls', cfg['selection']['min_area'], 20000, lambda _: None)

    auto_min_area = cfg['selection'].get('auto', {}).get('min_area_auto')
    start_min_area = cfg['selection']['min_area']
    state = {'frame': None, 'depth': None, 'drag': None, 'roi': None, 'basis': None}

    def on_mouse(event, x, y, flags, _):
        w = cfg['processing']['width']
        x = min(x, w - 1) if x < w else x - w            # mask 쪽 클릭도 같은 픽셀로
        if event == cv2.EVENT_LBUTTONDOWN:
            state['drag'] = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state['drag']:
            state['roi'] = (state['drag'], (x, y))
        elif event == cv2.EVENT_LBUTTONUP and state['drag']:
            (x0, y0), state['drag'], state['roi'] = state['drag'], None, None
            frame = state['frame']
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            if abs(x - x0) < 4 or abs(y - y0) < 4:          # 클릭
                z = state['depth'].data[y, x] * state['depth'].scale if state['depth'] else None
                print(f'픽셀 ({x},{y}) BGR={frame[y, x].tolist()} HSV={hsv[y, x].tolist()} Z={z}')
                return
            xa, xb, ya, yb = sorted((x0, x))[0], sorted((x0, x))[1], sorted((y0, y))[0], sorted((y0, y))[1]
            lo, hi = detector.suggest_hsv(hsv[ya:yb, xa:xb])
            for n, v in zip(names, lo + hi):
                cv2.setTrackbarPos(n, 'HSV controls', v)
            state['basis'] = dict(roi=[xa, ya, xb - xa, yb - ya], frame=frame.copy(),
                                  suggested={'lower': lo, 'upper': hi})
            print(f'영역 ({xa},{ya})-({xb},{yb}) 제안: lower={lo} upper={hi}')

    cv2.namedWindow('tune')
    cv2.setMouseCallback('tune', on_mouse)
    frozen = False
    with src as read:
        while True:
            if not frozen or state['frame'] is None:
                state['frame'], scale = detector.preprocess(read(), cfg)
                state['depth'] = detector.preprocess_depth(depth_of(read), scale)
            vals = [cv2.getTrackbarPos(n, 'HSV controls') for n in names]
            if all(l <= h for l, h in zip(vals[1:3], vals[4:6])):
                cfg['target']['hsv_ranges'] = detector.hsv_ranges_from_trackbars(vals[:3], vals[3:])
            cfg['selection']['min_area'] = max(1, cv2.getTrackbarPos('min_area', 'HSV controls'))
            det, mask = detector.detect(state['frame'], cfg, depth=state['depth'])  # tune은 처리 해상도 기준
            overlay = detector.draw(state['frame'], det, 'PAUSED' if frozen else 'TUNE')
            if state['roi']:
                cv2.rectangle(overlay, *state['roi'], (255, 0, 255), 1)
            cv2.imshow('tune', np.hstack([overlay, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
            key = cv2.waitKey(30) & 0xFF
            if key == ord(' '):
                frozen = not frozen
            elif key == ord('f'):
                if not hasattr(read, 'settings'):
                    print('노출 고정은 RealSense 카메라에서만 가능하다.')
                    continue
                cur = read.settings()
                cfg['camera'].update({k: int(v) for k, v in cur.items()})
                print('고정할 값(다음 실행부터 적용):', cfg['camera']['exposure'], cfg['camera']['white_balance'])
            elif key == ord('w'):
                # min_area: auto는 슬라이더를 움직였을 때만 숫자로 바꾼다
                keep_auto = auto_min_area and cfg['selection']['min_area'] == start_min_area
                write_tuning(a.config, cfg, keep_auto_min_area=keep_auto)
                if state['basis']:
                    save_hsv_basis(a.out / 'tune', state['basis'], cfg, camera_settings(read))
                print('기록:', cfg['target']['hsv_ranges'], 'min_area=', cfg['selection']['min_area'],
                      'exposure=', cfg['camera'].get('exposure'), 'white_balance=', cfg['camera'].get('white_balance'))
            elif key in (ord('q'), 27):
                break
    cv2.destroyAllWindows()


def save_hsv_basis(out, basis, cfg, settings):
    """HSV 범위를 정한 근거: 선택 영역의 HSV 분포, 제안값, 최종값, 영역 안 검출률·밖 오검출률."""
    frame = basis['frame']
    x, y, w, h = basis['roi']
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv[y:y + h, x:x + w].reshape(-1, 3)
    mask = detector.color_mask(frame, cfg)
    inside = np.zeros(mask.shape, bool)
    inside[y:y + h, x:x + w] = True
    stats = {ch: {f'p{q}': int(np.percentile(roi[:, i], q)) for q in (5, 50, 95)}
             for i, ch in enumerate('HSV')}
    info = dict(roi_xywh=basis['roi'], roi_hsv_percentiles=stats,
                suggested=basis['suggested'], final_hsv_ranges=cfg['target']['hsv_ranges'],
                roi_coverage=float(np.mean(mask[inside] > 0)),
                outside_false_positive_ratio=float(np.mean(mask[~inside] > 0)),
                min_area=cfg['selection']['min_area'], camera_settings=settings)
    view = frame.copy()
    cv2.rectangle(view, (x, y), (x + w, y + h), (255, 0, 255), 2)
    save_png(out / 'hsv_basis_frame.png', view)
    save_png(out / 'hsv_basis_mask.png', mask)
    (out / 'hsv_basis.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f"근거 저장: {out.resolve()}  영역 안 검출률={info['roi_coverage']:.2f} "
          f"영역 밖 오검출률={info['outside_false_positive_ratio']:.3f}")


def write_tuning(path, cfg, keep_auto_min_area=False):
    """주석을 지우지 않도록 hsv_ranges·min_area·exposure·white_balance 줄만 바꿔 쓴다."""
    if '/install/' in str(path) and not Path(path).is_symlink():
        print(f'경고: {path}는 설치 폴더의 복사본이다 — 저장소 파일은 바뀌지 않는다. '
              '--symlink-install로 빌드하거나 --config에 저장소의 detector.yaml을 준다.')
    scalars = {'min_area': 'auto' if keep_auto_min_area else cfg['selection']['min_area'],
               'exposure': cfg['camera'].get('exposure'),
               'white_balance': cfg['camera'].get('white_balance')}
    lines = Path(path).read_text(encoding='utf-8').splitlines()
    out, skip = [], False
    for line in lines:
        s = line.strip()
        if skip:
            if s.startswith('- lower') or s.startswith('upper'):
                continue
            skip = False
        if s.startswith('hsv_ranges:'):
            out.append(line)
            for r in cfg['target']['hsv_ranges']:
                out.append(f"    - lower: {list(r['lower'])}")
                out.append(f"      upper: {list(r['upper'])}")
            skip = True
            continue
        key = s.split(':')[0]
        if key in scalars:
            indent = line[:len(line) - len(line.lstrip())]
            comment = line[line.index('#'):] if '#' in line else ''
            value = 'null' if scalars[key] is None else scalars[key]
            line = f"{indent}{key}: {value}".ljust(29) + comment
        out.append(line)
    Path(path).write_text('\n'.join(out) + '\n', encoding='utf-8')
    detector.load_config(path)  # 기록 결과가 다시 읽히는지 확인


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('mode', choices=['live', 'scenes', 'tune', 'bench', 'collect'])
    src = p.add_mutually_exclusive_group()
    src.add_argument('--camera', action='store_true')
    src.add_argument('--image', type=Path, nargs='+')
    p.add_argument('--config', type=Path, default=detector.DEFAULT_CONFIG)
    p.add_argument('--out', type=Path, default=(Path.cwd() / 'results') / 'perception',
                   help='결과 폴더(기본 <cwd>/results/perception)')
    p.add_argument('--no-gui', action='store_true')
    p.add_argument('--label', default='baseline', help='bench 조건 이름 (예: bright, dim, near, far)')
    p.add_argument('--frames', type=int, default=100, help='bench 프레임 수')
    p.add_argument('--exposure', type=int, help='bench에서 config의 노출을 덮어쓴다(조명 조건 재현용)')
    p.add_argument('--gain', type=float, default=1.0, help='bench 밝기 배율 모의(실제 조명 변화 아님)')
    p.add_argument('--light', default='normal', help='collect 조명 라벨: dark | normal | bright ...')
    p.add_argument('--distance', type=float, help='collect 큐브까지 거리 [m]')
    p.add_argument('--absent', action='store_true', help='collect: 큐브 없는 장면(오검출 측정용)')
    a = p.parse_args()
    cfg = detector.load_config(a.config)
    print('설정:', a.config, '| HSV', cfg['target']['hsv_ranges'], '| min_area', cfg['selection']['min_area'])
    {'live': run_live, 'scenes': run_scenes, 'tune': run_tune, 'bench': run_bench,
     'collect': run_collect}[a.mode](a, cfg)


if __name__ == '__main__':
    main()
