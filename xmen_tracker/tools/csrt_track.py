"""HSV 검출 + CSRT 결합 추적 시험 도구(ROS 노드 아님).

추적 로직은 xmen_tracker/cube_tracker.py(tracker_node와 같은 코드)이고 설정은 detector.yaml의 tracking 절.
이 도구는 같은 로직을 모의 영상·저장 데이터·RealSense 실시간으로 돌려 '검출기만'(기존 방식)과 비교하고 기록한다.

실행
  python3 tools/csrt_track.py --demo                  # 모의 영상으로 검출기만 vs 결합 비교(움직임·가림·같은 색 큐브)
  python3 tools/csrt_track.py --camera                # RealSense 실시간(q 종료). 화면: 초록=검출, 주황=CSRT 확인
  python3 tools/csrt_track.py --dataset <ws>/results/perception/dataset --label normal_0.5m
  python3 tools/csrt_track.py --camera --record rec/run1   # 실시간 + 기록(n 키 = '지금 큐브 없음' 표시 전환)
  python3 tools/csrt_track.py --replay rec/run1            # 기록한 원본 프레임으로 다시 실행·채점

기록 폴더(--record)
  annotated.mp4   처리한 모든 프레임에 결과를 그린 영상(사람이 맞았는지 확인용)
  results.csv     프레임별 결과: 시각, 상태, 출처, ex·ey·면적비, bbox, 깊이, 처리시간, absent(n 키 표시)
  frame_*.png / depth_*.png   --record-every 프레임마다 원본 컬러 + 16bit 뎁스(--replay 입력)
  meta.json       fx·fy·뎁스 배율·설정 파일 경로
"""
import argparse
import csv
import json
import queue
import threading
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from xmen_tracker import detector  # noqa: E402
from xmen_tracker.cube_tracker import CubeTracker, Output  # noqa: E402

DEFAULT_CONFIG = HERE.parent / 'config' / 'detector.yaml'


class DetectorOnly:
    """비교 기준: 매 프레임 검출기 선택 결과만(추적·CSRT·균일성 검사 없음) — 기존 tracker_node 방식."""

    def __init__(self, cfg):
        self.cfg = cfg

    def step(self, raw_bgr, depth=None):
        t0 = time.perf_counter()
        frame, g = detector.preprocess(raw_bgr, self.cfg)
        det, _ = detector.detect(frame, self.cfg, g, detector.preprocess_depth(depth, g))
        ms = (time.perf_counter() - t0) * 1000
        if not det.detected:
            return Output('SEARCH', detect_ms=ms)
        return Output('SEARCH', (*det.error, det.area_ratio), det.bbox, detector.bbox_to_original(det.bbox, g),
                      'detector', det.depth_m, det.partial, detect_ms=ms)


def make_tracker(cfg, use_csrt=True):
    """use_csrt=True: HSV + CSRT 결합(tracker_node와 같은 CubeTracker), False: 검출기만."""
    return CubeTracker(cfg) if use_csrt else DetectorOnly(cfg)


def draw(frame, out, label=''):
    img = frame.copy()
    H, W = img.shape[:2]
    cv2.drawMarker(img, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
    if out.target:
        x, y, w, h = out.bbox
        color = (0, 220, 0) if out.source == 'detector' else (0, 140, 255)
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cx, cy = int((out.target[0] + 1) * W / 2), int((out.target[1] + 1) * H / 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        text = f'{out.state} {out.source} ex={out.target[0]:+.3f} ey={out.target[1]:+.3f}'
    else:
        text = f'{out.state} NO TARGET'
    cv2.rectangle(img, (0, 0), (W, 24), (255, 255, 255), -1)
    cv2.putText(img, f'{label} {text} csrt {out.csrt_ms:.1f}ms', (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
    return img


# ---------------- 모의 시험 영상 ----------------
def demo_sequence(dataset, light='normal', n=150, seed=0):
    """실제 빈 장면 위로 실제 큐브(정답 상자 영역 + 뎁스)를 움직이며 붙인다.
    구간: 0~39 이동 / 40~69 손이 큐브 대부분을 가림 / 70~109 같은 색 큐브(방해물)가 다른 곳에 등장 /
          110~129 이동 / 130~ 큐브가 사라짐(이후 출력이 있으면 오검출)."""
    rng = np.random.default_rng(seed)
    root = Path(dataset)
    meta = json.loads((root / f'{light}_0.5m' / 'meta.json').read_text())
    src = cv2.imread(str(root / f'{light}_0.5m' / 'frame_000.png'))
    sdep = cv2.imread(str(root / f'{light}_0.5m' / 'depth_000.png'), cv2.IMREAD_UNCHANGED)
    bg = cv2.imread(str(root / f'{light}_empty' / 'frame_000.png'))
    bdep = cv2.imread(str(root / f'{light}_empty' / 'depth_000.png'), cv2.IMREAD_UNCHANGED)
    di = meta['depth']
    x, y, w, h = meta['gt']
    pad = 3
    patch = src[y - pad:y + h + pad, x - pad:x + w + pad]
    pmask = cv2.inRange(cv2.cvtColor(patch, cv2.COLOR_BGR2HSV), (95, 120, 8), (125, 255, 255)) > 0
    pmask = cv2.morphologyEx(pmask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
    zc = int(np.median(sdep[y:y + h, x:x + w][sdep[y:y + h, x:x + w] > 0]))
    H, W = bg.shape[:2]
    ph, pw = patch.shape[:2]
    out = []
    for i in range(n):
        img, dep = bg.copy(), bdep.copy()
        gt = None
        if i < 130:
            px = int(120 + 360 * (0.5 - 0.5 * np.cos(i / 129 * np.pi)))       # 왼쪽 → 오른쪽
            py = int(150 + 40 * np.sin(i / 20))
            region = (slice(py, py + ph), slice(px, px + pw))
            img[region][pmask] = patch[pmask]
            dep[region][pmask] = zc + rng.integers(-2, 3, pmask.sum())
            gt = [px + pad, py + pad, w, h]
            if 40 <= i < 70:                                                 # 손이 큐브의 60~75%를 가림(2 cm 앞)
                cover = 0.6 + 0.15 * np.sin((i - 40) / 29 * np.pi)
                hx0 = px + pad + int(w * (1 - cover))
                cv2.rectangle(img, (hx0, py - 6), (px + pw + 8, py + ph + 8), (90, 120, 190), -1)
                cv2.rectangle(dep, (hx0, py - 6), (px + pw + 8, py + ph + 8), zc - 20, -1)
        if 70 <= i < 110:                                                    # 같은 색·같은 크기 큐브 방해물(더 큼: 더 가까움)
            big = cv2.resize(patch, None, fx=1.25, fy=1.25, interpolation=cv2.INTER_LINEAR)
            bm = cv2.resize(pmask.astype(np.uint8), None, fx=1.25, fy=1.25, interpolation=cv2.INTER_NEAREST) > 0
            qx, qy = 40, 200
            img[qy:qy + big.shape[0], qx:qx + big.shape[1]][bm] = big[bm]
            dep[qy:qy + big.shape[0], qx:qx + big.shape[1]][bm] = int(zc * 0.8)
        D = detector.DepthFrame(dep, di['scale'], di['fx'], di['fy'])
        out.append((img, D, gt, i))
    return out, (di['fx'], di['fy'])


def judge(out, gt, W, H):
    if out.target is None:
        return 'FN' if gt else 'TN'
    if not gt:
        return 'FP'
    cx, cy = (out.target[0] + 1) * W / 2, (out.target[1] + 1) * H / 2
    x, y, w, h = gt
    pad = 0.15 * max(w, h)
    return 'TP' if x - pad <= cx <= x + w + pad and y - pad <= cy <= y + h + pad else 'WRONG'


def run_demo(a, cfg):
    phases = [('이동', 0, 40), ('손으로 가림(60~75%)', 40, 70), ('같은 색 큐브 방해물 등장', 70, 110),
              ('이동', 110, 130), ('큐브 사라짐', 130, 150)]
    print('구간                     | 검출기만: TP·놓침·엉뚱한곳·오검출 | CSRT 결합: TP·놓침·엉뚱한곳·오검출 (CSRT로 메운 프레임)')
    tot = {'검출기만': {}, 'CSRT 결합': {}}
    for light in ('dark', 'normal', 'bright'):
        seq, (fx, fy) = demo_sequence(a.dataset, light)
        detector.resolve_size_limits(cfg)
        res = {}
        for name, use in [('검출기만', False), ('CSRT 결합', True)]:
            tr = make_tracker(cfg, use)
            rows, ms = [], []
            for img, D, gt, i in seq:
                o = tr.step(img, D)
                rows.append((i, judge(o, gt, img.shape[1], img.shape[0]), o.source))
                ms.append(o.csrt_ms + o.detect_ms)
                if a.out and light == 'normal' and use and i % 10 == 0:
                    a.out.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(a.out / f'demo_{i:03d}.png'), draw(img, o, f'#{i}'))
            res[name] = (rows, ms)
        print(f'--- 조명 {light}')
        for ph, s, e in phases:
            line = f'{ph:24s} |'
            for name in ('검출기만', 'CSRT 결합'):
                rows = [r for r in res[name][0] if s <= r[0] < e]
                c = {k: sum(r[1] == k for r in rows) for k in ('TP', 'FN', 'WRONG', 'FP')}
                bridged = sum(r[2] == 'csrt' for r in rows)
                line += f" {c['TP']:3d}·{c['FN']:3d}·{c['WRONG']:3d}·{c['FP']:3d}" + (f' ({bridged})' if name == 'CSRT 결합' else '') + '      |'
                t = tot[name].setdefault(ph, {'TP': 0, 'FN': 0, 'WRONG': 0, 'FP': 0, 'n': 0})
                for k in c: t[k] += c[k]
                t['n'] += len(rows)
            print(line)
        for name in res:
            print(f'    {name} 처리 시간 평균 {np.mean(res[name][1]):.1f} ms, p95 {np.percentile(res[name][1], 95):.1f} ms')
    print('\n합계(조명 3종)')
    for name, d in tot.items():
        tp = sum(v['TP'] for v in d.values()); pos = sum(v['TP'] + v['FN'] + v['WRONG'] for k, v in d.items() if k != '큐브 사라짐')
        wrong = sum(v['WRONG'] for v in d.values()); fp = d['큐브 사라짐']['FP']
        print(f'  {name:8s} 인식 {tp}/{pos} ({tp / pos:.1%}), 엉뚱한 물체 {wrong}, 큐브가 없을 때 출력 {fp}/{d["큐브 사라짐"]["n"]}')


def run_dataset(a, cfg):
    root = Path(a.dataset) / a.label
    meta = json.loads((root / 'meta.json').read_text())
    di = meta['depth']
    detector.resolve_size_limits(cfg)
    for name, use in [('검출기만', False), ('CSRT 결합', True)]:
        tr = make_tracker(cfg, use)
        res, ms = [], []
        for fp in sorted(root.glob('frame_*.png')):
            img = cv2.imread(str(fp)); dep = cv2.imread(str(fp.with_name(fp.name.replace('frame_', 'depth_'))), cv2.IMREAD_UNCHANGED)
            o = tr.step(img, detector.DepthFrame(dep, di['scale'], di['fx'], di['fy']))
            res.append(judge(o, meta['gt'], img.shape[1], img.shape[0])); ms.append(o.csrt_ms + o.detect_ms)
        print(f'{a.label} {name}: ' + ' '.join(f'{k} {res.count(k)}' for k in ('TP', 'FN', 'WRONG', 'FP', 'TN')) + f'  처리 {np.mean(ms):.1f} ms')



# ---------------- 기록 ----------------
CSV_FIELDS = ['frame', 't_s', 'state', 'source', 'ex', 'ey', 'area_ratio', 'x', 'y', 'w', 'h',
              'depth_m', 'detect_ms', 'csrt_ms', 'absent', 'raw']


def csv_row(i, t, o, absent, raw):
    tg, bb = o.target or ('',) * 3, o.bbox if o.target else ('',) * 4
    return [i, f'{t:.3f}', o.state, o.source or '', *(f'{v:.4f}' if v != '' else '' for v in tg), *bb,
            f'{o.depth_m:.3f}' if o.target and o.depth_m else '', f'{o.detect_ms:.2f}', f'{o.csrt_ms:.2f}',
            int(absent), raw]


class Recorder:
    """디스크 쓰기는 별도 스레드에서 한다(추적 루프의 처리 시간에 PNG 압축이 섞이지 않게).
    큐가 가득 차면 원본 프레임만 버리고 개수를 센다. 영상·CSV는 모든 프레임을 남긴다."""

    def __init__(self, out, size, fps, every, meta):
        self.out, self.every = Path(out), max(1, every)
        self.out.mkdir(parents=True, exist_ok=True)
        if any(self.out.glob('frame_*.png')) or (self.out / 'results.csv').exists():
            raise SystemExit(f'{self.out}에 이전 기록이 있습니다. 새 폴더를 지정하세요.')
        (self.out / 'meta.json').write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        self.video = cv2.VideoWriter(str(self.out / 'annotated.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        self.csv_file = open(self.out / 'results.csv', 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_FIELDS)
        self.q = queue.Queue(maxsize=60)
        self.dropped = self.saved = 0
        self.thread = threading.Thread(target=self._work, daemon=True)
        self.thread.start()

    def _work(self):
        while (item := self.q.get()) is not None:
            kind, *rest = item
            if kind == 'raw':
                i, img, dep = rest
                cv2.imwrite(str(self.out / f'frame_{i:06d}.png'), img)
                cv2.imwrite(str(self.out / f'depth_{i:06d}.png'), dep)
                self.saved += 1
            else:
                self.video.write(rest[0])

    def add(self, i, t, img, dep, out, annotated, absent):
        raw = 0
        if i % self.every == 0:
            try:
                self.q.put_nowait(('raw', i, img, dep))
                raw = 1
            except queue.Full:
                self.dropped += 1
        self.csv.writerow(csv_row(i, t, out, absent, raw))
        self.q.put(('video', annotated))          # 영상 프레임은 버리지 않는다(필요하면 잠깐 기다림)

    def close(self):
        self.q.put(None)
        self.thread.join()
        self.video.release()
        self.csv_file.close()
        print(f'기록: {self.out}  원본 {self.saved}쌍 저장, 큐 초과로 버림 {self.dropped}')


def summarize(rows):
    """results.csv 행(dict) 요약. absent 표시가 있으면 큐브 있음/없음 구간을 나눠 센다."""
    n = len(rows)
    if not n:
        return '프레임 없음'
    src = lambda r: r['source'] or 'none'
    lines = []
    for name, part in [('전체', rows), ('큐브 있음(absent=0)', [r for r in rows if r['absent'] == '0']),
                       ('큐브 없음(absent=1)', [r for r in rows if r['absent'] == '1'])]:
        if not part or (name != '전체' and len(part) == n and name.startswith('큐브 있음')):
            continue
        c = {k: sum(src(r) == k for r in part) for k in ('detector', 'csrt', 'none')}
        k = len(part)
        lines.append(f'{name} {k}프레임: 검출 {c["detector"]} ({c["detector"] / k:.1%}), '
                     f'CSRT {c["csrt"]} ({c["csrt"] / k:.1%}), 미검출 {c["none"]} ({c["none"] / k:.1%})')
    ms = np.array([float(r['detect_ms']) + float(r['csrt_ms']) for r in rows])
    lines.append(f'처리 평균 {ms.mean():.1f} ms, p95 {np.percentile(ms, 95):.1f} ms, 최대 {ms.max():.1f} ms')
    return '\n'.join(lines)


def run_camera(a, cfg):
    import pyrealsense2 as rs
    W, H = cfg['processing']['width'], cfg['processing']['height']
    pipe, conf = rs.pipeline(), rs.config()
    conf.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    conf.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    prof = pipe.start(conf)
    try:
        pipe.wait_for_frames(5000)
    except RuntimeError:
        # 이전 실행이 비정상 종료되면 D435가 스트림을 못 내는 상태로 남을 때가 있다 → 장치 리셋 후 한 번 재시도
        print('카메라 프레임이 오지 않아 장치를 리셋합니다(약 5초)...')
        pipe.stop()
        prof.get_device().hardware_reset()
        time.sleep(6)
        pipe = rs.pipeline()
        prof = pipe.start(conf)
        pipe.wait_for_frames(10000)
    sensor = prof.get_device().first_color_sensor()
    c = cfg['camera']
    if c.get('white_balance'):
        sensor.set_option(rs.option.enable_auto_white_balance, 0)
        sensor.set_option(rs.option.white_balance, float(c['white_balance']))
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    cfg['camera'].update(fx_px=intr.fx, fx_width=intr.width)      # 실제 초점거리로 크기 기준 재계산
    detector.resolve_size_limits(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    tr = CubeTracker(cfg)
    stats = {'detector': 0, 'csrt': 0, None: 0}
    ms = []
    fps = 30
    rec = None
    if a.record:
        rec = Recorder(a.record, (W, H), fps, a.record_every, {
            'depth': {'fx': intr.fx, 'fy': intr.fy, 'scale': scale}, 'fps': fps,
            'record_every': a.record_every, 'config': str(a.config.resolve()),
            'started': time.strftime('%Y-%m-%d %H:%M:%S')})
    absent = False
    t_start = time.monotonic()
    try:
        while True:
            try:
                fs = align.process(pipe.wait_for_frames(5000))
            except RuntimeError as e:
                print(f'카메라 프레임 수신 실패({e}) — 지금까지 기록을 저장하고 종료합니다. 다시 실행하면 리셋을 시도합니다.')
                break
            img = np.asanyarray(fs.get_color_frame().get_data()).copy()
            dep = np.asanyarray(fs.get_depth_frame().get_data()).copy()
            o = tr.step(img, detector.DepthFrame(dep, scale, intr.fx, intr.fy))
            i = len(ms)
            stats[o.source] += 1
            ms.append(o.csrt_ms + o.detect_ms)
            vis = None
            if rec or not a.no_gui:
                vis = draw(img, o, f'#{i}' + (' [ABSENT]' if absent else ''))
                if rec:
                    cv2.circle(vis, (W - 14, 12), 6, (0, 0, 255), -1)       # 기록 중 표시
            if rec:
                rec.add(i, time.monotonic() - t_start, img, dep, o, vis, absent)
            if a.no_gui:
                if len(ms) >= a.frames:
                    break
                continue
            cv2.imshow('csrt_track', vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('n'):
                absent = not absent
                print(f'#{i}: {"큐브 없음" if absent else "큐브 있음"} 구간 시작')
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        if rec:
            rec.close()
    n = len(ms)
    print(f'{n}프레임: 검출 {stats["detector"]}, CSRT로 메움 {stats["csrt"]}, 미검출 {stats[None]} | 처리 평균 {np.mean(ms):.1f} ms p95 {np.percentile(ms, 95):.1f} ms')
    if rec:
        with open(Path(a.record) / 'results.csv') as f:
            print(summarize(list(csv.DictReader(f))))


def run_replay(a, cfg):
    """--record로 저장한 원본 프레임(frame_/depth_)으로 다시 실행한다. 설정·코드를 바꾼 뒤 같은 장면으로 비교할 때.
    원본은 record_every 간격이라 실시간보다 프레임 간격이 넓다(CSRT에는 더 불리한 조건)."""
    root = Path(a.replay)
    meta = json.loads((root / 'meta.json').read_text())
    di = meta['depth']
    cfg['camera'].update(fx_px=di['fx'], fx_width=cfg['processing']['width'])
    detector.resolve_size_limits(cfg)
    absent_of = {}
    if (root / 'results.csv').exists():
        with open(root / 'results.csv') as f:
            absent_of = {int(r['frame']): r['absent'] for r in csv.DictReader(f)}
    frames = sorted(root.glob('frame_*.png'))
    if not frames:
        raise SystemExit(f'{root}에 frame_*.png가 없습니다.')
    print(f'{root}: 원본 {len(frames)}쌍 (기록 간격 {meta.get("record_every", 1)}프레임)')
    for name, use in [('검출기만', False), ('CSRT 결합', True)]:
        tr = make_tracker(cfg, use)
        rows = []
        for fp in frames:
            i = int(fp.stem.split('_')[1])
            img = cv2.imread(str(fp))
            dep = cv2.imread(str(fp.with_name(fp.name.replace('frame_', 'depth_'))), cv2.IMREAD_UNCHANGED)
            o = tr.step(img, detector.DepthFrame(dep, di['scale'], di['fx'], di['fy']))
            rows.append(dict(zip(CSV_FIELDS, map(str, csv_row(i, 0.0, o, absent_of.get(i, '0') == '1', 1)))))
        print(f'[{name}]\n' + summarize(rows))

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument('--demo', action='store_true')
    src.add_argument('--camera', action='store_true')
    src.add_argument('--label', help='--dataset의 조건 폴더 하나(예: normal_0.5m)')
    src.add_argument('--replay', type=Path, help='--record로 저장한 폴더를 다시 실행')
    p.add_argument('--dataset', default=str(Path.home() / 'ws/results/perception/dataset'))
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--out', type=Path, help='--demo: 결과 영상(10프레임마다) 저장 폴더')
    p.add_argument('--no-gui', action='store_true')
    p.add_argument('--frames', type=int, default=300, help='--camera --no-gui: 처리할 프레임 수')
    p.add_argument('--record', type=Path, help='--camera: 결과 영상·CSV·원본 프레임을 저장할 새 폴더')
    p.add_argument('--record-every', type=int, default=3,
                   help='--record: 원본 컬러+뎁스 PNG를 N프레임마다 저장(기본 3 = 10 fps, 분당 약 0.2 GB)')
    a = p.parse_args()
    cfg = detector.load_config(a.config)
    if a.record and not a.camera:
        p.error('--record는 --camera와 함께 씁니다')
    if a.demo:
        run_demo(a, cfg)
    elif a.camera:
        run_camera(a, cfg)
    elif a.replay:
        run_replay(a, cfg)
    else:
        run_dataset(a, cfg)


if __name__ == '__main__':
    main()
