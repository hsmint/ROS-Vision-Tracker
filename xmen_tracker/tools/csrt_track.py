"""CSRT 추적 + HSV·뎁스 검출 결합 — 독립 시험 도구(ROS 노드 아님).

검출(xmen_tracker/detector.py)은 매 프레임 돌리고, CSRT는 '같은 큐브를 계속 따라가는 것'과
'검출이 잠깐 놓친 프레임(가림 등)을 현재 프레임 증거로 메우는 것'에만 쓴다.

  추적 시작   검출기가 큐브 '전체'(partial 아님)를 confirm_frames 연속으로 같은 곳에서 잡으면 CSRT 시작.
              처음부터 일부만 보이는 물체(손에 쥔 같은 색 부품 등)로는 추적을 시작하지 않는다.
  추적 중     CSRT 상자와 겹치는 검출 후보를 목표로 고른다(다른 곳의 같은 색 물체는 무시).
              reinit_every 프레임마다, 또는 CSRT 상자와 검출이 어긋나면 검출 상자로 CSRT를 다시 맞춘다.
  검출 놓침   CSRT 상자 안을 '현재 프레임'에서 확인한다: 상자 안 목표색 픽셀 ≥ 마지막으로 잰 큐브 넓이 × verify_area,
              그 픽셀들의 뎁스 중앙값이 추적 거리 ± max(3 cm, 10%). 통과하면 출력 중심 = 상자 안 목표색 픽셀의
              중심(현재 측정값). CSRT 예측 위치나 이전 좌표를 그대로 내보내지 않는다.
  확인 실패   그 프레임은 즉시 미검출(None). '어느 물체를 따라가는지'만 hold_frames(1 s) 동안 유지해,
              다시 보이면 CSRT 상자나 마지막 위치 근처의 후보를 이어받는다(더 큰 같은 색 물체로 갈아타지 않음).

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
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from xmen_tracker import detector  # noqa: E402

DEFAULT_CONFIG = HERE.parent / 'config' / 'detector.yaml'


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x0, y0, x1, y1 = max(ax, bx), max(ay, by), min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


@dataclass
class Output:
    state: str                 # SEARCH | CONFIRM | TRACK
    target: tuple = None       # (ex, ey, area_ratio) — 현재 프레임 측정값. 미검출이면 None
    bbox: tuple = None         # (x, y, w, h) 처리 영상 좌표
    source: str = None         # 'detector' | 'csrt'(검출이 놓친 프레임을 CSRT 상자 + 현재 프레임 확인으로 메움)
    depth_m: float = None
    csrt_ms: float = 0.0
    detect_ms: float = 0.0


class CubeTracker:
    """검출 + CSRT. 상태는 '어느 물체를 따라가는가'만 갖고, 좌표는 항상 현재 프레임에서 잰다."""

    def __init__(self, cfg, fx, fy, use_csrt=True, confirm_frames=2, reinit_every=10, assoc_iou=0.2,
                 verify_area=0.15, hold_frames=30, assoc_dist=1.5, require_full_view=True,
                 min_fill=0.90, min_v=30, z_gate=0.15):
        self.cfg, self.fx, self.fy = cfg, fx, fy
        # 균일성 검사(라이브 기록 run2: 청바지 오검출 57건 중 53건 제거, 실측 큐브는 fill≈1.0·V≥38):
        #   min_fill  컨투어 안 픽셀 중 목표색(형태학 처리 전) 비율 — 데님은 직조 무늬로 듬성듬성(중앙값 0.87)
        #   min_v     목표색 픽셀 밝기 중앙값 — 그늘진 데님 V≈20, 데이터셋에서 가장 어두운 큐브(dark_0.1m) V≈41
        # z_gate      추적 중 연결할 후보의 깊이 허용폭(추적 거리 × 비율, 최소 5 cm, 놓친 프레임마다 +2 cm)
        self.min_fill, self.min_v, self.z_gate = min_fill, min_v, z_gate
        d = cfg.get('depth', {})
        self.z_range = (d.get('min_m', 0.1), d.get('max_m', 1.1))
        self.use_csrt = use_csrt
        self.confirm_frames, self.reinit_every, self.assoc_iou = confirm_frames, reinit_every, assoc_iou
        self.verify_area, self.hold_frames, self.assoc_dist = verify_area, hold_frames, assoc_dist
        self.require_full_view = require_full_view
        self.reset()

    def reset(self):
        self.state, self.tracker, self.track_box, self.track_z = 'SEARCH', None, None, None
        self.confirm, self.lost, self.since_init, self.track_area = 0, 0, 0, None

    # ---------- 측정 ----------
    def _measure(self, frame, cand, scale):
        """검출 후보 → (ex, ey, 면적비), bbox. 중심은 detector와 같은 볼록껍질 중심."""
        co = detector.to_original(cand.center, scale)
        g = detector.as_geometry(scale)
        h, w = frame.shape[:2]
        fs = (g.W, g.H) if g.W else (w, h)
        ex, ey = detector.normalized_error(co, fs)
        return (ex, ey, cand.area / (g.sx * g.sy) / (fs[0] * fs[1])), cv2.boundingRect(cand.contour)

    def _verify_box(self, frame, mask, depth, box, scale):
        """CSRT 상자를 현재 프레임에서 확인. 통과하면 (target, bbox, z) — 상자 안 목표색 픽셀로 잰 값."""
        H, W = mask.shape
        x, y, w, h = (int(round(v)) for v in box)
        x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + w), min(H, y + h)
        if x1 - x0 < 3 or y1 - y0 < 3:
            return None
        roi = mask[y0:y1, x0:x1] > 0
        need = max(15, self.verify_area * (self.track_area or 0))
        if roi.sum() < need:                          # 큐브가 거의 안 보임(가림이 너무 큼)
            return None
        z = None
        if depth is not None:
            dz = depth.data[y0:y1, x0:x1][roi]
            dz = dz[dz > 0]
            if len(dz) >= 10:
                z = float(np.median(dz)) * depth.scale
                if not self.z_range[0] <= z <= self.z_range[1]:
                    return None                      # 검출기와 같은 거리 범위(0.1~1.1 m) 밖
                if self.track_z is not None and abs(z - self.track_z) > max(0.03, 0.10 * self.track_z):
                    return None                      # 다른 거리의 물체(앞을 지나간 손 등)
            elif self.track_z is not None:
                # 목표색 픽셀에 뎁스가 없을 때(어두운 큐브·원거리에서 흔함): 상자 전체 뎁스로 확인한다.
                # 상자 대부분이 측정되는데 추적 거리와 다르면 CSRT가 다른 물체(가까운 청바지 등)로 옮겨 간 것.
                # 상자 전체도 뎁스가 거의 없으면 판단 근거가 없으므로 색 확인만으로 둔다.
                db = depth.data[y0:y1, x0:x1]
                valid = db[db > 0]
                if len(valid) >= 0.5 * db.size:
                    zb = float(np.median(valid)) * depth.scale
                    if abs(zb - self.track_z) > max(0.03, 0.10 * self.track_z):
                        return None
        ys, xs = np.nonzero(roi)
        cx, cy = xs.mean() + x0, ys.mean() + y0
        g = detector.as_geometry(scale)
        fs = (g.W, g.H) if g.W else (W, H)
        ex, ey = detector.normalized_error(detector.to_original((cx, cy), scale), fs)
        return (ex, ey, len(xs) / (g.sx * g.sy) / (fs[0] * fs[1])), (x0, y0, x1 - x0, y1 - y0), z

    # ---------- 한 프레임 ----------
    def step(self, raw_bgr, depth=None):
        t0 = time.perf_counter()
        frame, scale = detector.preprocess(raw_bgr, self.cfg)
        dproc = detector.preprocess_depth(depth, scale)
        det, mask = detector.detect(frame, self.cfg, scale, dproc)
        accepted = [c for c in det.candidates if c.accepted]
        if self.use_csrt and accepted and (self.min_fill or self.min_v):
            accepted = self._uniform(frame, accepted)
            if det.selected is not None and not any(c is det.selected for c in accepted):
                # 검출기가 고른 후보가 균일성 검사에서 빠짐 → 남은 것 중 가장 큰 것(검출기 선택 규칙과 같게 면적 기준)
                det.selected = max(accepted, key=lambda c: c.area) if accepted else None
                det.detected = det.selected is not None
                det.partial = getattr(det.selected, 'partial', None)
                det.depth_m = getattr(det.selected, 'depth_m', None)
        detect_ms = (time.perf_counter() - t0) * 1000
        csrt_ms = 0.0

        if not self.use_csrt:                        # 비교용: 검출기만
            if not det.detected:
                return Output('SEARCH', detect_ms=detect_ms)
            tgt, bb = self._measure(raw_bgr, det.selected, scale)
            return Output('SEARCH', tgt, bb, 'detector', det.depth_m, detect_ms=detect_ms)

        if self.state == 'TRACK':
            t1 = time.perf_counter()
            ok, box = self.tracker.update(self._small(frame))
            csrt_ms = (time.perf_counter() - t1) * 1000
            box = tuple(v / self.csrt_scale for v in box) if ok else self.track_box
            # 1) CSRT 상자와 겹치거나 마지막 위치 근처의 검출 후보 = 같은 큐브
            best = self._associate(accepted, box)
            if best is not None:
                tgt, bb = self._measure(raw_bgr, best, scale)
                self.lost, self.since_init = 0, self.since_init + 1
                if best.depth_m is not None:
                    self.track_z = best.depth_m
                if best.partial is None:
                    self.track_area = best.area
                if self.since_init >= self.reinit_every or iou(bb, box) < 0.5:
                    self._init(frame, bb)               # 흘러감 방지: 검출 상자로 다시 맞춘다
                self.track_box = bb
                return Output('TRACK', tgt, bb, 'detector', best.depth_m, csrt_ms, detect_ms)
            # 2) 검출이 놓침 → CSRT 상자를 현재 프레임에서 확인
            v = self._verify_box(frame, mask, dproc, box, scale) if ok else None
            if v is not None:
                tgt, bb, z = v
                self.lost, self.track_box = 0, bb
                return Output('TRACK', tgt, bb, 'csrt', z, csrt_ms, detect_ms)
            self.lost += 1
            if self.lost >= self.hold_frames:
                self.reset()                                # 1 s 동안 다시 확인 못 함 → 대상 잊음
            return Output(self.state, csrt_ms=csrt_ms, detect_ms=detect_ms)   # 미검출: 이전 좌표 안 씀

        # SEARCH / CONFIRM: 검출기 결과를 그대로 내되, 큐브 '전체'가 연속으로 보일 때만 추적을 시작
        if not det.detected:
            self.confirm = 0
            self.state = 'SEARCH'
            return Output('SEARCH', detect_ms=detect_ms)
        tgt, bb = self._measure(raw_bgr, det.selected, scale)
        full = det.partial is None or not self.require_full_view
        if full and self.track_box is not None and iou(bb, self.track_box) >= self.assoc_iou:
            self.confirm += 1
        else:
            self.confirm = 1 if full else 0
        self.track_box = bb
        if self.confirm >= self.confirm_frames:
            self.track_z, self.track_area = det.depth_m, det.selected.area
            self._init(frame, bb)
            self.state = 'TRACK'
        else:
            self.state = 'CONFIRM' if self.confirm else 'SEARCH'
        if det.partial and self.require_full_view:
            return Output(self.state, detect_ms=detect_ms)      # 일부만 보이는 새 물체는 목표로 내지 않음
        return Output(self.state, tgt, bb, 'detector', det.depth_m, detect_ms=detect_ms)

    def _associate(self, accepted, box):
        """추적 중인 큐브와 같은 후보: CSRT 상자와 IoU ≥ assoc_iou, 또는 마지막 확인 위치에서
        (상자 장변 × assoc_dist) 안. 여럿이면 IoU가 크고 가까운 것."""
        lx, ly, lw, lh = self.track_box
        lc, reach = (lx + lw / 2, ly + lh / 2), self.assoc_dist * max(lw, lh)
        best, key = None, None
        tol = None
        if self.track_z is not None:
            tol = max(0.05, self.z_gate * self.track_z) + 0.02 * self.lost
        for c in accepted:
            if tol is not None and c.depth_m is not None and abs(c.depth_m - self.track_z) > tol:
                continue                                 # 같은 자리라도 거리가 크게 다르면 앞을 지나간 다른 물체
            bb = cv2.boundingRect(c.contour)
            o = iou(bb, box)
            d = np.hypot(bb[0] + bb[2] / 2 - lc[0], bb[1] + bb[3] / 2 - lc[1])
            if o >= self.assoc_iou or d <= reach:
                k = (o, -d)
                if key is None or k > key:
                    best, key = c, k
        return best

    def _uniform(self, frame, cands):
        """큐브 면은 색이 고르다: 컨투어 안 목표색 비율 ≥ min_fill, 목표색 밝기 중앙값 ≥ min_v."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        raw = detector.in_ranges(hsv, self.cfg['target']['hsv_ranges']) > 0
        keep = []
        for c in cands:
            x, y, w, h = cv2.boundingRect(c.contour)
            m = np.zeros((h, w), np.uint8)
            cv2.drawContours(m, [c.contour - (x, y)], -1, 1, -1)
            inside = m > 0
            r = raw[y:y + h, x:x + w][inside]
            if not r.size:
                continue
            v = hsv[y:y + h, x:x + w, 2][inside][r]
            if r.mean() >= self.min_fill and (not len(v) or np.median(v) >= self.min_v):
                keep.append(c)
        return keep

    def _small(self, frame):
        s = self.csrt_scale
        return frame if s == 1.0 else cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)

    def _init(self, frame, bb, max_side=64):
        """CSRT 시작. 상자가 크면(가까운 큐브) 영상을 줄여 상자 장변 ≤ max_side px로 돌린다
        — CSRT 시간은 상자 크기에 비례(0.1 m에서 17 ms → 약 6 ms)."""
        x, y, w, h = bb
        pad = max(2, int(0.1 * max(w, h)))
        self.csrt_scale = min(1.0, max_side / (max(w, h) + 2 * pad))
        s = self.csrt_scale
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(self._small(frame), tuple(int(round(v * s)) for v in
                                                     (max(0, x - pad), max(0, y - pad), w + 2 * pad, h + 2 * pad)))
        self.since_init = 0


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
            tr = CubeTracker(cfg, fx, fy, use_csrt=use)
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
        tr = CubeTracker(cfg, di['fx'], di['fy'], use_csrt=use)
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
    tr = CubeTracker(cfg, intr.fx, intr.fy)
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
        tr = CubeTracker(cfg, di['fx'], di['fy'], use_csrt=use)
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
