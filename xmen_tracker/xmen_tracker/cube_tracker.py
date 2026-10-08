"""HSV 검출 + CSRT 결합 추적(tracker_node와 tools/csrt_track.py 공용).

검출(detector.py)은 매 프레임 돌리고, CSRT는 검출이 놓친 프레임에서만 쓴다.

  추적 시작   큐브 '전체'(partial 아님)가 confirm_frames 연속으로 같은 곳에서 검출되면 TRACK.
              처음부터 일부만 보이는 물체(손에 쥔 같은 색 부품 등)로는 추적을 시작하지 않는다.
  추적 중     마지막 위치(+속도 예측) 근처이고 거리(뎁스)가 맞는 검출 후보를 같은 큐브로 잇는다
              (다른 곳의 같은 색 물체, 앞을 지나간 다른 거리의 물체로 갈아타지 않음).
  검출 놓침   그때 처음으로 CSRT를 직전 프레임의 큐브 상자로 시작해 현재 프레임에서 위치를 찾고,
              그 상자를 현재 프레임에서 확인한다: 목표색 픽셀 ≥ 마지막 큐브 넓이 × verify_area,
              뎁스가 추적 거리 ± max(3 cm, 10%). 통과하면 출력 = 상자 안 목표색 픽셀의 중심(현재 측정값).
              CSRT 예측 위치나 이전 좌표를 그대로 내보내지 않는다.
  확인 실패   그 프레임은 미검출. '어느 물체를 따라가는지'만 hold_frames 동안 유지한다.

속도: 검출이 이어지는 프레임에는 CSRT를 돌리지 않는다(검출만 약 2 ms). CSRT는 놓친 프레임에서만,
큐브 상자 장변 ≤ csrt_max_side px로 줄인 영상에서 돈다.
균일성 검사: 큐브 면은 색이 고르고 진하다. 컨투어 안 목표색 비율 ≥ min_fill, 목표색 밝기 중앙값 ≥ min_v,
채도 중앙값 ≥ min_s(데님은 직조 무늬로 듬성듬성하고 채도가 낮다). CSRT 상자 확인에도 같은 색 기준을 쓴다.
CSRT로만 이은 프레임이 max_bridge를 넘으면 놓고, 대상이 검출로 확인되지 않는 프레임에 다른 곳에 큐브 전체가
switch_frames 연속으로 보이면 그 큐브로 다시 잡는다(가짜 대상에 오래 묶이지 않게).
설정은 detector.yaml의 tracking 절.
"""
import time
from dataclasses import dataclass

import cv2
import numpy as np

from xmen_tracker import detector

DEFAULTS = {
    'csrt': True,             # false = 검출기만(비교·CSRT 없는 OpenCV용)
    'confirm_frames': 2,
    'hold_frames': 30,        # 이만큼 연속으로 확인 못 하면 대상을 잊는다(20 Hz에서 1.5 s)
    'assoc_dist': 1.5,        # 예측 위치에서 상자 장변 × 이 값 안이면 같은 큐브
    'z_gate': 0.15,           # 같은 큐브로 이을 깊이 허용폭(추적 거리 × 비율, 최소 5 cm, 놓친 프레임마다 +2 cm)
    'verify_area': 0.15,      # CSRT 상자 안 목표색 픽셀 하한(마지막 큐브 넓이 대비)
    'min_fill': 0.90,
    'min_v': 30,
    'min_s': 235,             # 목표색 픽셀 채도 중앙값 하한. 큐브 246~255(데이터셋 전 조건), 밝은 곳 청바지 204~222
    'switch_frames': 1,       # 대상이 검출로 확인되지 않는 프레임에 다른 곳에 큐브 전체가 이만큼 연속으로 보이면 그쪽으로 바꾼다
    'max_bridge': 30,         # 검출기 확인 없이 CSRT로만 이을 수 있는 연속 프레임. 넘으면 대상을 놓고 다시 찾는다
    'csrt_max_side': 64,
    'require_full_view': True,
    # CSRT 내부 설정(cv2.TrackerCSRT_Params). 큐브처럼 작은 상자는 템플릿(기본 200)으로 키워서 계산하므로
    # 시간이 상자 크기가 아니라 template_size에 좌우된다. 48·스케일 9·분할 끔: 놓친 프레임 최대 28 → 5 ms,
    # 모의·라이브 기록 인식률 동일(설정 비교: 기본 / 100 / 64 / 48).
    'csrt_params': {'number_of_scales': 9, 'use_segmentation': False, 'template_size': 48},
}


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x0, y0, x1, y1 = max(ax, bx), max(ay, by), min(ax + aw, bx + bw), min(ay + ah, by + bh)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def create_csrt(params=None):
    """OpenCV 빌드에 따라 cv2 또는 cv2.legacy에 있다. 없으면 None. params: TrackerCSRT_Params 항목."""
    for mod in (cv2, getattr(cv2, 'legacy', None)):
        f = getattr(mod, 'TrackerCSRT_create', None) if mod is not None else None
        if f is None:
            continue
        if not params:
            return f()
        q = mod.TrackerCSRT_Params()
        for k, v in params.items():
            if not hasattr(q, k):
                raise ValueError(f'tracking.csrt_params: 알 수 없는 항목 {k}')
            setattr(q, k, type(getattr(q, k))(v))
        return f(q)
    return None


def csrt_available():
    return create_csrt() is not None


@dataclass
class Output:
    state: str                 # SEARCH | CONFIRM | TRACK
    target: tuple = None       # (ex, ey, area_ratio) — 현재 프레임 측정값. 미검출이면 None
    bbox: tuple = None         # (x, y, w, h) 처리 영상 좌표
    bbox_original: tuple = None  # 입력 영상 좌표
    source: str = None         # 'detector' | 'csrt'
    depth_m: float = None
    partial: str = None
    csrt_ms: float = 0.0
    detect_ms: float = 0.0


class CubeTracker:
    """검출 + CSRT. 상태는 '어느 물체를 따라가는가'만 갖고, 좌표는 항상 현재 프레임에서 잰다."""

    def __init__(self, cfg, fx=None, fy=None, **overrides):
        self.cfg = cfg
        t = dict(DEFAULTS, **(cfg.get('tracking') or {}))
        t['csrt_params'] = dict(DEFAULTS['csrt_params'], **(t.get('csrt_params') or {}))
        t.update({k: v for k, v in overrides.items() if v is not None})
        unknown = set(t) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'tracking: 알 수 없는 항목 {sorted(unknown)}')
        self.p = t
        self.use_csrt = bool(t['csrt']) and csrt_available()
        d = detector.depth_cfg(cfg) or {}
        self.z_range = (d.get('min_m', 0.1), d.get('max_m', 1.1))
        self.reset()

    def reset(self):
        self.state = 'SEARCH'
        self.track_box = self.track_z = self.track_area = None
        self.velocity = (0.0, 0.0)
        self.confirm = self.lost = self.bridged = 0
        self.challenger, self.challenger_n = None, 0    # 대상이 확인되지 않는 동안 다른 곳에 큐브 전체가 보인 연속 프레임
        self.tracker = None
        self.prev_frame = None       # 마지막으로 큐브를 확인한 프레임(검출이 놓치면 여기서 CSRT 시작)

    # ---------- 한 프레임 ----------
    def step(self, raw_bgr, depth=None):
        """raw_bgr: 입력 BGR, depth: 입력 해상도 DepthFrame 또는 None."""
        t0 = time.perf_counter()
        frame, g = detector.preprocess(raw_bgr, self.cfg)
        self.frame_size = (raw_bgr.shape[1], raw_bgr.shape[0])
        dproc = detector.preprocess_depth(depth, g)
        det, mask = detector.detect(frame, self.cfg, g, dproc)
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        accepted = [c for c in det.candidates if c.accepted]
        if accepted and (self.p['min_fill'] or self.p['min_v'] or self.p['min_s']):
            accepted = self._uniform(frame, accepted)
        detect_ms = (time.perf_counter() - t0) * 1000

        if self.state == 'TRACK':
            return self._track(frame, mask, dproc, g, accepted, detect_ms)
        return self._search(frame, g, accepted, detect_ms)

    def _search(self, frame, g, accepted, detect_ms):
        best = detector.select(accepted, (frame.shape[1] / 2, frame.shape[0] / 2),
                               self.cfg['selection']['tie_ratio'])
        if best is None:
            self.confirm, self.state = 0, 'SEARCH'
            return Output('SEARCH', detect_ms=detect_ms)
        bb = cv2.boundingRect(best.contour)
        full = best.partial is None or not self.p['require_full_view']
        if full and self.track_box is not None and iou(bb, self.track_box) >= 0.2:
            self.confirm += 1
        else:
            self.confirm = 1 if full else 0
        self.track_box = bb
        if full and self.confirm >= self.p['confirm_frames']:
            self.state, self.lost = 'TRACK', 0
            self._accept(frame, best, bb, first=True)
        else:
            self.state = 'CONFIRM' if self.confirm else 'SEARCH'
        if best.partial and self.p['require_full_view']:
            return Output(self.state, detect_ms=detect_ms)      # 일부만 보이는 새 물체는 목표로 내지 않음
        return self._out(self.state, best, bb, g, 'detector', 0.0, detect_ms)

    def _track(self, frame, mask, dproc, g, accepted, detect_ms):
        best = self._associate(accepted)
        if best is not None:
            bb = cv2.boundingRect(best.contour)
            self._accept(frame, best, bb)
            self.lost = self.bridged = 0
            self.challenger, self.challenger_n = None, 0
            self.tracker = None                   # 검출이 이어지는 동안 CSRT는 쉰다
            return self._out('TRACK', best, bb, g, 'detector', 0.0, detect_ms)
        # 대상이 이 프레임에서 검출로 확인되지 않음. 다른 곳에 큐브 '전체'가 confirm_frames 연속으로 보이면
        # 그쪽으로 바꾼다(추적하던 것이 가려진 채 끝났거나, 처음부터 다른 물체였을 수 있다).
        switched = self._challenge(frame, g, accepted, detect_ms)
        if switched is not None:
            return switched
        csrt_ms, v = 0.0, None
        self.bridged += 1
        if self.use_csrt and self.bridged <= self.p['max_bridge']:
            t1 = time.perf_counter()
            box = self._csrt_box(frame)
            csrt_ms = (time.perf_counter() - t1) * 1000
            if box is not None:
                v = self._verify_box(mask, dproc, box, g)
        if v is not None:
            tgt, bb, z = v
            self.lost = 0
            self._move(bb)
            return Output('TRACK', tgt, bb, detector.bbox_to_original(bb, g), 'csrt', z, None, csrt_ms, detect_ms)
        self.lost += 1
        if self.lost >= self.p['hold_frames']:
            self.reset()
        return Output(self.state, csrt_ms=csrt_ms, detect_ms=detect_ms)   # 미검출: 이전 좌표 안 씀

    # ---------- 도움 ----------
    def _challenge(self, frame, g, accepted, detect_ms):
        full = [c for c in accepted if c.partial is None]
        if not full:
            self.challenger, self.challenger_n = None, 0
            return None
        best = max(full, key=lambda c: c.area)
        bb = cv2.boundingRect(best.contour)
        # 빠르게 움직이는 큐브는 프레임 사이 위치가 크게 바뀌므로 '같은 자리' 조건 없이 연속 프레임 수만 센다
        self.challenger_n += 1
        self.challenger = bb
        if self.challenger_n < self.p['switch_frames']:
            return None
        self.reset()
        self.state = 'TRACK'
        self._accept(frame, best, bb, first=True)
        return self._out('TRACK', best, bb, g, 'detector', 0.0, detect_ms)

    def _out(self, state, cand, bb, g, source, csrt_ms, detect_ms):
        co = detector.to_original(cand.center, g)
        geo = detector.as_geometry(g)
        fs = (geo.W, geo.H) if geo.W else self.frame_size
        ex, ey = detector.normalized_error(co, fs)
        tgt = (ex, ey, cand.area / (geo.sx * geo.sy) / (fs[0] * fs[1]))
        return Output(state, tgt, bb, detector.bbox_to_original(bb, g), source, cand.depth_m, cand.partial,
                      csrt_ms, detect_ms)

    def _move(self, bb):
        """확인된 새 상자로 이동량(속도)을 갱신한다."""
        if self.track_box is not None:
            (lx, ly, lw, lh), (x, y, w, h) = self.track_box, bb
            dx, dy = (x + w / 2) - (lx + lw / 2), (y + h / 2) - (ly + lh / 2)
            vx, vy = self.velocity
            self.velocity = (0.5 * vx + 0.5 * dx, 0.5 * vy + 0.5 * dy)
        self.track_box = bb

    def _accept(self, frame, cand, bb, first=False):
        if first:
            self.velocity = (0.0, 0.0)
            self.track_box = bb
        else:
            self._move(bb)
        if cand.depth_m is not None:
            self.track_z = cand.depth_m
        if cand.partial is None or self.track_area is None:
            self.track_area = cand.area
        self.prev_frame = frame

    def _associate(self, accepted):
        """예측 위치(마지막 상자 + 속도 × 놓친 프레임 수)와 IoU ≥ 0.2이거나 중심이 (장변 × assoc_dist) 안,
        그리고 깊이가 추적 거리 허용폭 안인 후보. 여럿이면 IoU가 크고 가까운 것."""
        lx, ly, lw, lh = self.track_box
        k = self.lost + 1
        px, py = lx + self.velocity[0] * k, ly + self.velocity[1] * k
        pred = (px, py, lw, lh)
        pc = (px + lw / 2, py + lh / 2)
        reach = self.p['assoc_dist'] * max(lw, lh) * (1 + 0.1 * self.lost)
        tol = None
        if self.track_z is not None:
            tol = max(0.05, self.p['z_gate'] * self.track_z) + 0.02 * self.lost
        best, key = None, None
        for c in accepted:
            if tol is not None and c.depth_m is not None and abs(c.depth_m - self.track_z) > tol:
                continue
            bb = cv2.boundingRect(c.contour)
            o = iou(bb, pred)
            d = np.hypot(bb[0] + bb[2] / 2 - pc[0], bb[1] + bb[3] / 2 - pc[1])
            if o >= 0.2 or d <= reach:
                kk = (o, -d)
                if key is None or kk > key:
                    best, key = c, kk
        return best

    def _csrt_box(self, frame):
        """검출이 놓친 프레임의 CSRT 상자(처리 영상 좌표). 처음 놓친 프레임에서 직전 확인 프레임으로 시작한다."""
        if self.tracker is None:
            if self.prev_frame is None:
                return None
            self._init(self.prev_frame, self.track_box)
            if self.tracker is None:
                return None
        ok, box = self.tracker.update(self._small(frame))
        if not ok:
            return None
        return tuple(v / self.csrt_scale for v in box)

    def _small(self, frame):
        s = self.csrt_scale
        return frame if s == 1.0 else cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)

    def _init(self, frame, bb):
        """CSRT 시작. 상자가 크면(가까운 큐브) 영상을 줄여 상자 장변 ≤ csrt_max_side px로 돌린다
        — CSRT 시간은 상자 크기에 비례(0.1 m에서 17 ms → 약 6 ms)."""
        x, y, w, h = bb
        pad = max(2, int(0.1 * max(w, h)))
        self.csrt_scale = min(1.0, self.p['csrt_max_side'] / (max(w, h) + 2 * pad))
        s = self.csrt_scale
        self.tracker = create_csrt(self.p['csrt_params'])
        if self.tracker is None:
            return
        self.tracker.init(self._small(frame), tuple(int(round(v * s)) for v in
                                                     (max(0, x - pad), max(0, y - pad), w + 2 * pad, h + 2 * pad)))

    def _verify_box(self, mask, depth, box, g):
        """CSRT 상자를 현재 프레임에서 확인. 통과하면 (target, bbox, z) — 상자 안 목표색 픽셀로 잰 값."""
        H, W = mask.shape
        x, y, w, h = (int(round(v)) for v in box)
        x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + w), min(H, y + h)
        if x1 - x0 < 3 or y1 - y0 < 3:
            return None
        roi = mask[y0:y1, x0:x1] > 0
        if roi.sum() < max(15, self.p['verify_area'] * (self.track_area or 0)):
            return None                               # 큐브가 거의 안 보임(가림이 너무 큼)
        if self.hsv is not None and (self.p['min_s'] or self.p['min_v']):
            px = self.hsv[y0:y1, x0:x1][roi]          # 상자 안 목표색 픽셀이 큐브 면처럼 진하고 밝은지(청바지 차단)
            if np.median(px[:, 1]) < self.p['min_s'] or np.median(px[:, 2]) < self.p['min_v']:
                return None
        z = None
        if depth is not None:
            dz = depth.data[y0:y1, x0:x1][roi]
            dz = dz[dz > 0]
            tol = max(0.03, 0.10 * self.track_z) if self.track_z is not None else None
            if len(dz) >= 10:
                z = float(np.median(dz)) * depth.scale
                if not self.z_range[0] <= z <= self.z_range[1]:
                    return None                       # 검출기와 같은 거리 범위 밖
                if tol is not None and abs(z - self.track_z) > tol:
                    return None                       # 다른 거리의 물체(앞을 지나간 손 등)
            elif tol is not None:
                # 목표색 픽셀에 뎁스가 없으면(어두운 큐브·원거리) 상자 전체 뎁스로 확인한다.
                # 상자 대부분이 측정되는데 추적 거리와 다르면 CSRT가 다른 물체(가까운 청바지 등)로 옮겨 간 것.
                db = depth.data[y0:y1, x0:x1]
                valid = db[db > 0]
                if len(valid) >= 0.5 * db.size and abs(float(np.median(valid)) * depth.scale - self.track_z) > tol:
                    return None
        ys, xs = np.nonzero(roi)
        geo = detector.as_geometry(g)
        fs = (geo.W, geo.H) if geo.W else self.frame_size
        ex, ey = detector.normalized_error(detector.to_original((xs.mean() + x0, ys.mean() + y0), g), fs)
        return (ex, ey, len(xs) / (geo.sx * geo.sy) / (fs[0] * fs[1])), (x0, y0, x1 - x0, y1 - y0), z

    def _uniform(self, frame, cands):
        """큐브 면은 색이 고르다: 컨투어 안 목표색 비율 ≥ min_fill, 목표색 밝기 중앙값 ≥ min_v."""
        hsv = self.hsv
        keep = []
        for c in cands:
            x, y, w, h = cv2.boundingRect(c.contour)
            sub = hsv[y:y + h, x:x + w]
            m = np.zeros((h, w), np.uint8)
            cv2.drawContours(m, [c.contour], -1, 1, -1, offset=(-x, -y))
            inside = m > 0
            r = detector.in_ranges(sub, self.cfg['target']['hsv_ranges'])[inside] > 0
            if not r.size:
                continue
            px = sub[inside][r]
            if r.mean() >= self.p['min_fill'] and (not len(px) or (np.median(px[:, 2]) >= self.p['min_v']
                                                              and np.median(px[:, 1]) >= self.p['min_s'])):
                keep.append(c)
        return keep
