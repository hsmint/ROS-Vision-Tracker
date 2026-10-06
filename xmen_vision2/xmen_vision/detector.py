"""HSV·Contour 검출 파이프라인.

영상 → (리사이즈) → HSV 변환 → 색상 마스크 → 잡음 제거 → 컨투어 → (뎁스 검증) → 대상 선택 → 중심 계산
뎁스 검증: 컨투어 안 뎁스 중앙값 Z로 거리 범위와 실제 크기(px × Z / f)를 확인한다.
상태를 갖지 않는다: 매 프레임 독립적으로 판단하므로 미검출 시 이전 좌표가 섞여 들어올 수 없다.
"""
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parent / 'config.yaml'


def load_config(path=DEFAULT_CONFIG):
    cfg = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    for r in cfg['target']['hsv_ranges']:
        if len(r['lower']) != 3 or len(r['upper']) != 3:
            raise ValueError('hsv_ranges의 lower/upper는 [H,S,V] 세 값이어야 한다.')
    if cfg['processing']['blur_ksize'] % 2 == 0 and cfg['processing']['blur_ksize'] != 0:
        raise ValueError('blur_ksize는 0 또는 홀수여야 한다.')
    d = cfg.get('depth')
    if d and d['enabled']:
        if not 0 < d['min_m'] < d['max_m']:
            raise ValueError('depth: 0 < min_m < max_m 이어야 한다.')
        if not 0 < d['area_m2'][0] < d['area_m2'][1]:
            raise ValueError('depth.area_m2는 [하한, 상한]이어야 한다.')
    return cfg


def depth_cfg(cfg):
    """뎁스 검증 설정. 꺼져 있으면 None."""
    d = cfg.get('depth')
    return d if d and d['enabled'] else None


@dataclass
class DepthFrame:
    """컬러에 정렬된 뎁스. data=uint16(0=측정 실패), scale=값→m 배율, fx·fy=컬러 초점거리(px)."""
    data: np.ndarray
    scale: float
    fx: float
    fy: float


@dataclass
class Candidate:
    contour: np.ndarray
    area: float
    center: tuple        # (x, y) 픽셀, 모멘트 기반
    solidity: float
    accepted: bool
    reason: str          # 탈락 사유 또는 'ok'
    depth_m: float = None   # 컨투어 안 뎁스 중앙값(m). 뎁스 없으면 None
    size_m: tuple = None    # 실제 크기 추정 (짧은 변, 긴 변) m
    area_m2: float = None   # 실제 면적 추정 m²


@dataclass
class Detection:
    """미검출이면 center·center_original·error·area_ratio·bbox가 모두 None이다."""
    detected: bool
    frame_size: tuple = None      # 실제(원본) 프레임 (W, H)
    image_center: tuple = None    # 처리 해상도 영상 중심 (w/2, h/2) — 표시·선택용
    center: tuple = None          # 목표 중심, 처리 해상도 픽셀
    center_original: tuple = None # 목표 중심, 실제 프레임 픽셀
    error: tuple = None           # 정규화 오차 (ex, ey) = ((cx-W/2)/(W/2), (cy-H/2)/(H/2)), 오른쪽·아래 +
    area: float = None            # 컨투어 면적, 처리 해상도 px
    area_ratio: float = None      # contour_area / (W×H)
    bbox: tuple = None            # (x, y, w, h), 처리 해상도
    num_candidates: int = 0       # 필터 통과 후보 수
    depth_m: float = None         # 목표 뎁스 중앙값(m). 뎁스 없음·측정 실패면 None
    size_m: tuple = None          # 목표 실제 크기 추정 (짧은 변, 긴 변) m
    candidates: list = field(default_factory=list, repr=False)
    selected: Candidate = field(default=None, repr=False)

    def summary(self):
        """JSON 저장용. 컨투어 배열은 제외한다."""
        keys = ('detected', 'frame_size', 'image_center', 'center', 'center_original', 'error',
                'area', 'area_ratio', 'bbox', 'num_candidates', 'depth_m', 'size_m')
        d = {k: getattr(self, k) for k in keys}
        d['rejected'] = [c.reason for c in self.candidates if not c.accepted]
        return d


def preprocess(frame, cfg):
    """처리 해상도로 리사이즈. (리사이즈 영상, 원본→처리 배율 (sx, sy))를 반환한다."""
    p = cfg['processing']
    h, w = frame.shape[:2]
    if (w, h) == (p['width'], p['height']):
        return frame, (1.0, 1.0)
    small = cv2.resize(frame, (p['width'], p['height']), interpolation=cv2.INTER_AREA)
    return small, (p['width'] / w, p['height'] / h)


def preprocess_depth(depth, scale):
    """뎁스를 처리 해상도로 맞춘다. 0(측정 실패)이 실제 거리와 섞이지 않게 INTER_NEAREST, 초점거리도 같은 배율."""
    if depth is None or scale == (1.0, 1.0):
        return depth
    sx, sy = scale
    h, w = depth.data.shape
    data = cv2.resize(depth.data, (round(w * sx), round(h * sy)), interpolation=cv2.INTER_NEAREST)
    return DepthFrame(data, depth.scale, depth.fx * sx, depth.fy * sy)


def to_original(point, scale):
    """처리 해상도 좌표를 원본 좌표로 복원한다(픽셀 중심 규약, examples/10 참고)."""
    if point is None:
        return None
    (u, v), (sx, sy) = point, scale
    return ((u + .5) / sx - .5, (v + .5) / sy - .5)


def color_mask(frame, cfg):
    p = cfg['processing']
    if p['blur_ksize']:
        frame = cv2.GaussianBlur(frame, (p['blur_ksize'],) * 2, 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for r in cfg['target']['hsv_ranges']:
        mask |= cv2.inRange(hsv, np.array(r['lower'], np.uint8), np.array(r['upper'], np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p['morph_kernel'],) * 2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=p['open_iterations'])
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=p['close_iterations'])


def contour_depth(contour, depth, d):
    """컨투어 안 뎁스 중앙값(m). 유효 픽셀 비율이 min_valid_ratio 미만이면 None.
    경계 픽셀은 배경 뎁스가 섞이므로 침식해서 뺀다. 1 m의 작은 목표는 침식 후 남는 게 없으면 침식 전을 쓴다."""
    m = np.zeros(depth.data.shape, np.uint8)
    cv2.drawContours(m, [contour], -1, 255, -1)
    inner = cv2.erode(m, np.ones((3, 3), np.uint8), iterations=d['erode'])
    z = depth.data[(inner if inner.any() else m) > 0]
    valid = z[z > 0]
    if len(z) == 0 or len(valid) < d['min_valid_ratio'] * len(z):
        return None
    return float(np.median(valid)) * depth.scale


def evaluate(contour, frame_area, s, depth=None, d=None):
    """d는 depth_cfg(cfg). depth가 있으면 픽셀 최소 면적은 d['min_area_px'](잡음 바닥)만 쓰고
    크기 판단은 실제 크기 검증에 맡긴다 — 1 m에서 3×3 cm 면은 수백 px라 min_area로는 걸러진다."""
    use_depth = depth is not None and d is not None
    area = cv2.contourArea(contour)
    m = cv2.moments(contour)
    if m['m00'] == 0:
        return Candidate(contour, area, None, 0.0, False, 'zero_moment')
    center = (m['m10'] / m['m00'], m['m01'] / m['m00'])
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    solidity = area / hull_area if hull_area else 0.0
    if area < (d['min_area_px'] if use_depth else s['min_area']):
        return Candidate(contour, area, center, solidity, False, 'too_small')
    if area > s['max_area_ratio'] * frame_area:
        return Candidate(contour, area, center, solidity, False, 'too_large')
    if solidity < s['min_solidity']:
        return Candidate(contour, area, center, solidity, False, 'low_solidity')
    if not use_depth:
        return Candidate(contour, area, center, solidity, True, 'ok')
    z = contour_depth(contour, depth, d)
    if z is None:
        # 최소 측정거리보다 가깝거나 반사면이라 뎁스가 없다. 색만으로 믿을 만큼 크면(기존 min_area) 통과
        ok = area >= s['min_area']
        return Candidate(contour, area, center, solidity, ok, 'ok' if ok else 'no_depth')
    if not d['min_m'] <= z <= d['max_m']:
        return Candidate(contour, area, center, solidity, False, 'depth_range', z)
    # 핀홀: 실제 길이 = 픽셀 길이 × Z / f. 회전에 무관하도록 최소 외접 회전사각형의 변을 쓴다
    f = (depth.fx + depth.fy) / 2
    w, h = cv2.minAreaRect(contour)[1]
    size = (min(w, h) * z / f, max(w, h) * z / f)
    area_m2 = area * z * z / (depth.fx * depth.fy)
    # 가림이 있으면 작아질 수 있으므로 하한은 면적만 느슨하게, 상한은 변·면적 모두 본다
    ok = (size[0] <= d['max_short_m'] and size[1] <= d['max_long_m']
          and d['area_m2'][0] <= area_m2 <= d['area_m2'][1])
    return Candidate(contour, area, center, solidity, ok, 'ok' if ok else 'size_mismatch', z, size, area_m2)


def select(accepted, image_center, tie_ratio):
    """가장 큰 면적. 면적 차이가 tie_ratio 이내인 후보끼리는 영상 중심에 가까운 것."""
    if not accepted:
        return None
    largest = max(c.area for c in accepted)
    close = [c for c in accepted if c.area >= largest * (1 - tie_ratio)]
    return min(close, key=lambda c: np.hypot(c.center[0] - image_center[0],
                                              c.center[1] - image_center[1]))


def normalized_error(center_original, frame_size):
    """ex=(cx-W/2)/(W/2), ey=(cy-H/2)/(H/2). W·H는 실제 프레임 크기, 오른쪽·아래가 양수."""
    (cx, cy), (W, H) = center_original, frame_size
    return ((cx - W / 2) / (W / 2), (cy - H / 2) / (H / 2))


def detect(frame, cfg, scale=(1.0, 1.0), depth=None):
    """frame은 preprocess를 거친 BGR 영상, scale은 preprocess가 돌려준 원본→처리 배율.
    depth는 처리 해상도에 맞춘 DepthFrame(preprocess_depth) 또는 None(뎁스 검증 생략).
    반환 (Detection, mask)."""
    h, w = frame.shape[:2]
    frame_size = (round(w / scale[0]), round(h / scale[1]))
    image_center = (w / 2, h / 2)
    mask = color_mask(frame, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    d = depth_cfg(cfg)
    candidates = [evaluate(c, w * h, cfg['selection'], depth, d) for c in contours]
    accepted = [c for c in candidates if c.accepted]
    best = select(accepted, image_center, cfg['selection']['tie_ratio'])
    if best is None:
        # 미검출: 좌표·오차 모두 None. 이전 프레임 값으로 채우지 않는다.
        return Detection(False, frame_size, image_center, candidates=candidates), mask
    center_original = to_original(best.center, scale)
    return Detection(
        True, frame_size, image_center,
        center=best.center,
        center_original=center_original,
        error=normalized_error(center_original, frame_size),
        area=best.area,
        area_ratio=best.area / (w * h),   # 리사이즈해도 면적비는 같다
        bbox=cv2.boundingRect(best.contour),
        num_candidates=len(accepted),
        depth_m=best.depth_m,
        size_m=best.size_m,
        candidates=candidates,
        selected=best,
    ), mask


def draw(frame, det, label=''):
    """원본 위에 컨투어(초록=선택, 노랑=통과 후보, 회색=탈락)·목표 중심·영상 중심을 그린다."""
    out = frame.copy()
    for c in det.candidates:
        if c.area < 50:
            continue  # 아주 작은 잡음은 그리지 않는다
        color = (0, 220, 255) if c.accepted else (150, 150, 150)
        cv2.drawContours(out, [c.contour], -1, color, 1)
    line2 = None
    icx, icy = map(round, det.image_center)
    cv2.drawMarker(out, (icx, icy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
    cv2.drawMarker(out, (icx, icy), (0, 0, 0), cv2.MARKER_CROSS, 30, 1)
    if det.detected:
        cv2.drawContours(out, [det.selected.contour], -1, (0, 255, 0), 2)
        cx, cy = map(round, det.center)
        cv2.circle(out, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(out, (icx, icy), (cx, cy), (0, 0, 255), 1)
        ex, ey = det.error
        ox, oy = map(round, det.center_original)
        text = f'DETECTED c=({ox},{oy}) ex={ex:+.3f} ey={ey:+.3f} area={det.area_ratio:.2%} n={det.num_candidates}'
        if det.depth_m is not None:   # 640 폭에 한 줄로는 넘치므로 둘째 줄
            line2 = f'Z={det.depth_m:.3f}m size={det.size_m[0] * 100:.1f}x{det.size_m[1] * 100:.1f}cm'
        color = (0, 160, 0)
    else:
        text = 'NO TARGET  center=None ex=None ey=None'
        color = (0, 0, 255)
    cv2.rectangle(out, (0, 0), (out.shape[1], 46 if line2 else 26), (255, 255, 255), -1)
    cv2.putText(out, (label + '  ' if label else '') + text, (6, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if line2:
        cv2.putText(out, line2, (6, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def hsv_ranges_from_trackbars(lo, hi):
    """lo/hi = [H,S,V]. H low > H high이면 0/179를 넘는 범위(빨강)로 보고 두 범위로 나눈다."""
    lo, hi = list(map(int, lo)), list(map(int, hi))
    if lo[0] <= hi[0]:
        return [{'lower': lo, 'upper': hi}]
    return [{'lower': lo, 'upper': [179] + hi[1:]},
            {'lower': [0] + lo[1:], 'upper': hi}]


def trackbars_from_hsv_ranges(ranges):
    """hsv_ranges_from_trackbars의 역변환. 반환 (lo, hi)."""
    if len(ranges) == 2 and ranges[0]['upper'][0] == 179 and ranges[1]['lower'][0] == 0:
        return list(ranges[0]['lower']), [ranges[1]['upper'][0]] + list(ranges[0]['upper'][1:])
    return list(ranges[0]['lower']), list(ranges[0]['upper'])


def suggest_hsv(hsv_pixels, h_margin=5, sv_margin=30, pct=5):
    """목표 영역 HSV 픽셀(N×3)에서 범위를 제안한다. 반환 (lo, hi) — 트랙바 형식."""
    px = hsv_pixels.reshape(-1, 3).astype(np.int32)
    h = px[:, 0]
    # 빨강처럼 0과 179 양쪽에 걸치면 90만큼 돌려서 백분위를 구한 뒤 되돌린다
    wrap = np.mean(h < 15) > 0.1 and np.mean(h > 164) > 0.1
    hs = (h + 90) % 180 if wrap else h
    h_lo, h_hi = np.percentile(hs, [pct, 100 - pct])
    h_lo, h_hi = int(h_lo) - h_margin, int(np.ceil(h_hi)) + h_margin
    if wrap:
        h_lo, h_hi = (h_lo - 90) % 180, (h_hi - 90) % 180
    else:
        h_lo, h_hi = max(0, h_lo), min(179, h_hi)
    s_lo = max(0, int(np.percentile(px[:, 1], pct)) - sv_margin)
    v_lo = max(0, int(np.percentile(px[:, 2], pct)) - sv_margin)
    return [h_lo, s_lo, v_lo], [h_hi, 255, 255]
