"""HSV·Contour 검출 파이프라인.

영상 → (리사이즈) → HSV 변환 → 색상 마스크 → 잡음 제거 → 컨투어 → (뎁스 검증) → 대상 선택 → 중심 계산
뎁스 검증: 컨투어 안 뎁스 중앙값 Z로 거리 범위와 실제 크기(px × Z / f)를 확인한다.
상태를 갖지 않는다: 매 프레임 독립적으로 판단하므로 미검출 시 이전 좌표가 섞여 들어올 수 없다.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np
import yaml

def package_path(*parts):
    """xmen_tracker 패키지 파일 경로. 설치(share)에 있으면 그것을, 없으면 소스 위치를 쓴다.
    --symlink-install이면 share의 파일은 소스(저장소) 파일을 가리킨다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('xmen_tracker')).joinpath(*parts)
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent.joinpath(*parts)


DEFAULT_CONFIG = package_path('config', 'detector.yaml')


def load_config(path=DEFAULT_CONFIG):
    cfg = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    t = cfg['target']
    for r in t['hsv_ranges'] + (t.get('relaxed_ranges') or []):
        if len(r['lower']) != 3 or len(r['upper']) != 3:
            raise ValueError('hsv_ranges/relaxed_ranges의 lower/upper는 [H,S,V] 세 값이어야 한다.')
    if cfg['processing']['blur_ksize'] % 2 == 0 and cfg['processing']['blur_ksize'] != 0:
        raise ValueError('blur_ksize는 0 또는 홀수여야 한다.')
    d = cfg.get('depth')
    if d and d['enabled']:
        if not 0 < d['min_m'] < d['max_m']:
            raise ValueError('depth: 0 < min_m < max_m 이어야 한다.')
        if not 0 < d['area_m2'][0] < d['area_m2'][1]:
            raise ValueError('depth.area_m2는 [하한, 상한]이어야 한다.')
    resolve_size_limits(cfg)
    return cfg


def depth_cfg(cfg):
    """뎁스 검증 설정. 꺼져 있으면 None."""
    d = cfg.get('depth')
    return d if d and d['enabled'] else None


def focal_px(cfg):
    fx = cfg['camera'].get('fx_px') or 615.0          # D435 컬러 640×480 근사값
    return fx * cfg['processing']['width'] / cfg['camera'].get('fx_width', 640)


def faces_mm2(cfg):
    """size_mm: 숫자(정육면체 한 변) 또는 [가로, 세로, 높이]. 반환 (가장 작은 면, 가장 큰 면) 넓이 [mm²]."""
    size = cfg['target']['size_mm']
    a, b, c = sorted([float(size)] * 3 if isinstance(size, (int, float)) else map(float, size))
    return a * b, b * c


def expected_area_px(cfg, distance_m, face_mm2):
    """핀홀 모델: 넓이 face_mm2인 면이 distance_m에서 정면으로 보일 때 화면에서의 넓이 [px²]."""
    return face_mm2 * (focal_px(cfg) / (distance_m * 1000.0)) ** 2


def resolve_size_limits(cfg):
    """selection.min_area: auto → 가장 먼 거리에서 가장 작은 면(30×30)이 보일 때 넓이 × min_area_factor.
    selection.max_area_ratio: auto → 가장 가까운 거리에서 가장 큰 면(30×60)이 보일 때 넓이 기준.
    min_area_factor(기본 0.3)는 비스듬히 보이거나 일부가 가려질 때를 위한 여유다."""
    s, t, p = cfg['selection'], cfg['target'], cfg['processing']
    prev = s.get('auto') or {}
    auto_min = s.get('min_area') == 'auto' or prev.get('min_area_auto')       # 재계산(실제 fx 반영) 지원
    auto_max = s.get('max_area_ratio') == 'auto' or prev.get('max_area_ratio_auto')
    s['auto'] = {}
    if 'size_mm' not in t or 'distance_range_m' not in t:
        if auto_min or auto_max:
            raise ValueError('auto 크기 제한에는 target.size_mm과 target.distance_range_m이 필요하다.')
        return
    near, far = t['distance_range_m']
    small_face, big_face = faces_mm2(cfg)
    frame_area = p['width'] * p['height']
    far_area = expected_area_px(cfg, far, small_face)
    near_area = expected_area_px(cfg, near, big_face)
    s['auto'] = dict(far_small_face_px=round(far_area), near_big_face_px=round(near_area))
    if auto_min:
        s['min_area'] = max(30, int(s.get('min_area_factor', 0.3) * far_area))
        s['auto']['min_area_auto'] = True
    if auto_max:
        # 가까울 때 두 면이 보이거나 화면 밖으로 나가도 버리지 않게 1.6배, 상한 0.95
        s['max_area_ratio'] = round(min(0.95, max(0.6, 1.6 * near_area / frame_area)), 3)
        s['auto']['max_area_ratio_auto'] = True


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
    split: bool = False     # size_mismatch 덩어리를 뎁스 층으로 나눈 조각이면 True
    partial: str = None     # 일부만 보이는 큐브로 통과: 'border'(화면 가장자리에 잘림) | 'occluded'(앞 물체에 가림)


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
    partial: str = None           # 'border' | 'occluded' | None(전체가 보임)
    candidates: list = field(default_factory=list, repr=False)
    selected: Candidate = field(default=None, repr=False)

    def summary(self):
        """JSON 저장용. 컨투어 배열은 제외한다."""
        keys = ('detected', 'frame_size', 'image_center', 'center', 'center_original', 'error',
                'area', 'area_ratio', 'bbox', 'num_candidates', 'depth_m', 'size_m', 'partial')
        d = {k: getattr(self, k) for k in keys}
        d['rejected'] = [c.reason for c in self.candidates if not c.accepted]
        return d


class Geometry(NamedTuple):
    """원본 → 처리 영상 변환. 처리 좌표 u = (x − ox + .5)·sx − .5. W·H = 원본(실제 프레임) 크기."""
    sx: float
    sy: float
    ox: int = 0
    oy: int = 0
    W: int = None
    H: int = None
    cw: int = None     # 자른 영역 크기(원본 px). None이면 자르지 않음
    ch: int = None


def as_geometry(scale):
    """예전 형식 (sx, sy) 튜플도 받는다."""
    return scale if isinstance(scale, Geometry) else Geometry(float(scale[0]), float(scale[1]))


def preprocess(frame, cfg):
    """처리 해상도(예: 640×360)로 맞춘다. 반환 (처리 영상, Geometry).
    가로세로 비가 다르면(640×480 사진 → 16:9) 늘리거나 누르지 않고 가운데를 잘라 비율을 맞춘 뒤 줄인다.
    누르면 정사각형이 직사각형이 되어 모양 검사(장변/단변)와 크기 계산이 틀어진다."""
    p = cfg['processing']
    tw, th = p['width'], p['height']
    h, w = frame.shape[:2]
    if (w, h) == (tw, th):
        return frame, Geometry(1.0, 1.0, 0, 0, w, h)
    ox = oy = 0
    cw, ch = w, h
    if abs(w * th - h * tw) > max(w, h):          # 비율이 1픽셀 이상 다르면 자른다
        if w * th > h * tw:                       # 원본이 더 넓다 → 좌우를 자른다
            cw = round(h * tw / th)
            ox = (w - cw) // 2
        else:                                     # 원본이 더 높다(4:3 → 16:9) → 위아래를 자른다
            ch = round(w * th / tw)
            oy = (h - ch) // 2
    crop = frame[oy:oy + ch, ox:ox + cw]
    small = crop if (cw, ch) == (tw, th) else cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA)
    return small, Geometry(tw / cw, th / ch, ox, oy, w, h, cw, ch)


def preprocess_depth(depth, scale):
    """뎁스를 컬러와 같은 방식으로 자르고 줄인다. 0(측정 실패)이 실제 거리와 섞이지 않게 INTER_NEAREST,
    초점거리도 같은 배율(자르기는 초점거리를 바꾸지 않는다)."""
    g = as_geometry(scale)
    if depth is None or (g.sx, g.sy, g.ox, g.oy) == (1.0, 1.0, 0, 0):
        return depth
    h, w = depth.data.shape
    cw, ch = g.cw or w, g.ch or h
    crop = depth.data[g.oy:g.oy + ch, g.ox:g.ox + cw]
    data = cv2.resize(crop, (round(cw * g.sx), round(ch * g.sy)), interpolation=cv2.INTER_NEAREST)
    return DepthFrame(data, depth.scale, depth.fx * g.sx, depth.fy * g.sy)


def to_original(point, scale):
    """처리 해상도 좌표를 원본 좌표로 복원한다(픽셀 중심 규약 + 자른 만큼 더함, examples/10 참고)."""
    if point is None:
        return None
    g = as_geometry(scale)
    u, v = point
    return ((u + .5) / g.sx - .5 + g.ox, (v + .5) / g.sy - .5 + g.oy)


def in_ranges(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for r in ranges:
        mask |= cv2.inRange(hsv, np.array(r['lower'], np.uint8), np.array(r['upper'], np.uint8))
    return mask


def color_mask(frame, cfg):
    """엄격 범위(hsv_ranges) 마스크 + 선택적 개선 두 가지.
    - min_chroma: 색 세기(max−min, 0~255)가 이보다 낮은 픽셀 제외. 어두운 픽셀은 S=(max−min)/max의 분모가
      작아 잡음만으로 S가 커지므로, V 하한을 낮출 때 잡음이 목표색으로 들어오는 것을 막는다.
    - relaxed_ranges(히스테리시스): 완화 범위 픽셀은 엄격 범위 픽셀과 연결된 덩어리일 때만 인정.
      그늘진 면·하이라이트 가장자리처럼 목표 일부만 범위를 벗어난 경우를 되살리고,
      완화 범위만 만족하는 배경 덩어리(셔츠 단추 등)는 씨앗이 없어 버려진다."""
    p, t = cfg['processing'], cfg['target']
    if p['blur_ksize']:
        frame = cv2.GaussianBlur(frame, (p['blur_ksize'],) * 2, 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = in_ranges(hsv, t['hsv_ranges'])
    chroma_ok = None
    if t.get('min_chroma'):
        b, g, r = cv2.split(frame)     # numpy max(axis=2)보다 약 80배 빠르다(14 ms → 0.2 ms)
        chroma = cv2.subtract(cv2.max(cv2.max(b, g), r), cv2.min(cv2.min(b, g), r))
        chroma_ok = cv2.inRange(chroma, t['min_chroma'], 255)
        mask &= chroma_ok
    if t.get('relaxed_ranges'):
        weak = in_ranges(hsv, t['relaxed_ranges']) | mask
        if chroma_ok is not None:
            weak &= chroma_ok
        n, labels = cv2.connectedComponents(weak, connectivity=8)
        if n > 1:
            seeds = np.unique(labels[mask > 0])
            keep = np.zeros(n, bool)
            keep[seeds[seeds > 0]] = True
            mask = keep.take(labels).astype(np.uint8) * 255
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
    valid = z[np.isfinite(z) & (z > 0)]
    if len(z) == 0 or len(valid) < d['min_valid_ratio'] * len(z):
        return None
    return float(np.median(valid)) * depth.scale


def box_fit_gap(contour, max_vertices=6):
    """상자 모양 정도. 볼록껍질을 꼭짓점 max_vertices개 이하의 다각형으로 근사했을 때
    '평균 틈' = (껍질 넓이 − 다각형 넓이) ÷ 둘레 ÷ √(껍질 넓이)와 껍질 넓이(px)를 반환한다.
    상자(직육면체) 실루엣은 어느 방향에서도 꼭짓점 4~6개인 볼록 다각형이라 틈이 모서리에만 조금 생기고,
    원·타원·고리처럼 테두리 전체가 곡선인 물체는 둘레 전체에 틈이 생긴다.
    (가장 크게 벗어난 한 점을 보면 흐림으로 둥글어진 모서리 하나 때문에 상자도 원처럼 보여 평균을 쓴다.)"""
    hull = cv2.convexHull(contour)
    ha = cv2.contourArea(hull)
    if ha <= 0:
        return 1.0, 0.0
    peri = cv2.arcLength(hull, True)
    lo, hi = 0.0, peri
    for _ in range(20):                         # 이분 탐색: 꼭짓점 ≤ max_vertices가 되는 최소 근사 오차
        mid = (lo + hi) / 2
        if len(cv2.approxPolyDP(hull, mid, True)) <= max_vertices:
            hi = mid
        else:
            lo = mid
    poly = cv2.approxPolyDP(hull, hi, True)
    return (ha - cv2.contourArea(poly)) / peri / np.sqrt(ha), ha


def shape_reason(contour, area, solidity, s):
    """모양 검사. 통과면 None. 큐브(30×30×60)는 어느 방향에서도 가늘고 길지 않다(옆면 2:1, 반쯤 가려도 ~2.5:1).
    box_fit: 실루엣이 꼭짓점 6개 이하의 다각형(사각 면으로 된 상자)에 맞지 않으면 'not_box'(box_fit_gap).
    뎁스 유무와 상관없이 항상 적용한다 — 뎁스가 없을 때 건너뛰면 줄무늬·막대가 그대로 통과한다."""
    (_, (rw, rh), _) = cv2.minAreaRect(contour)
    if 'max_aspect' in s and min(rw, rh) > 0 and max(rw, rh) / min(rw, rh) > s['max_aspect']:
        return 'bad_aspect'
    if 'min_extent' in s and rw * rh > 0 and area / (rw * rh) < s['min_extent']:
        return 'low_extent'
    if solidity < s['min_solidity']:
        return 'low_solidity'
    bf = s.get('box_fit')
    if bf and bf.get('enabled', True):
        # 허용 틈 = base + pixel/√넓이(작게 보일수록 픽셀 계단·흐림으로 모서리가 둥글어지므로 더 허용).
        # min_px보다 작으면 해상도가 부족해 원과 상자를 구분할 수 없으므로 검사하지 않는다(큐브를 놓치지 않게).
        gap, ha = box_fit_gap(contour, bf.get('max_vertices', 6))
        if ha >= bf.get('min_px', 500) and gap > bf['base'] + bf['pixel'] / np.sqrt(ha):
            return 'not_box'
    return None


def touches_border(contour, frame_wh, px, max_angle_deg=30.0, elongated=1.5):
    """화면 가장자리에서 잘린 큐브로 볼 수 있으면 True.
    가장자리(px 이내)에 닿고, 길쭉한 덩어리(장단비 ≥ elongated)라면 긴 변이 닿은 가장자리와 나란해야 한다
    (왼쪽·오른쪽 → 세로, 위·아래 → 가로). 가장자리에 끝만 닿은 가로 띠(청바지 주름 등)는 잘림으로 보지 않는다."""
    x, y, w, h = cv2.boundingRect(contour)
    W, H = frame_wh
    sides = {'v': x <= px or x + w >= W - px, 'h': y <= px or y + h >= H - px}
    if not (sides['v'] or sides['h']):
        return False
    (_, (rw, rh), ang) = cv2.minAreaRect(contour)
    if min(rw, rh) <= 0 or max(rw, rh) / min(rw, rh) < elongated:
        return True                                   # 길쭉하지 않으면 방향과 상관없이 잘림으로 본다
    # 긴 변의 방향(0° = 가로, 90° = 세로)
    long_deg = ang if rw >= rh else ang + 90.0
    tilt_from_h = abs(((long_deg + 90.0) % 180.0) - 90.0)        # 가로에서 벌어진 각
    return (sides['v'] and tilt_from_h >= 90.0 - max_angle_deg) or (sides['h'] and tilt_from_h <= max_angle_deg)


def front_occlusion(contour, depth, z, p, occluder_ok=None):
    """보이는 모양이 앞 물체(손가락 등)에 가려 생긴 것인지 뎁스로 판단한다. 반환 (오목부 앞 비율, 둘레 앞 비율).
    - 오목부: 볼록껍질 안인데 컨투어 밖인 부분. 가려진 큐브는 여기가 큐브보다 가깝고(가린 물체),
      홈·구멍이 있는 다른 물체는 여기로 뒤쪽 배경이 보여 더 멀다.
    - 둘레: 컨투어 바로 바깥 ring_px 픽셀. 곧게 잘린 가림(가로지른 손가락)은 오목부가 없어 둘레로 본다.
    '앞' = 큐브 Z보다 max(front_margin_m, Z×front_margin_ratio) 이상 가까운 유효 뎁스.
    occluder_ok: 가린 물체로 인정할 픽셀(파란 계열이 아닌 곳). None이면 모든 픽셀."""
    m = np.zeros(depth.data.shape, np.uint8)
    cv2.drawContours(m, [contour], -1, 255, -1)
    hm = np.zeros_like(m)
    cv2.drawContours(hm, [cv2.convexHull(contour)], -1, 255, -1)
    k = np.ones((3, 3), np.uint8)
    deficit = (hm > 0) & (cv2.dilate(m, k) == 0)                 # 경계 1픽셀은 빼고
    ring = (cv2.dilate(m, k, iterations=p.get('ring_px', 4)) > 0) & (m == 0)
    zz = depth.data.astype(np.float32) * depth.scale
    margin = max(p.get('front_margin_m', 0.01), p.get('front_margin_ratio', 0.015) * z)
    if occluder_ok is not None:
        # 같은 파란 계열(청바지 주름 등 같은 물체의 다른 부분)은 가린 물체로 치지 않는다.
        # 비율의 분모에서 빼면 오히려 비율이 커지므로 '앞이 아닌 유효 픽셀'로 센다(멀리 둔다).
        zz = np.where(occluder_ok, zz, np.where(zz > 0, z + 1.0, 0))

    def front_ratio(region):
        v = zz[region]
        v = v[v > 0]
        return float(np.mean(v < z - margin)) if len(v) else 0.0
    big_deficit = deficit.sum() >= 0.05 * max(1, (hm > 0).sum())
    return (front_ratio(deficit) if big_deficit else 0.0), front_ratio(ring)


def evaluate(contour, frame_area, s, depth=None, d=None, frame_wh=None, occluder_ok=None):
    """후보 하나를 판정한다. 순서: 면적 → 모양 → (뎁스가 있으면) 거리 범위·실제 크기.
    d는 depth_cfg(cfg). 뎁스가 있으면 픽셀 최소 면적은 d['min_area_px'](잡음 바닥)만 쓰고
    크기 판단은 실제 크기 검증에 맡긴다.
    일부만 보이는 큐브(selection.partial): 모양·넓이 하한 탈락이 '화면 가장자리에 잘림' 또는
    '앞 물체에 가림(뎁스)'으로 설명될 때만 기준을 완화한다. 크기 상한은 완화하지 않는다."""
    use_depth = depth is not None and d is not None
    p = s.get('partial') or {}
    area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    # center_method: hull이면 볼록껍질 중심. 큐브는 볼록하므로 하이라이트·그늘이 가장자리를 파먹어도
    # 중심이 덜 끌려간다. 면적·모양 검사는 원래 컨투어 기준.
    m = cv2.moments(hull if s.get('center_method') == 'hull' else contour)
    if m['m00'] == 0:
        return Candidate(contour, area, None, 0.0, False, 'zero_moment')
    center = (m['m10'] / m['m00'], m['m01'] / m['m00'])
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area else 0.0
    if area < (d['min_area_px'] if use_depth else s['min_area']):
        return Candidate(contour, area, center, solidity, False, 'too_small')
    if area > s['max_area_ratio'] * frame_area:
        return Candidate(contour, area, center, solidity, False, 'too_large')
    z = contour_depth(contour, depth, d) if use_depth else None
    border = bool(p.get('enabled') and frame_wh and touches_border(contour, frame_wh, p.get('border_px', 2),
                                                                   p.get('border_max_angle_deg', 30.0)))
    occl = {}

    def occlusion():
        if 'v' not in occl:   # 필요할 때 한 번만 계산
            occl['v'] = front_occlusion(contour, depth, z, p, occluder_ok) if (p.get('enabled') and z) else (0.0, 0.0)
        return occl['v']
    partial = 'border' if border else None
    bad = shape_reason(contour, area, solidity, s)
    if bad and p.get('enabled'):
        deficit_front, ring_front = occlusion()
        concave_ok = deficit_front >= p.get('front_ratio', 0.6)              # 오목부가 앞 물체로 설명됨
        cut_ok = border or concave_ok or ring_front >= p.get('ring_front_ratio', 0.2)
        if cut_ok:
            relaxed = dict(s, max_aspect=p.get('max_aspect', 6.0))
            if concave_ok:
                relaxed.update(min_extent=p.get('min_extent', 0.35), min_solidity=p.get('min_solidity', 0.5))
            if shape_reason(contour, area, solidity, relaxed) is None:
                bad = None
                partial = partial or 'occluded'
    if bad:
        return Candidate(contour, area, center, solidity, False, bad, z, partial=partial)
    if not use_depth:
        return Candidate(contour, area, center, solidity, True, 'ok', partial=partial)
    if z is None:
        if d.get('require_valid_depth', False):
            return Candidate(contour, area, center, solidity, False, 'no_depth', partial=partial)
        # 최소 측정거리보다 가깝거나 반사면이라 뎁스가 없다. 색만으로 믿을 만큼 크면(no_depth_min_area) 통과.
        # 단, 가장 가까운 거리(min_m)에 있다고 쳐도 목표보다 크면 다른 물체다(가까이 든 같은 색 카드 등)
        if area < d.get('no_depth_min_area', s['min_area']) and not border:
            return Candidate(contour, area, center, solidity, False, 'no_depth', partial=partial)
        f = (depth.fx + depth.fy) / 2
        w, h = cv2.minAreaRect(contour)[1]
        z0 = d['min_m']
        ok = (min(w, h) * z0 / f <= d['max_short_m'] and max(w, h) * z0 / f <= d['max_long_m']
              and area * z0 * z0 / (depth.fx * depth.fy) <= d['area_m2'][1])
        return Candidate(contour, area, center, solidity, ok, 'ok' if ok else 'size_mismatch', partial=partial)
    if not d['min_m'] <= z <= d['max_m']:
        return Candidate(contour, area, center, solidity, False, 'depth_range', z, partial=partial)
    # 핀홀: 실제 길이 = 픽셀 길이 × Z / f. 회전에 무관하도록 최소 외접 회전사각형의 변을 쓴다
    f = (depth.fx + depth.fy) / 2
    w, h = cv2.minAreaRect(contour)[1]
    size = (min(w, h) * z / f, max(w, h) * z / f)
    area_m2 = area * z * z / (depth.fx * depth.fy)
    lower = d['area_m2'][0]
    if area_m2 < lower and p.get('enabled') and not partial and max(occlusion()[1], occlusion()[0]) >= p.get('ring_front_ratio', 0.2):
        partial = 'occluded'                                   # 작게 보이는 이유가 앞 물체
    if partial:
        lower = p.get('min_area_m2', 0.0001)                   # 일부만 보이면 넓이 하한을 낮춘다(상한은 그대로)
    ok = (size[0] <= d['max_short_m'] and size[1] <= d['max_long_m']
          and lower <= area_m2 <= d['area_m2'][1])
    return Candidate(contour, area, center, solidity, ok, 'ok' if ok else 'size_mismatch', z, size, area_m2,
                     partial=partial)


def split_by_depth(contour, depth, d, z=None):
    """같은 색 물체가 겹쳐 한 덩어리가 된 컨투어를 뎁스가 끊기는 경계에서 나눈다. 반환: 조각 컨투어 목록.

    3×3 이웃의 뎁스 차가 split_jump_m(또는 거리×split_jump_ratio)보다 큰 픽셀을 경계로 보고 잘라
    연결 성분마다 조각으로 만든다. 앞뒤로 떨어진 물체 사이는 뎁스가 튀어서 나뉘고,
    기울어진 카드처럼 거리가 매끄럽게 변하는 한 물체는 나뉘지 않는다(일정 두께로 자르면
    큰 물체가 목표 크기의 띠로 쪼개져 오검출된다). 같은 거리에 나란히 붙은 물체는 나뉘지 않는다.
    """
    m = np.zeros(depth.data.shape, np.uint8)
    cv2.drawContours(m, [contour], -1, 255, -1)
    if z is None:   # 점 잡음이 경계로 잡히지 않게 5×5 중앙값(프레임마다 한 번 계산해 넘겨받을 수 있다)
        z = cv2.medianBlur(depth.data, 5).astype(np.float32) * depth.scale
    valid = z > 0
    if valid[m > 0].mean() < d.get('split_min_valid', 0.6):
        return []   # 뎁스가 대부분 비면 측정 실패 영역이 경계처럼 보여 물체 일부가 목표 크기로 잘린다
    k = np.ones((3, 3), np.uint8)
    z_hi = cv2.dilate(np.where(valid, z, 0).astype(np.float32), k)
    z_lo = cv2.erode(np.where(valid, z, np.inf).astype(np.float32), k)
    jump = np.maximum(d.get('split_jump_m', 0.02), z * d.get('split_jump_ratio', 0.03))
    edge = (z_hi - z_lo) > jump
    keep = ((m > 0) & valid & ~edge).astype(np.uint8) * 255
    keep = cv2.morphologyEx(keep, cv2.MORPH_OPEN, k)
    found, _ = cv2.findContours(keep, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pieces = [c for c in found if cv2.contourArea(c) >= d['min_area_px']]
    return pieces if len(pieces) > 1 else []   # 안 나뉘었으면 원래 판정(size_mismatch) 유지


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
    """frame은 preprocess를 거친 BGR 영상, scale은 preprocess가 돌려준 Geometry(또는 (sx, sy)).
    depth는 처리 해상도에 맞춘 DepthFrame(preprocess_depth) 또는 None(뎁스 검증 생략).
    반환 (Detection, mask)."""
    h, w = frame.shape[:2]
    g = as_geometry(scale)
    frame_size = (g.W, g.H) if g.W else (round(w / g.sx), round(h / g.sy))
    image_center = (w / 2, h / 2)
    mask = color_mask(frame, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    d = depth_cfg(cfg)
    occluder_ok = None
    pp = cfg['selection'].get('partial') or {}
    if depth is not None and pp.get('enabled'):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        occluder_ok = in_ranges(hsv, pp.get('occluder_exclude_ranges',
                                            [{'lower': [95, 60, 20], 'upper': [130, 255, 255]}])) == 0
    candidates = [evaluate(c, w * h, cfg['selection'], depth, d, (w, h), occluder_ok) for c in contours]
    if depth is not None and d is not None and d.get('split', True):
        # 같은 색 물체와 겹쳐 커진 덩어리: 뎁스 층으로 나눠 조각마다 다시 검증(한 단계만).
        # 겹친 덩어리는 크기뿐 아니라 모양(L자·들쭉날쭉)으로도 탈락하므로 모양 탈락도 나눠 본다.
        split_reasons = ('size_mismatch', 'bad_aspect', 'low_extent', 'low_solidity')
        parents = [c for c in candidates if c.reason in split_reasons and c.area >= 2 * d['min_area_px']]
        zf = cv2.medianBlur(depth.data, 5).astype(np.float32) * depth.scale if parents else None
        for parent in parents:
            for piece in split_by_depth(parent.contour, depth, d, zf):
                cand = evaluate(piece, w * h, cfg['selection'], depth, d, (w, h), occluder_ok)
                cand.split = True
                candidates.append(cand)
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
        area_ratio=best.area / (g.sx * g.sy) / (frame_size[0] * frame_size[1]),   # contour_area/(W×H), 실제 프레임 기준
        bbox=cv2.boundingRect(best.contour),
        num_candidates=len(accepted),
        depth_m=best.depth_m,
        size_m=best.size_m,
        partial=best.partial,
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
        if det.partial:               # 일부만 보이는 큐브: 중심은 보이는 부분의 중심
            line2 = (line2 + '  ' if line2 else '') + f'PARTIAL({det.partial})'
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


class TrackerDetector:
    """Adapt the perception pipeline to RGB images and aligned metric depth.

    Keep the whole input image and return boxes in its pixel coordinates.
    Tracking always requires valid depth, even if offline tools allow fallback.
    Intrinsics are (fx, fy) at the input resolution; config values are a fallback.
    """

    def __init__(self, cfg):
        from copy import deepcopy
        self.config = deepcopy(cfg)
        self.config['depth'].update(enabled=True, require_valid_depth=True)

    def detect(self, rgb, depth, intrinsics=None):
        if (rgb.ndim != 3 or rgb.shape[2] != 3 or depth is None
                or depth.shape != rgb.shape[:2] or rgb.size == 0):
            return (0.0, 0.0, 0.0), None
        cfg = self.config
        height, width = rgb.shape[:2]
        cfg['processing'].update(width=width, height=height)
        fx, fy = intrinsics or (focal_px(cfg), focal_px(cfg))
        if not all(np.isfinite(f) and f > 0 for f in (fx, fy)):
            return (0.0, 0.0, 0.0), None
        resolve_size_limits(cfg)
        metric = np.where(np.isfinite(depth) & (depth > 0), depth, 0).astype(np.float32)
        result, _ = detect(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), cfg,
                           depth=DepthFrame(metric, 1.0, fx, fy))
        if not result.detected:
            return (0.0, 0.0, 0.0), None
        return (*map(float, result.error), float(result.area_ratio)), result.bbox
