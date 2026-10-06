"""블루큐브 인식률 평가 — 조명(어두움·보통·밝음) × 거리(0.1~1 m), 같은 데이터로 설정 여러 개를 비교한다.

데이터
  --sim             실제 큐브 영상 1장에서 조건을 모의 생성(빠른 비교용, 실제 조건과 다름)
  --dataset DIR     tuning collect로 모은 실제 데이터(DIR/<라벨>/meta.json + frame_*.png)

판정: 목표 프레임에서 검출 중심이 정답 상자(GT) 안이면 TP, 아니면 FN(엉뚱한 곳 검출은 wrong으로 따로 셈).
      목표 없는 프레임에서 검출되면 FP.

  ros2 run target_perception evaluate --sim                                  # 패키지 config/detector.yaml
  ros2 run target_perception evaluate --dataset <ws>/results/perception/dataset --config A.yaml B.yaml
"""
import argparse
import json
import time
import zlib
from pathlib import Path

import cv2
import numpy as np

from target_perception import detector
from tracking_common import interface as I

BASE_FRAME = detector.package_path('data', 'sim_cube.png')             # 실제 D435 640×480(큐브를 손에 든 장면)
NEG_FRAME = detector.package_path('data', 'sim_negative_shirt.png')    # 실제 D435 640×480(파란 줄무늬 셔츠, 큐브 없음)
FACE_MM = (30.0, 60.0)   # 기준 영상에서 보이는 큐브 면(30×60 옆면) — 모의 거리 환산에만 쓴다

LIGHT = {
    # 이름: (밝기 배율, 베일 글레어 0~1, 잡음 표준편차) — 어두움은 자동노출 이득 잡음, 밝음은 과노출·산란광
    'dark3':   (0.12, 0.00, 4.5),
    'dark2':   (0.22, 0.00, 3.5),
    'dark1':   (0.45, 0.00, 2.5),
    'normal':  (1.00, 0.00, 1.5),
    'bright1': (1.80, 0.05, 1.5),
    'bright2': (2.80, 0.12, 1.5),
    'bright3': (4.00, 0.20, 1.5),
}
DISTANCES = [0.1, 0.2, 0.35, 0.5, 0.75, 1.0]


# ---------------- 모의 데이터 ----------------
def base_frames():
    f = cv2.imread(str(BASE_FRAME))
    if f is None:
        raise SystemExit(f'기준 영상 없음: {BASE_FRAME}')
    # tune이 그려 둔 분홍 측정 상자를 지운다(주변 큐브 색으로 채움)
    pink = cv2.inRange(f, (200, 0, 200), (255, 80, 255))
    f = cv2.inpaint(f, cv2.dilate(pink, np.ones((3, 3), np.uint8)), 3, cv2.INPAINT_TELEA)
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (95, 150, 20), (130, 255, 255))
    n, _, st, _ = cv2.connectedComponentsWithStats(m)
    i = 1 + int(np.argmax(st[1:, 4]))
    gt = tuple(int(v) for v in st[i, :4])             # x, y, w, h
    x, y, w, h = gt
    hole = np.zeros(m.shape, np.uint8)
    hole[max(0, y - 4):y + h + 4, max(0, x - 4):x + w + 4] = 255
    neg = cv2.inpaint(f, hole, 7, cv2.INPAINT_TELEA)  # 같은 장면에서 큐브만 지운 음성 영상
    negs = [neg]
    s = cv2.imread(str(NEG_FRAME))
    if s is not None:
        negs.append(s)
    return f, gt, negs


def scale_about(img, gt, scale, center):
    """center를 중심으로 scale배 확대(가까이)·축소(멀리). gt 상자도 같이 변환. 화면 크기는 유지."""
    H, W = img.shape[:2]
    cx, cy = center
    M = np.float32([[scale, 0, W / 2 - scale * cx], [0, scale, H / 2 - scale * cy]])
    interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    out = cv2.warpAffine(img, M, (W, H), flags=interp, borderMode=cv2.BORDER_REPLICATE)  # 반사하면 큐브가 복제된다
    if gt is None:
        return out, None
    x, y, w, h = gt
    nx, ny = scale * x + M[0, 2], scale * y + M[1, 2]
    return out, (nx, ny, w * scale, h * scale)


def light(img, name, rng, glare_spot=None):
    g, veil, sigma = LIGHT[name]
    f = img.astype(np.float32) * g + veil * 255.0
    if glare_spot is not None and name.startswith('bright'):
        # 큐브 면의 정반사 하이라이트(밝을수록 크고 강함)
        (cx, cy), r, k = glare_spot
        yy, xx = np.mgrid[:f.shape[0], :f.shape[1]]
        spot = np.exp(-(((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * r * r)))
        f += (k * 255.0 * spot)[..., None]
    f += rng.normal(0, sigma, f.shape)
    return np.clip(f, 0, 255).astype(np.uint8)


def sim_samples(seeds=6):
    pos, gt, negs = base_frames()
    x, y, w, h = gt
    center = (x + w / 2, y + h / 2)
    # 화면 넓이 = 면 넓이 × (fx/거리)² → 거리 = fx × √(면 넓이) / √(화면 넓이)
    d_now = 615.0 * np.sqrt(FACE_MM[0] * FACE_MM[1]) / 1000 / np.sqrt(w * h)
    samples = []
    for d in DISTANCES:
        sc = d_now / d
        img, box = scale_about(pos, gt, sc, center)
        bx, by, bw, bh = box
        for ln in LIGHT:
            amp = {'bright1': .5, 'bright2': .8, 'bright3': 1.0}.get(ln, 0)
            spot = ((bx + bw * .35, by + bh * .3), max(2.0, .18 * min(bw, bh)), amp)
            for s in range(seeds):
                rng = np.random.default_rng(zlib.crc32(f'{d}|{ln}|{s}'.encode()))  # hash()는 실행마다 바뀐다
                samples.append(dict(label=f'{ln}@{d}m', light=ln, distance=d, positive=True,
                                    frame=light(img, ln, rng, spot), gt=box))
        for k, neg in enumerate(negs):
            nimg, _ = scale_about(neg, None, sc, center)
            for ln in LIGHT:
                rng = np.random.default_rng(zlib.crc32(f'neg{k}|{d}|{ln}'.encode()))
                samples.append(dict(label=f'neg{k}:{ln}@{d}m', light=ln, distance=d, positive=False,
                                    frame=light(nimg, ln, rng), gt=None))
    return samples, dict(base=str(BASE_FRAME), base_gt=gt, base_distance_m=round(d_now, 2),
                         visible_face_mm=FACE_MM)


# ---------------- 실제 데이터 ----------------
def dataset_samples(root, use_depth=True):
    """collect가 뎁스도 저장했으면(depth_NNN.png + meta.depth) 노드와 같이 뎁스 검증까지 해서 평가한다."""
    samples = []
    for meta_p in sorted(Path(root).glob('*/meta.json')):
        meta = json.loads(meta_p.read_text(encoding='utf-8'))
        dinfo = meta.get('depth') if use_depth else None
        for fp in sorted(meta_p.parent.glob('frame_*.png')):
            frame = cv2.imread(str(fp))
            depth = None
            dp = fp.with_name(fp.name.replace('frame_', 'depth_'))
            if dinfo and dp.exists():
                depth = detector.DepthFrame(cv2.imread(str(dp), cv2.IMREAD_UNCHANGED),
                                            dinfo['scale'], dinfo['fx'], dinfo['fy'])
            samples.append(dict(label=meta['label'], light=meta.get('light', '?'),
                                distance=meta.get('distance_m'), positive=meta['gt'] is not None,
                                frame=frame, gt=meta['gt'], file=str(fp), depth=depth))
    if not samples:
        raise SystemExit(f'{root}에 데이터가 없다. ros2 run target_perception tuning collect --camera 로 모은다.')
    return samples, dict(dataset=str(root), depth_frames=sum(s['depth'] is not None for s in samples))


# ---------------- 평가 ----------------
def judge(cfg, s):
    t0 = time.perf_counter()
    frame, scale = detector.preprocess(s['frame'], cfg)
    det, mask = detector.detect(frame, cfg, scale, detector.preprocess_depth(s.get('depth'), scale))
    ms = (time.perf_counter() - t0) * 1000
    if not s['positive']:
        return ('FP' if det.detected else 'TN'), ms, det, mask
    if not det.detected:
        return 'FN', ms, det, mask
    x, y, w, h = s['gt']
    cx, cy = det.center_original
    pad = 0.15 * max(w, h)
    inside = x - pad <= cx <= x + w + pad and y - pad <= cy <= y + h + pad
    return ('TP' if inside else 'WRONG'), ms, det, mask


def evaluate(cfg, samples):
    res = {}
    for s in samples:
        v, ms, det, _ = judge(cfg, s)
        key = (s['light'], s['distance'], s['positive'])
        r = res.setdefault(key, dict(n=0, TP=0, FN=0, WRONG=0, FP=0, TN=0, ms=[], reasons={}, cerr=[]))
        r['n'] += 1
        r[v] += 1
        if v == 'TP':
            x, y, w, h = s['gt']
            cx, cy = det.center_original
            r['cerr'].append(np.hypot(cx - (x + w / 2), cy - (y + h / 2)) / max(w, h))
        r['ms'].append(ms)
        if v in ('FN', 'WRONG'):
            for c in det.candidates:
                if c.area >= 30:
                    r['reasons'][c.reason] = r['reasons'].get(c.reason, 0) + 1
    return res


def table(name, res, lights, dists):
    pos = {k: v for k, v in res.items() if k[2]}
    neg = {k: v for k, v in res.items() if not k[2]}
    lines = [f'### {name}\n', '인식률 (TP / 목표 프레임). 괄호 = 엉뚱한 곳 검출 수\n',
             '| 조명 \\ 거리 | ' + ' | '.join(f'{d} m' for d in dists) + ' | 평균 |',
             '|---|' + '---|' * (len(dists) + 1)]
    for ln in lights:
        cells, rates = [], []
        for d in dists:
            r = pos.get((ln, d, True))
            if not r:
                cells.append('-')
                continue
            rate = r['TP'] / r['n']
            rates.append(rate)
            cells.append(f"{rate:.0%}" + (f" ({r['WRONG']})" if r['WRONG'] else ''))
        lines.append(f'| {ln} | ' + ' | '.join(cells) + f" | {np.mean(rates):.0%} |" if rates else '')
    tp = sum(r['TP'] for r in pos.values())
    n = sum(r['n'] for r in pos.values())
    fp = sum(r['FP'] for r in neg.values())
    nn = sum(r['n'] for r in neg.values())
    ms = [m for r in res.values() for m in r['ms']]
    ce = [e for r in pos.values() for e in r['cerr']]
    ce_mean, ce_p95 = (float(np.mean(ce)), float(np.percentile(ce, 95))) if ce else (None, None)
    lines.append(f"\n전체 인식률 **{tp}/{n} = {tp / n:.1%}** · 오검출(목표 없는 프레임) **{fp}/{nn} = "
                 f"{(fp / nn if nn else 0):.1%}** · 처리 {np.mean(ms):.2f} ms/프레임"
                 + (f" · 중심 오차(정답 상자 장변 대비) 평균 {ce_mean:.1%}, p95 {ce_p95:.1%}" if ce else '') + '\n')
    if fp:
        bad = sorted(((k, v['FP']) for k, v in neg.items() if v['FP']), key=lambda kv: -kv[1])[:6]
        lines.append('오검출 조건: ' + ', '.join(f'{k[0]}@{k[1]}m×{c}' for k, c in bad) + '\n')
    return '\n'.join(lines), dict(recall=tp / n, fp_rate=fp / nn if nn else 0, ms=float(np.mean(ms)),
                                  center_err_mean=ce_mean, center_err_p95=ce_p95)


def per_light(res):
    out = {}
    for (ln, d, positive), r in res.items():
        if positive:
            t = out.setdefault(ln, [0, 0])
            t[0] += r['TP']
            t[1] += r['n']
    return {k: v[0] / v[1] for k, v in out.items()}


def ablations(path):
    """개선 설정에서 한 가지씩 되돌린 변형."""
    import copy
    base = detector.load_config(path)
    raw = detector.yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    out = []

    def variant(name, fn):
        c = copy.deepcopy(raw)
        fn(c)
        detector.resolve_size_limits(c)
        out.append((name, c))
    variant('-relaxed(히스테리시스)', lambda c: c['target'].pop('relaxed_ranges', None))
    variant('-min_chroma', lambda c: c['target'].pop('min_chroma', None))
    variant('-V하한8→26', lambda c: [r['lower'].__setitem__(2, 26) for r in c['target']['hsv_ranges'] + c['target'].get('relaxed_ranges', [])])
    variant('-S하한170→208', lambda c: [r['lower'].__setitem__(1, 208) for r in c['target']['hsv_ranges']])
    variant('-auto면적(800 고정)', lambda c: c['selection'].update(min_area=800, max_area_ratio=0.6))
    variant('-형태필터', lambda c: [c['selection'].pop(k, None) for k in ('max_aspect', 'min_extent')])
    return out


def montage(cfgs, samples, out, dists, lights):
    """설정별로 조명×거리 대표 장면의 검출 결과를 한 장으로."""
    pick = [s for s in samples if s['positive']]
    seen, rows = set(), {}
    for s in pick:
        k = (s['light'], s['distance'])
        if k in seen:
            continue
        seen.add(k)
        rows.setdefault(s['light'], {})[s['distance']] = s
    for name, cfg in cfgs:
        grid = []
        for ln in lights:
            tiles = []
            for d in dists:
                s = rows.get(ln, {}).get(d)
                if s is None:
                    tiles.append(np.full((180, 240, 3), 40, np.uint8))   # 빠진 조건(다른 칸과 같은 크기)
                    continue
                v, _, det, _ = judge(cfg, s)
                frame, g = detector.preprocess(s['frame'], cfg)
                img = detector.draw(frame, det)
                x, y, w, h = s['gt']                       # 원본 좌표 → 처리 영상 좌표(자르기·배율)
                x, y, w, h = map(int, ((x - g.ox) * g.sx, (y - g.oy) * g.sy, w * g.sx, h * g.sy))
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                img = cv2.resize(img, (240, 180))
                color = (0, 200, 0) if v == 'TP' else (0, 0, 255)
                cv2.rectangle(img, (0, 150), (240, 180), (255, 255, 255), -1)
                cv2.putText(img, f'{ln} {d}m {v}', (4, 172), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                tiles.append(img)
            grid.append(np.hstack(tiles))
        cv2.imwrite(str(out / f'montage_{name}.png'), np.vstack(grid))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument('--sim', action='store_true')
    src.add_argument('--dataset', type=Path)
    p.add_argument('--config', type=Path, nargs='+', default=[detector.DEFAULT_CONFIG])
    p.add_argument('--out', type=Path, default=I.results_dir() / 'perception' / 'eval',
                   help='결과 폴더(기본 <ws>/results/perception/eval)')
    p.add_argument('--seeds', type=int, default=6)
    p.add_argument('--ablation', action='store_true', help='마지막 --config에서 개선을 하나씩 뺀 변형도 평가')
    p.add_argument('--no-depth', action='store_true', help='--dataset: 저장된 뎁스를 쓰지 않고 색·모양만으로 평가')
    a = p.parse_args()
    samples, info = sim_samples(a.seeds) if a.sim else dataset_samples(a.dataset, not a.no_depth)
    present = {s['light'] for s in samples}
    order = list(LIGHT) + ['dark', 'normal', 'bright']           # 모의 이름 + 실제 수집 이름 순서
    lights = [l for l in dict.fromkeys(order) if l in present] + sorted(present - set(order))
    dists = sorted({s['distance'] for s in samples if s['distance'] is not None})
    a.out.mkdir(parents=True, exist_ok=True)
    md = [f"# 블루큐브 인식률 평가 ({'모의' if a.sim else '실제 데이터'})\n",
          f"데이터: {json.dumps(info, ensure_ascii=False)} · 목표 프레임 {sum(s['positive'] for s in samples)}장, "
          f"목표 없는 프레임 {sum(not s['positive'] for s in samples)}장\n"]
    if a.sim:
        md.append('> 모의 조건은 실제 영상 1장을 밝기·잡음·산란광·하이라이트·확대/축소로 변형한 것이다. '
                  '실제 조명의 색온도 변화·자동노출 동작·초점 흐림·원근은 반영하지 않는다. 최종 판단은 실제 데이터로 한다.\n')
    summary, cfgs = {}, []
    jobs = [(cp.stem, cp, detector.load_config(cp)) for cp in a.config]
    if a.ablation:
        jobs += [(n, a.config[-1], c) for n, c in ablations(a.config[-1])]
    for name, cp, cfg in jobs:
        cfgs.append((name, cfg))
        text, s = table(f'{name} (`{cp.name}`{" 변형" if name.startswith("-") else ""}) — min_area={cfg["selection"]["min_area"]}, '
                        f'max_area_ratio={cfg["selection"]["max_area_ratio"]}', evaluate(cfg, samples), lights, dists)
        md.append(text)
        s['per_light'] = per_light(evaluate(cfg, samples)) if a.ablation else None
        summary[name] = s
        print(f'{name:12s} 인식률 {s["recall"]:.1%}  오검출 {s["fp_rate"]:.1%}  {s["ms"]:.2f} ms  '
              f'중심오차 평균 {s["center_err_mean"] or 0:.1%} p95 {s["center_err_p95"] or 0:.1%}')
    if a.ablation:
        md.append('## 개선 항목별 기여 (하나씩 뺐을 때)\n')
        md.append('| 설정 | 전체 인식률 | 오검출 | ' + ' | '.join(lights) + ' | ms |')
        md.append('|---|---|---|' + '---|' * len(lights) + '---|')
        for n, v in summary.items():
            pl = v['per_light'] or {}
            md.append(f"| {n} | {v['recall']:.1%} | {v['fp_rate']:.1%} | " +
                      ' | '.join(f"{pl.get(l, 0):.0%}" for l in lights) + f" | {v['ms']:.2f} |")
        md.append('')
    montage([c for c in cfgs if not c[0].startswith('-')], samples, a.out, dists, lights)
    tag = 'sim' if a.sim else 'dataset'
    (a.out / f'eval_{tag}.md').write_text('\n'.join(md), encoding='utf-8')
    (a.out / f'eval_{tag}.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('저장:', a.out / f'eval_{tag}.md')


if __name__ == '__main__':
    main()
