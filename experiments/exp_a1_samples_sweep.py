"""실험 A-1/A-3: 실측 스캔 맵(양식장 모사 실내 환경)에서 샘플 수 스윕 + 안전 마진 검증.

검증 대상 주장 (종합설계.md):
  "4000노드 샘플링이 커버리지(좁은 통로)와 연산량의 균형점"
  "안전 마진(로봇 크기)을 occupancy grid에 반영해 협소 공간 충돌 방지"

비교: A*(GT) vs 자체 PRM(strict / strict+inflation 0.15m) vs repo OMPL PRM(원본 맵 해석).
출력: exp_a1_samples_sweep.csv (per-run), exp_a3_safety_margin.csv (플래너별 집계), exp_a1_paths.png
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import planners as P

HERE = os.path.dirname(__file__)
MAP = os.path.join(HERE, "..", "map", "map.yaml")
# 원본 prm_rviz.py(NAS, 2025-05-30) 파라미터 그대로:
#   robot_radius=0.25m, safe zone = dist > 0.8*radius, KNN k=30
ROBOT_R = 0.8 * 0.25    # = 0.2m, 원본 safe zone 기준
K = 30
SAMPLES = [250, 500, 1000, 2000, 4000]
SEEDS_CUSTOM = [1, 2, 3, 4, 5]
SAMPLES_OMPL = [500, 2000, 4000]
SEEDS_OMPL = [1, 2, 3]
N_PAIRS = 5


def pick_pairs(free_s, res, origin, n=N_PAIRS, min_len=5.0, seed=42):
    """침식된 실제 free 영역에서 A*-연결된 시작/목표 쌍 선정."""
    er = P.inflate(free_s, ROBOT_R, res)
    ys, xs = np.nonzero(er)
    rng = np.random.default_rng(seed)
    pairs = []
    while len(pairs) < n:
        i, j = rng.choice(len(xs), 2, replace=False)
        s_px, g_px = (int(xs[i]), int(ys[i])), (int(xs[j]), int(ys[j]))
        d = P.astar(er, s_px, g_px, res)
        if d and d >= min_len:
            pairs.append((s_px, g_px))
    return pairs


def main():
    free_s, img, res, origin = P.load_map(MAP, "strict")
    infl = P.inflate(free_s, ROBOT_R, res)
    pairs = pick_pairs(free_s, res, origin)
    origin = np.array(origin)

    # 쌍별 GT (A*): raw strict 기준과 inflated 기준 둘 다
    gt = []
    for s_px, g_px in pairs:
        gt.append({"raw": P.astar(free_s, s_px, g_px, res),
                   "infl": P.astar(infl, s_px, g_px, res)})

    rows = []
    keep_paths = {}  # 시각화용 (pair 0)
    for pi, (s_px, g_px) in enumerate(pairs):
        s_w, g_w = np.array(s_px) * res + origin, np.array(g_px) * res + origin

        for ns in SAMPLES:
            for seed in SEEDS_CUSTOM:
                for name, inf_m, gt_key in [("custom_strict", 0.0, "raw"),
                                            ("custom_inflated", ROBOT_R, "infl")]:
                    r = P.prm_custom(free_s, res, origin, s_w, g_w, ns,
                                     k=K, seed=seed, inflate_m=inf_m)
                    m = (P.path_metrics(r["path"], img, res, origin)
                         if r["success"] else {})
                    rows.append(dict(planner=name, pair=pi, samples=ns, seed=seed,
                                     success=int(r["success"]),
                                     time_s=round(r["time_s"], 4),
                                     len_m=round(m.get("len_m", np.nan), 3),
                                     ratio_vs_astar=round(m["len_m"] / gt[pi][gt_key], 3)
                                     if r["success"] else np.nan,
                                     min_clear_m=round(m.get("min_clear_m", np.nan), 3),
                                     frac_unknown=round(m.get("frac_unknown", np.nan), 3),
                                     n_occ_hits=m.get("n_occ_hits", ""), error=""))
                    if pi == 0 and ns == 4000 and seed == 1 and r["success"]:
                        keep_paths[name] = r["path"]

        for ns in SAMPLES_OMPL:
            for seed in SEEDS_OMPL:
                r = P.repo_plan(MAP, s_w, g_w, ns, seed=seed)
                m = (P.path_metrics(r["path"], img, res, origin)
                     if r["success"] else {})
                rows.append(dict(planner="repo_ompl", pair=pi, samples=ns, seed=seed,
                                 success=int(r["success"]),
                                 time_s=round(r["time_s"], 4),
                                 len_m=round(m.get("len_m", np.nan), 3),
                                 ratio_vs_astar=round(m["len_m"] / gt[pi]["raw"], 3)
                                 if r["success"] else np.nan,
                                 min_clear_m=round(m.get("min_clear_m", np.nan), 3),
                                 frac_unknown=round(m.get("frac_unknown", np.nan), 3),
                                 n_occ_hits=m.get("n_occ_hits", ""),
                                 error=r["error"][:80]))
                if pi == 0 and ns == 4000 and seed == 1 and r["success"]:
                    keep_paths["repo_ompl"] = r["path"]
        print(f"pair {pi} done (A* raw {gt[pi]['raw']:.2f} m, infl {gt[pi]['infl']:.2f} m)")

    out1 = os.path.join(HERE, "exp_a1_samples_sweep.csv")
    with open(out1, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print("saved", out1, f"({len(rows)} rows)")

    # ---- A-3: 플래너별 안전 지표 집계 ----
    agg = {}
    for r in rows:
        a = agg.setdefault((r["planner"], r["samples"]),
                           dict(n=0, ok=0, clear=[], unk=[], occ=0, ratio=[], t=[]))
        a["n"] += 1; a["ok"] += r["success"]
        if r["success"]:
            a["clear"].append(r["min_clear_m"]); a["unk"].append(r["frac_unknown"])
            a["occ"] += int(r["n_occ_hits"] > 0); a["ratio"].append(r["ratio_vs_astar"])
            a["t"].append(r["time_s"])
    out3 = os.path.join(HERE, "exp_a3_safety_margin.csv")
    with open(out3, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["planner", "samples", "success_rate", "mean_ratio_vs_astar",
                    "mean_min_clear_m", "worst_min_clear_m", "mean_frac_unknown",
                    "pct_paths_hitting_occupied", "mean_time_s"])
        for (pl, ns), a in sorted(agg.items()):
            ok = max(a["ok"], 1)
            w.writerow([pl, ns, round(a["ok"] / a["n"], 3),
                        round(float(np.mean(a["ratio"])), 3) if a["ratio"] else "",
                        round(float(np.mean(a["clear"])), 3) if a["clear"] else "",
                        round(float(np.min(a["clear"])), 3) if a["clear"] else "",
                        round(float(np.mean(a["unk"])), 3) if a["unk"] else "",
                        round(a["occ"] / ok, 3),
                        round(float(np.mean(a["t"])), 4) if a["t"] else ""])
    print("saved", out3)

    # ---- 시각화 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 9))
    disp = np.full(img.shape, 0.6)          # unknown 회색
    disp[img < 89] = 0.0                    # occupied 검정
    disp[free_s] = 1.0                      # free 흰색
    ax.imshow(disp, cmap="gray", origin="upper")
    colors = {"custom_strict": "tab:blue", "custom_inflated": "tab:green",
              "repo_ompl": "tab:red"}
    for name, path in keep_paths.items():
        px = (np.asarray(path) - origin) / res
        ax.plot(px[:, 0], px[:, 1], "-o", ms=2, lw=1.5,
                color=colors[name], label=name)
    s_px, g_px = pairs[0]
    ax.plot(*s_px, "k^", ms=10, label="start")
    ax.plot(*g_px, "k*", ms=14, label="goal")
    ax.legend(); ax.set_title("pair 0, 4000 samples — free(white)/unknown(gray)/occupied(black)")
    fig.savefig(os.path.join(HERE, "exp_a1_paths.png"), dpi=140, bbox_inches="tight")
    print("saved exp_a1_paths.png")


if __name__ == "__main__":
    main()
