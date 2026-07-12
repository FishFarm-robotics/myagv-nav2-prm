"""실험 A-2: 좁은 통로 합성 맵에서 샘플 수 vs 통과 성공률.

검증 대상 주장 (종합설계.md):
  "샘플 수가 적으면 좁은 통로를 놓친다 → 4000노드로 커버리지 확보"

맵: 10×10 m (200×200 px, res 0.05), 중앙 세로벽(두께 0.25 m)에
    폭 w인 문 하나. 시작(좌) → 목표(우).
문 폭 스윕: 0.10~0.80 m (로봇 폭 0.30 m 대비 0.33×~2.7×).
출력: exp_a2_narrow_passage.csv
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import planners as P

HERE = os.path.dirname(__file__)
RES = 0.05
N = 200
WALL_X, WALL_T = 100, 5              # 벽 위치/두께 [px]
DOOR_W_PX = [2, 3, 4, 6, 10, 16]     # 0.10, 0.15, 0.20, 0.30, 0.50, 0.80 m
SAMPLES = [250, 500, 1000, 2000, 4000]
SEEDS = range(1, 21)


def make_map(door_px):
    free = np.ones((N, N), dtype=bool)
    free[:, WALL_X:WALL_X + WALL_T] = False
    lo = N // 2 - door_px // 2
    free[lo:lo + door_px, WALL_X:WALL_X + WALL_T] = True
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False  # 외벽
    return free


def main():
    origin = np.array([0.0, 0.0])
    s_w, g_w = np.array([25, 100]) * RES, np.array([175, 100]) * RES
    rows = []
    for door_px in DOOR_W_PX:
        free = make_map(door_px)
        gt = P.astar(free, (25, 100), (175, 100), RES)
        for ns in SAMPLES:
            ok, times, ratios = 0, [], []
            for seed in SEEDS:
                r = P.prm_custom(free, RES, origin, s_w, g_w, ns, k=30, seed=seed)
                times.append(r["time_s"])
                if r["success"]:
                    ok += 1
                    seg = np.linalg.norm(np.diff(r["path"], axis=0), axis=1).sum()
                    ratios.append(seg / gt)
            rows.append([round(door_px * RES, 2), ns, round(ok / len(SEEDS), 3),
                         round(float(np.mean(ratios)), 3) if ratios else "",
                         round(float(np.mean(times)), 4)])
            print(f"door {door_px*RES:.2f} m, samples {ns}: "
                  f"success {ok}/{len(SEEDS)}")
    out = os.path.join(HERE, "exp_a2_narrow_passage.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["door_w_m", "samples", "success_rate",
                    "mean_ratio_vs_astar", "mean_time_s"])
        w.writerows(rows)
    print("saved", out)


if __name__ == "__main__":
    main()
