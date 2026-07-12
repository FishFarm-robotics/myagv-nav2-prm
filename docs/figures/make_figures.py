"""README 차트 생성 → docs/figures/*.png

experiments/의 CSV와 레포의 map/에서 차트 4종을 만든다.
  chart1_map_interpretation.png : "검정만 회피" 설계가 의미하는 것 (unknown 92%)
  chart2_wall_tunneling.png     : OMPL 충돌검사 터널링 실측 + 도식
  chart3_a2_narrow_passage.png  : 문 폭 × 샘플 수 → 성공률
  chart4_a3_safety.png          : 플래너별 안전 지표

usage: python docs/figures/make_figures.py  (레포 루트에서)
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = "AppleGothic"   # macOS 한글 (리눅스: NanumGothic 등으로 교체)
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
EXP = os.path.join(ROOT, "experiments")
MAP = os.path.join(ROOT, "map", "map.yaml")
sys.path.insert(0, EXP)
import planners as P
from exp_a1_samples_sweep import pick_pairs


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


free_s, img, res, origin = P.load_map(MAP, "strict")
free_r, _, _, _ = P.load_map(MAP, "repo")
origin = np.array(origin)
n_occ = int((img < 89).sum())
n_unk = int((img == 205).sum())
n_free = int(free_s.sum())
total = img.size
a3 = read(os.path.join(EXP, "exp_a3_safety_margin.csv"))

# ---------- 차트 1 ----------
fig, axes = plt.subplots(1, 2, figsize=(13, 6.5))
crop = slice(100, 480), slice(60, 480)
for ax, grid, title in [
        (axes[0], free_r, f"당시 설계: \"검정만 회피\" → 주행 가능 {free_r.sum()/total:.0%}"),
        (axes[1], free_s, f"표준 해석: 확인된 free만 → 주행 가능 {n_free/total:.1%}")]:
    disp = np.full(img.shape, 0.55)
    disp[img < 89] = 0.0
    disp[grid] = 1.0
    ax.imshow(disp[crop], cmap="gray", vmin=0, vmax=1)
    ax.set_title(title, fontsize=13, pad=10)
    ax.set_xticks([]); ax.set_yticks([])
axes[0].text(0.02, 0.02, "흰색 = 플래너가 지나갈 수 있다고 믿는 영역", transform=axes[0].transAxes,
             fontsize=10, color="tab:red", va="bottom",
             bbox=dict(fc="white", alpha=0.85, ec="none"))
fig.suptitle(f"픽셀 분포: occupied {n_occ:,} / unknown {n_unk:,} ({n_unk/total:.0%}) / free {n_free:,}",
             fontsize=14, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(os.path.join(HERE, "chart1_map_interpretation.png"), dpi=150, bbox_inches="tight")
plt.close(fig)

# ---------- 차트 2 ----------
pairs = pick_pairs(free_s, res, origin)
s_px, g_px = pairs[0]
s_w, g_w = np.array(s_px) * res + origin, np.array(g_px) * res + origin
for seed in range(1, 11):  # OMPL 전역 RNG 비결정 → 집계(침범 100%)와 일치하는 경로 확보
    repo = P.repo_plan(MAP, s_w, g_w, 4000, seed=seed)
    if repo["success"] and P.path_metrics(repo["path"], img, res, origin)["n_occ_hits"] > 0:
        break
cust = P.prm_custom(free_s, res, origin, s_w, g_w, 4000, k=30, seed=1, inflate_m=0.2)

fig = plt.figure(figsize=(13, 6.5))
gs = fig.add_gridspec(2, 2, width_ratios=[1.3, 1])
ax = fig.add_subplot(gs[:, 0])
disp = np.full(img.shape, 0.75)
disp[img == 205] = 0.55
disp[img < 89] = 0.0
disp[free_s] = 1.0
ax.imshow(disp, cmap="gray", vmin=0, vmax=1)
for r, c, lbl in [(repo, "tab:red", "make_prm.py(OMPL) 경로"),
                  (cust, "tab:green", "런타임 방식 + inflation 0.2m")]:
    if r["success"]:
        px = (r["path"] - origin) / res
        ax.plot(px[:, 0], px[:, 1], "-o", ms=3, lw=2, color=c, label=lbl)
p = repo["path"]
seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
samples = np.vstack([np.linspace(p[i], p[i+1], max(2, int(seg[i]/(res*0.5))+1))
                     for i in range(len(seg))])
spx = ((samples - origin) / res).astype(int)
hit = img[spx[:, 1], spx[:, 0]] < 89
ax.plot(spx[hit, 0], spx[hit, 1], "x", ms=9, mew=2.5, color="crimson",
        label=f"벽 침범 지점 ({hit.sum()}곳)")
ax.plot(*s_px, "k^", ms=11); ax.plot(*g_px, "k*", ms=15)
ax.set_xlim(120, 330); ax.set_ylim(320, 120)
ax.legend(loc="lower right", fontsize=10)
ax.set_title("미인지 결함 실측 — 같은 시작/목표, 4000노드", fontsize=13)
ax.set_xticks([]); ax.set_yticks([])

ax2 = fig.add_subplot(gs[0, 1])
ax2.axhspan(-0.5, 0.5, xmin=0.47, xmax=0.50, color="k")
xs = np.arange(0, 1.61, 0.47)
ax2.plot(xs, np.zeros_like(xs), "o-", color="tab:red", ms=10)
for x in xs:
    ax2.annotate("검사", (x, 0), textcoords="offset points", xytext=(0, 12),
                 ha="center", fontsize=9, color="tab:red")
ax2.annotate("벽 5cm", (0.77, 0.52), fontsize=10, ha="center",
             bbox=dict(fc="white", ec="none"))
ax2.annotate("", xy=(0.47, -0.25), xytext=(0, -0.25),
             arrowprops=dict(arrowstyle="<->", color="gray"))
ax2.text(0.235, -0.42, "검사 간격 ≈ 0.47m\n(공간 폭의 1%)", fontsize=9, color="gray", ha="center")
ax2.set_xlim(-0.15, 1.75); ax2.set_ylim(-0.6, 0.6)
ax2.axis("off")
ax2.set_title("OMPL 기본 충돌검사: 검사점이 5cm 벽을 건너뜀", fontsize=11)

ax3 = fig.add_subplot(gs[1, 1])
row_r = next(r for r in a3 if r["planner"] == "repo_ompl" and r["samples"] == "4000")
row_i = next(r for r in a3 if r["planner"] == "custom_inflated" and r["samples"] == "4000")
txt = (f"4000노드 집계 (실측 스캔 맵 5쌍, k=30)\n\n"
       f"                       OMPL 도구    런타임 방식\n"
       f"occupied 침범 경로   {float(row_r['pct_paths_hitting_occupied']):.0%}         {float(row_i['pct_paths_hitting_occupied']):.0%}\n"
       f"최악 이격거리          {row_r['worst_min_clear_m']}m        {row_i['worst_min_clear_m']}m\n"
       f"미탐사 공간 통과       {float(row_r['mean_frac_unknown']):.0%}          {float(row_i['mean_frac_unknown']):.0%}\n\n"
       f"침범 판정 = 경로를 2.5cm 간격 재샘플링해 occupied 픽셀 위 여부 실측")
ax3.text(0.0, 0.5, txt, fontsize=10.5, family="AppleGothic", va="center")
ax3.axis("off")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "chart2_wall_tunneling.png"), dpi=150, bbox_inches="tight")
plt.close(fig)

# ---------- 차트 3 ----------
a2 = read(os.path.join(EXP, "exp_a2_narrow_passage.csv"))
doors = sorted({r["door_w_m"] for r in a2}, key=float)
fig, ax = plt.subplots(figsize=(8.5, 5.5))
cmap = plt.cm.viridis(np.linspace(0, 0.9, len(doors)))
for d, c in zip(doors, cmap):
    rows = [r for r in a2 if r["door_w_m"] == d]
    xs = [int(r["samples"]) for r in rows]
    ys = [float(r["success_rate"]) * 100 for r in rows]
    ax.plot(xs, ys, "o-", color=c, label=f"문 폭 {d}m" + (" (로봇 폭)" if d == "0.3" else ""))
ax.axhline(95, ls=":", color="gray", lw=1)
ax.set_xscale("log"); ax.set_xticks([250, 500, 1000, 2000, 4000])
ax.get_xaxis().set_major_formatter(plt.ScalarFormatter())
ax.get_xaxis().set_minor_formatter(plt.NullFormatter())
ax.tick_params(axis="x", which="minor", length=0)
ax.set_xlabel("PRM 샘플 수"); ax.set_ylabel("통과 성공률 (%)")
ax.set_title("좁은 통로 통과 성공률 — \"4000노드\" 설계의 실측 근거와 한계\n(합성 맵, 20시드/조건, k=30)")
ax.legend(fontsize=9); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "chart3_a2_narrow_passage.png"), dpi=150, bbox_inches="tight")
plt.close(fig)

# ---------- 차트 4 ----------
fig, axes = plt.subplots(1, 2, figsize=(11, 5))
names = [("repo_ompl", "make_prm.py\n(OMPL 도구)"), ("custom_strict", "런타임 방식\n(inflation 없음)"),
         ("custom_inflated", "런타임 방식\n+ inflation 0.2m")]
colors = ["tab:red", "tab:orange", "tab:green"]
rows4k = [next(r for r in a3 if r["planner"] == p_ and r["samples"] == "4000") for p_, _ in names]
axes[0].bar([n for _, n in names], [float(r["worst_min_clear_m"]) for r in rows4k], color=colors)
axes[0].axhline(0.2, ls="--", color="k", lw=1)
axes[0].text(2.45, 0.202, "safe zone 기준 0.2m", fontsize=9, ha="right", va="bottom")
axes[0].set_ylabel("최악 최소 이격거리 (m)")
axes[0].set_title("장애물과의 최악 이격거리 (4000노드)")
axes[1].bar([n for _, n in names], [float(r["pct_paths_hitting_occupied"]) * 100 for r in rows4k], color=colors)
axes[1].set_ylabel("occupied 셀 침범 경로 비율 (%)")
axes[1].set_title("벽을 침범한 경로 비율 (4000노드)")
for ax in axes:
    ax.grid(axis="y", alpha=0.3)
fig.suptitle("안전 지표 비교 — inflation은 길이 손해 없이 이격을 보장", fontsize=13)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "chart4_a3_safety.png"), dpi=150, bbox_inches="tight")
plt.close(fig)

print("saved 4 charts →", HERE)
