"""공통 플래너 모듈 — 실험 A용.

비교 대상:
  - astar        : occupancy grid 위 8-연결 A* (사실상 최적 경로 = ground truth)
  - prm_custom   : 종합설계.md에 기술된 자체 구현 (균일 샘플링 + KDTree kNN + Dijkstra,
                   Bresenham식 픽셀 해상도 충돌검사, 선택적 obstacle inflation)
  - repo plan_prm: FishFarm-robotics/myagv-nav2-prm/scripts/make_prm.py 원본 (OMPL PRM)

맵 해석 두 가지:
  - mode="repo"  : make_prm.py 원본 그대로 (img > occupied_thresh*255 → free).
                   unknown(205) 픽셀이 free로 분류됨 — 검증 대상.
  - mode="strict": 표준 ROS map_server 해석 (free_thresh 기준, img > 191만 free).
"""
import heapq
import os
import time

import cv2
import numpy as np
import yaml
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra as cs_dijkstra
from scipy.spatial import cKDTree

def load_map(yaml_file, mode="strict"):
    with open(yaml_file) as f:
        info = yaml.safe_load(f)
    img = cv2.imread(os.path.join(os.path.dirname(yaml_file), info["image"]),
                     cv2.IMREAD_GRAYSCALE)
    if mode == "repo":       # make_prm.py 원본 로직 재현 (unknown=205도 free로 분류됨)
        free = img > info["occupied_thresh"] * 255
    elif mode == "strict":   # 표준 해석: free 확정 픽셀만 (trinary: 254=free, 205=unknown)
        free = (img > (1.0 - info["free_thresh"]) * 255) & (img != 205)
    else:
        raise ValueError(mode)
    return free.astype(bool), img, info["resolution"], tuple(info["origin"][:2])


def inflate(free, radius_m, res):
    """장애물(=not free)을 로봇 반경만큼 팽창."""
    if radius_m <= 0:
        return free
    r = max(1, int(round(radius_m / res)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    return cv2.erode(free.astype(np.uint8), kernel).astype(bool)


def world_to_px(pts_w, origin, res):
    return ((np.asarray(pts_w, dtype=float) - origin) / res).astype(int)


# ---------- A* (ground truth) ----------
_SQRT2 = 2 ** 0.5
_MOVES = [(-1, -1, _SQRT2), (-1, 0, 1), (-1, 1, _SQRT2), (0, -1, 1),
          (0, 1, 1), (1, -1, _SQRT2), (1, 0, 1), (1, 1, _SQRT2)]


def astar(free, start_px, goal_px, res):
    """8-연결 A*. 반환: (world경로 없이) 경로 길이[m] 또는 None. 대각선 코너컷 금지."""
    h, w = free.shape
    sx, sy = start_px
    gx, gy = goal_px
    if not (free[sy, sx] and free[gy, gx]):
        return None
    dist = {(sx, sy): 0.0}
    pq = [(0.0, sx, sy)]
    while pq:
        f, x, y = heapq.heappop(pq)
        if (x, y) == (gx, gy):
            return dist[(x, y)] * res
        for dx, dy, c in _MOVES:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h and free[ny, nx]):
                continue
            if dx and dy and not (free[y, nx] and free[ny, x]):  # corner cut 방지
                continue
            nd = dist[(x, y)] + c
            if nd < dist.get((nx, ny), np.inf):
                dist[(nx, ny)] = nd
                heapq.heappush(pq, (nd + np.hypot(gx - nx, gy - ny), nx, ny))
    return None


# ---------- 자체 PRM (KDTree + Dijkstra) ----------
def _edges_collision_free(free, res, origin, p0, p1):
    """모든 엣지를 한 번에 res/2 간격으로 샘플링해 free 여부 판정 (완전 벡터화)."""
    seg = np.linalg.norm(p1 - p0, axis=1)
    m = np.maximum(2, np.ceil(seg / (res * 0.5)).astype(int) + 1)
    edge_id = np.repeat(np.arange(len(seg)), m)
    starts = np.concatenate([[0], np.cumsum(m)[:-1]])
    local = np.arange(m.sum()) - starts[edge_id]
    t = local / (m[edge_id] - 1)
    pts = p0[edge_id] + t[:, None] * (p1 - p0)[edge_id]
    px = ((pts - origin) / res).astype(int)
    h, w = free.shape
    inb = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
    bad = ~inb | ~free[np.clip(px[:, 1], 0, h - 1), np.clip(px[:, 0], 0, w - 1)]
    return np.bincount(edge_id, weights=bad, minlength=len(seg)) == 0


def prm_custom(free, res, origin, start_w, goal_w, n_samples,
               k=10, seed=0, inflate_m=0.0):
    """자체 PRM. 반환 dict: success, path(world Nx2)|None, time_s."""
    t0 = time.perf_counter()
    grid = inflate(free, inflate_m, res)
    rng = np.random.default_rng(seed)
    fy, fx = np.nonzero(grid)
    if len(fx) == 0:
        return {"success": False, "path": None, "time_s": time.perf_counter() - t0}
    sel = rng.choice(len(fx), size=min(n_samples, len(fx)), replace=False)
    jitter = rng.uniform(0, 1, (len(sel), 2))
    pts = np.column_stack([fx[sel], fy[sel]]) + jitter  # 픽셀 내 균일 샘플
    pts_w = pts * res + origin
    pts_w = np.vstack([pts_w, start_w, goal_w])
    n = len(pts_w)

    tree = cKDTree(pts_w)
    _, idxs = tree.query(pts_w, k=min(k + 1, n))
    ii = np.repeat(np.arange(n), idxs.shape[1] - 1)
    jj = idxs[:, 1:].ravel()
    keep = ii < jj  # 중복 제거
    ii, jj = ii[keep], jj[keep]
    ok = _edges_collision_free(grid, res, origin, pts_w[ii], pts_w[jj])
    ii, jj = ii[ok], jj[ok]
    w_e = np.linalg.norm(pts_w[ii] - pts_w[jj], axis=1)

    g = csr_matrix((np.concatenate([w_e, w_e]),
                    (np.concatenate([ii, jj]), np.concatenate([jj, ii]))),
                   shape=(n, n))
    s_idx, g_idx = n - 2, n - 1
    dist, pred = cs_dijkstra(g, indices=s_idx, return_predecessors=True)
    if not np.isfinite(dist[g_idx]):
        return {"success": False, "path": None, "time_s": time.perf_counter() - t0}
    node, path_idx = g_idx, []
    while node != -9999:
        path_idx.append(node)
        node = pred[node]
    return {"success": True, "path": pts_w[path_idx[::-1]],
            "time_s": time.perf_counter() - t0}


# ---------- repo make_prm.py (OMPL) 충실 포팅 ----------
# 원본은 구형 Py++ 바인딩(ROS 데스크톱) 대상이라 현행 pip ompl(pybind11)에서
# 그대로 실행 불가. 로직 동일 유지, API 이름만 치환:
#   StateValidityCheckerFn → 콜러블 직접 전달
#   numMilestone()         → milestoneCount()
#   PlannerTerminationConditionFn → PlannerTerminationCondition
#   connectionstrategy.kNearest   → 미노출(제거). setMaxNearestNeighbors(10)는 유지.
def repo_plan(map_yaml, start_w, goal_w, n_samples, radius=0.5, seed=None):
    """원본 plan_prm 로직 실행. 반환 dict: success, path, time_s, error."""
    try:
        from ompl import base as ob, geometric as og, util as ou
    except ImportError as e:  # ompl 미설치 시 이 비교만 건너뜀
        return {"success": False, "path": None, "time_s": 0.0,
                "error": f"ompl not installed: {e}"}
    if seed is not None:
        ou.RNG.setSeed(max(1, seed))  # OMPL은 seed 0 거부
    free, _, res, origin = load_map(map_yaml, mode="repo")
    h, w = free.shape
    t0 = time.perf_counter()
    try:
        space = ob.RealVectorStateSpace(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, origin[0]); bounds.setHigh(0, origin[0] + w * res)
        bounds.setLow(1, origin[1]); bounds.setHigh(1, origin[1] + h * res)
        space.setBounds(bounds)
        ss = og.SimpleSetup(space)

        def validity(state):
            mx = int((state[0] - origin[0]) / res)
            my = int((state[1] - origin[1]) / res)
            return 0 <= mx < w and 0 <= my < h and bool(free[my, mx])
        ss.setStateValidityChecker(validity)

        prm = og.PRM(ss.getSpaceInformation())
        prm.setMaxNearestNeighbors(10)
        ss.setPlanner(prm)

        start_st, goal_st = space.allocState(), space.allocState()
        start_st[0], start_st[1] = float(start_w[0]), float(start_w[1])
        goal_st[0], goal_st[1] = float(goal_w[0]), float(goal_w[1])
        ss.setStartAndGoalStates(start_st, goal_st, 0.05)
        ss.setup()  # 현행 바인딩은 constructRoadmap 전 명시 setup 필요

        prm.clear()
        prm.constructRoadmap(ob.PlannerTerminationCondition(
            lambda: prm.milestoneCount() >= n_samples))
        if not ss.solve(1.0):
            raise RuntimeError("PRM failed to find a path")
        path_st = ss.getSolutionPath()
        pts = [[st[0], st[1]] for st in path_st.getStates()]
        return {"success": True, "path": np.array(pts),
                "time_s": time.perf_counter() - t0, "error": ""}
    except Exception as e:  # 실패도 데이터
        return {"success": False, "path": None,
                "time_s": time.perf_counter() - t0, "error": str(e)}


# ---------- 경로 품질 지표 ----------
def path_metrics(path_w, img, res, origin):
    """길이, 최소 이격거리(occupied 기준), unknown 통과 비율, occupied 침범 수."""
    p = np.asarray(path_w, dtype=float)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    length = float(seg.sum())
    # 경로를 res/2 간격으로 리샘플
    m = np.maximum(2, np.ceil(seg / (res * 0.5)).astype(int) + 1)
    samples = [np.linspace(p[i], p[i + 1], m[i]) for i in range(len(seg))]
    s = np.vstack(samples) if samples else p
    px = ((s - origin) / res).astype(int)
    h, w = img.shape
    px = px[(px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)]
    vals = img[px[:, 1], px[:, 0]]
    # occupied(0 근방)만 장애물로 본 distance transform
    occ_mask = (img < 89).astype(np.uint8)  # p>0.65 → occupied
    dt = cv2.distanceTransform(1 - occ_mask, cv2.DIST_L2, 5) * res
    clear = dt[px[:, 1], px[:, 0]]
    return {"len_m": length,
            "min_clear_m": float(clear.min()) if len(clear) else np.nan,
            "frac_unknown": float((vals == 205).mean()) if len(vals) else np.nan,
            "n_occ_hits": int((vals < 89).sum())}
