#!/usr/bin/env python3
"""
make_prm.py  -  Build a PRM path on an occupancy-grid map (.pgm + .yaml).

usage:
  ./make_prm.py --map ~/maps/office.yaml --start 0 0 --goal 2 1.5 \
                --samples 800 --radius 0.5 --out ~/maps/office_prm.yaml
"""

import cv2, yaml, argparse, math, os, numpy as np
from ompl import base as ob
from ompl import geometric as og

# ---------- map utils ----------
def load_map(yaml_file):
    with open(yaml_file) as f:
        info = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_file), info["image"])
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(img_path)
    occ = (img > info["occupied_thresh"] * 255).astype(np.uint8)  # 1 = free
    return occ, info["resolution"], info["origin"]  # (x0, y0, yaw0)

def world_to_map(pt, origin, res):
    mx = int((pt[0] - origin[0]) / res)
    my = int((pt[1] - origin[1]) / res)
    return mx, my

# ---------- PRM planner ----------
def plan_prm(occ, res, origin, start, goal,
             n_samples=500, connect_radius=0.5):
    h, w = occ.shape
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, origin[0]);          bounds.setHigh(0, origin[0] + w * res)
    bounds.setLow(1, origin[1]);          bounds.setHigh(1, origin[1] + h * res)
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)

    # free pixel ⇒ valid state
    def validity(state):
        mx, my = world_to_map((state[0], state[1]), origin, res)
        return 0 <= mx < w and 0 <= my < h and occ[my, mx] == 1
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validity))

    prm = og.PRM(ss.getSpaceInformation())
    prm.setMaxNearestNeighbors(10)
    prm.setConnectionStrategy(og.connectionstrategy.kNearest(connect_radius / res))
    ss.setPlanner(prm)

    start_st, goal_st = ob.State(space), ob.State(space)
    start_st[0], start_st[1] = start
    goal_st[0], goal_st[1] = goal
    ss.setStartAndGoalStates(start_st, goal_st, 0.05)

    # generate samples
    prm.clear()
    prm.constructRoadmap(ob.PlannerTerminationConditionFn(
        lambda: prm.numMilestone() >= n_samples))

    if not ss.solve(1.0):
        raise RuntimeError("PRM failed to find a path")

    path_st = ss.getSolutionPath()
    pts = [[st[i] for i in range(2)] for st in path_st.getStates()]
    return pts

# ---------- save ----------
def save_yaml(points, out):
    data = {'poses': [[x, y, 0.0] for x, y in points]}
    with open(out, 'w') as f:
        yaml.safe_dump(data, f)
    print(f"Saved {len(points)} waypoints -> {out}")

# ---------- main ----------
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='map YAML')
    ap.add_argument('--start', nargs=2, type=float, required=True)
    ap.add_argument('--goal',  nargs=2, type=float, required=True)
    ap.add_argument('--samples', type=int, default=500)
    ap.add_argument('--radius',  type=float, default=0.5)
    ap.add_argument('--out',     default='prm_path.yaml')
    args = ap.parse_args()

    occ, res, origin = load_map(args.map)
    pts = plan_prm(occ, res, origin, args.start, args.goal,
                   args.samples, args.radius)
    save_yaml(pts, args.out)
