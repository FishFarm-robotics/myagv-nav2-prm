# MyAGV Nav2 + PRM Path Planning

ROS 2 navigation package for the [MyAGV](https://www.elephantrobotics.com/en/myagv-en/)
mobile robot, built on **Nav2**, with custom offline global planners on top of a
SLAM-built occupancy grid.

The map (`map/map.pgm` + `map.yaml`) and the reference point cloud were produced
from a fish-farm environment scan.

## Contents

### Nav2 bringup
- `launch/navigation2_active.launch.py` — brings up `nav2_bringup` with this
  package's map, `param/myagv.yaml` tuning, and an RViz view.
- `param/myagv.yaml`, `rviz/*.rviz` — robot-specific Nav2 params and RViz configs.

### Custom planning scripts (`scripts/`)
- **`make_prm.py`** — standalone **PRM** planner over the occupancy grid using
  [OMPL](https://ompl.kavrakilab.org/). Loads the `.pgm`/`.yaml` map, treats free
  pixels as valid states, builds a roadmap, solves start→goal, and saves the
  waypoints as YAML.
- **`make_nav2_path.py`** — asks Nav2's `ComputePathToPose` action server for a
  path (navfn planner) and dumps it to YAML, for comparison against the PRM path.
- **`prm_path_publisher.py`** — publishes a saved waypoint YAML as a
  `nav_msgs/Path` on `/prm_path` for RViz visualization.
- **`ply_publisher.py`** — streams an Open3D `.ply` cloud as `PointCloud2` on
  `/ply_points`.

## Build & run

```bash
# in a colcon workspace
colcon build --packages-select myagv_navigation2
source install/setup.bash
ros2 launch myagv_navigation2 navigation2_active.launch.py

# offline PRM path on the map
./scripts/make_prm.py --map map/map.yaml --start 0 0 --goal 2 1.5 \
                      --samples 800 --radius 0.5 --out prm_path.yaml
ros2 run ... prm_path_publisher.py prm_path.yaml    # visualize in RViz
```

## Notes

- Requires ROS 2 + `nav2_bringup`; `make_prm.py` also needs OMPL's Python
  bindings, `ply_publisher.py` needs `open3d`.
- Some scripts have hard-coded local paths (map / `.ply` locations) — adjust to
  your setup. colcon `build/`, `install/`, `log/` artifacts are not committed.
