#!/usr/bin/env python3
"""
PRMVisualizer (디버깅/시각화용)
────────────────────────────────────────────────────────
- final_map.yaml 을 읽어 PGM 맵 로드
- Safe‑zone, 샘플링 노드, 연결된 선, 최종 경로를 모두 OpenCV 창으로 시각화
- Rviz Marker 로도 경로(LineStrip) 발행
- 시작점과 목표점을 지정된 좌표로 설정

수행 흐름
1. map_img 및 safe_zone 계산  → "Safe Zone" 창
2. 지정된 시작점/목표점 설정 및 안전지대 검증
3. 샘플 노드 찍어 보기           → "Samples" 창
4. K‑NN 연결 선 시각화           → "Connections" 창
5. PRM 탐색 결과(Rviz + OpenCV) → "PRM Path" 창

ESC를 눌러 모든 창을 닫고 노드 종료.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import numpy as np
import cv2
import networkx as nx
from scipy.spatial import KDTree
import random
import yaml
import os
import csv


class PRMVisualizer(Node):
    def __init__(self):
        super().__init__("prm_visualizer")

        # ───────── 맵 메타 읽기 ─────────
        yaml_path = "../video6.yaml"
        with open(yaml_path, "r") as f:
            meta = yaml.safe_load(f)

        self.map_path   = os.path.join(os.path.dirname(yaml_path), meta["image"])
        self.resolution = float(meta["resolution"])
        self.origin     = meta["origin"][:2]      # [x, y]
        self.robot_radius = 0.25                   # [m]

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, "prm_path_marker", 1)
        self.create_timer(1.0, self.run_prm_once)

        self.run_once = False

    # ───────── 좌표 변환 ─────────
    def world_to_map(self, pos):
        mx = int((pos[0] - self.origin[0]) / self.resolution)
        my = int((self.origin[1] + self.height * self.resolution - pos[1]) / self.resolution)
        return mx, my

    def map_to_world(self, px):
        wx = px[0] * self.resolution + self.origin[0]
        wy = self.origin[1] + self.height * self.resolution - px[1] * self.resolution
        return wx, wy

    # ───────── 가장 가까운 안전지대 점 찾기 ─────────
    def find_nearest_safe_point(self, safe_zone, target_px, search_radius=50):
        """지정된 점 주변에서 가장 가까운 안전지대 점을 찾기"""
        tx, ty = target_px
        
        # 탐색 반경 내에서 안전지대 점들 찾기
        for radius in range(1, search_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if dx*dx + dy*dy <= radius*radius:  # 원형 탐색
                        x, y = tx + dx, ty + dy
                        if (0 <= x < self.width and 0 <= y < self.height and 
                            safe_zone[y, x]):
                            return (x, y)
        return None

    # ───────── 안전지대에서 무작위 점 선택 ─────────
    def get_random_safe_point(self, safe_zone):
        """안전지대에서 무작위 점을 하나 선택"""
        safe_points = np.where(safe_zone)
        if len(safe_points[0]) == 0:
            return None
        
        idx = random.randint(0, len(safe_points[0]) - 1)
        y = safe_points[0][idx]
        x = safe_points[1][idx]
        return (x, y)

    # ───────── 메인 루프 ─────────
    def run_prm_once(self):
        if self.run_once:
            return

        # 1. 맵 로드 & safe_zone 계산
        map_img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        self.height, self.width = map_img.shape

        dist_px = cv2.distanceTransform(map_img, cv2.DIST_L2, 3)
        dist_m  = dist_px * self.resolution
        safe_zone = dist_m > (0.8 * self.robot_radius)

        # Safe‑zone 시각화
        cv2.imshow("Safe Zone", (safe_zone.astype(np.uint8) * 255))
        cv2.imwrite(os.path.join(os.path.expanduser("~/ply_files/prmRviz2/image"), "safe_zone.png"), (safe_zone.astype(np.uint8) * 255))
        cv2.waitKey(1)

        # 2. 시작점과 목표점 지정 (월드 좌표)
        start_world = (0, -1)       # 시작점: (0, -1)
        goal_world = (-0.1, -3)     # 목표점: (-0.1, -3)
        
        # 월드 좌표를 픽셀 좌표로 변환
        start_px = self.world_to_map(start_world)
        goal_px = self.world_to_map(goal_world)
        
        # 안전지대 검증 함수
        def is_in_safe_zone(px):
            x, y = px
            return (0 <= x < self.width and 0 <= y < self.height and safe_zone[y, x])
        
        # 지정된 점들이 안전지대에 있는지 확인
        if not is_in_safe_zone(start_px):
            self.get_logger().error(f"시작점 {start_world} → 픽셀{start_px}이 안전지대에 없습니다!")
            # 가까운 안전지대 점 찾기
            start_px = self.find_nearest_safe_point(safe_zone, start_px)
            if start_px is None:
                self.get_logger().error("시작점 근처에 안전지대를 찾을 수 없습니다.")
                self.run_once = True
                return
            start_world = self.map_to_world(start_px)
            self.get_logger().info(f"시작점을 가까운 안전지대로 이동: ({start_world[0]:.2f}, {start_world[1]:.2f})")
        
        if not is_in_safe_zone(goal_px):
            self.get_logger().error(f"목표점 {goal_world} → 픽셀{goal_px}이 안전지대에 없습니다!")
            # 가까운 안전지대 점 찾기
            goal_px = self.find_nearest_safe_point(safe_zone, goal_px)
            if goal_px is None:
                self.get_logger().error("목표점 근처에 안전지대를 찾을 수 없습니다.")
                self.run_once = True
                return
            goal_world = self.map_to_world(goal_px)
            self.get_logger().info(f"목표점을 가까운 안전지대로 이동: ({goal_world[0]:.2f}, {goal_world[1]:.2f})")

        # 선택된 시작점과 목표점을 로그 출력
        self.get_logger().info(f"시작점 (픽셀): {start_px}, (월드): ({start_world[0]:.2f}, {start_world[1]:.2f})")
        self.get_logger().info(f"목표점 (픽셀): {goal_px}, (월드): ({goal_world[0]:.2f}, {goal_world[1]:.2f})")
        
        # 두 점 사이의 거리 확인
        distance_px = np.linalg.norm(np.subtract(start_px, goal_px))
        distance_m = distance_px * self.resolution
        self.get_logger().info(f"시작점-목표점 거리: {distance_px:.1f}px ({distance_m:.2f}m)")

        # 3. 샘플 노드 생성 (랜덤 샘플 + 시작/목표)
        N = 4000
        samples = []
        while len(samples) < N:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            if safe_zone[y, x]:
                samples.append((x, y))

        # 시작점과 목표점 추가
        samples.extend([start_px, goal_px])
        s_idx = len(samples) - 2
        g_idx = len(samples) - 1

        # 샘플 노드 시각화
        sample_vis = cv2.cvtColor(map_img.copy(), cv2.COLOR_GRAY2BGR)
        for x, y in samples[:-2]:  # 일반 샘플들
            cv2.circle(sample_vis, (x, y), 1, (128, 128, 128), -1)
        cv2.circle(sample_vis, start_px, 5, (0, 255, 0), -1)  # 시작점 (초록색)
        cv2.circle(sample_vis, goal_px, 5, (0, 0, 255), -1)   # 목표점 (빨간색)
        cv2.imshow("Samples", sample_vis)
        cv2.imwrite(os.path.join(os.path.expanduser("~/ply_files/prmRviz2/image"), "samples.png"), sample_vis)
        cv2.waitKey(1)

        # 4. PRM 그래프 구성 (KNN=30)
        kdtree = KDTree(samples)
        G = nx.Graph()
        conn_vis = sample_vis.copy()

        for i, p1 in enumerate(samples):
            _, idxs = kdtree.query(p1, k=30)
            for j in idxs[1:]:
                p2 = samples[j]
                line = np.linspace(p1, p2, num=50, dtype=int)
                if np.all([
                    0 <= x < self.width and 0 <= y < self.height and safe_zone[y, x]
                    for x, y in line
                ]):
                    G.add_edge(i, j, weight=np.linalg.norm(np.subtract(p1, p2)))
                    # 연결 선 시각화
                    cv2.line(conn_vis, p1, p2, (200, 200, 0), 1)

        cv2.imshow("Connections", conn_vis)
        cv2.imwrite(os.path.join(os.path.expanduser("~/ply_files/prmRviz2/image"), "connections.png"), conn_vis)
        cv2.waitKey(1)

        try:
            path_idx = nx.shortest_path(G, source=s_idx, target=g_idx, weight="weight")
            self.get_logger().info(f"경로 탐색 성공! 경로 길이: {len(path_idx)}개 노드")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            self.get_logger().error("경로 탐색 실패")
            self.run_once = True
            return

        # 5. 최종 경로 처리 & 시각화
        path_px    = [samples[i] for i in path_idx]
        path_world = [self.map_to_world(p) for p in path_px]

        # Rviz Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        marker.color.r, marker.color.a = 1.0, 1.0
        marker.id = 0
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in path_world]
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Rviz에 PRM 경로({len(marker.points)}점) 발행")

        # OpenCV 시각화
        path_vis = conn_vis.copy()
        for i, p in enumerate(path_px):
            if i == 0:  # 시작점
                cv2.circle(path_vis, p, 5, (0, 255, 0), -1)
            elif i == len(path_px) - 1:  # 목표점
                cv2.circle(path_vis, p, 5, (0, 0, 255), -1)
            else:  # 경로상의 점들
                cv2.circle(path_vis, p, 2, (255, 0, 255), -1)
        cv2.polylines(path_vis, [np.array(path_px, dtype=np.int32)], False, (0, 0, 255), 3)
        cv2.imshow("PRM Path", path_vis)
        cv2.imwrite(os.path.join(os.path.expanduser("~/ply_files/prmRviz2/image"), "prm_path.png"), path_vis)

        # PRM 경로를 CSV로 저장
        csv_path = os.path.join(os.path.expanduser("~/ply_files/prmRviz2/image"), "prm_path.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            for wx, wy in path_world:
                writer.writerow([wx, wy])

        self.get_logger().info(f"경로가 CSV 파일로 저장됨: {csv_path}")

        # ESC 입력 시 노드 종료
        if cv2.waitKey(0) == 27:
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        self.run_once = True


def main(args=None):
    rclpy.init(args=args)
    node = PRMVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()