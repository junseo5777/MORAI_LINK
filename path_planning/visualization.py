#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import threading

import numpy as np
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from macaron_8.msg import CurrentLink

class MoraiGlobalPathVisualizer:
    def __init__(self):
        self.frame_id = 'map'
        self.origin_utm_x = 302595.0
        self.origin_utm_y = 4124145.0
        self.use_utm_offset = True        # UTM 원점 보정 사용
        self.ego_marker_default_z = 1.0   # z 추정 실패 시 기본 높이
        self.ego_marker_z_offset = 0.0    # ego 마커 z 보정값
        self.ego_z_grid_cell_size = 10.0  # z 탐색 그리드 크기(m)
        self.current_link_marker_z_offset = 0.25

        self.current_pose_topic = '/current_pose'
        self.map_data_topic = '/morai_map_data'
        self.route_data_topic = '/morai_route_data'
        self.current_link_topic = '/current_link_data'

        self.route_marker_topic = '/global_path_marker'
        self.map_marker_topic = '/morai_link_markers'
        self.dijkstra_marker_topic = '/dijkstra_route_markers'
        self.ego_marker_topic = '/ego_vehicle_marker'
        self.current_link_marker_topic = '/current_link_marker'

        self.route_marker_pub = rospy.Publisher(
            self.route_marker_topic,
            Marker,
            queue_size=10,
            latch=True
        )
        self.map_marker_pub = rospy.Publisher(
            self.map_marker_topic,
            MarkerArray,
            queue_size=10,
            latch=True
        )
        self.dijkstra_marker_pub = rospy.Publisher(
            self.dijkstra_marker_topic,
            MarkerArray,
            queue_size=10,
            latch=True
        )
        self.ego_marker_pub = rospy.Publisher(
            self.ego_marker_topic,
            Marker,
            queue_size=10
        )
        self.current_link_marker_pub = rospy.Publisher(
            self.current_link_marker_topic,
            Marker,
            queue_size=10
        )

        self.link_by_idx = {}
        self.last_current_link_idx = None
        self.ego_segment_start_xy = np.empty((0, 2), dtype=np.float64)
        self.ego_segment_vec_xy = np.empty((0, 2), dtype=np.float64)
        self.ego_segment_len_sq = np.empty((0,), dtype=np.float64)
        self.ego_segment_start_z = np.empty((0,), dtype=np.float64)
        self.ego_segment_delta_z = np.empty((0,), dtype=np.float64)
        self.ego_z_grid = {}
        self.ego_z_lock = threading.RLock()

        self.map_data_sub = rospy.Subscriber(
            self.map_data_topic,
            String,
            self.map_data_callback,
            queue_size=1
        )
        self.route_data_sub = rospy.Subscriber(
            self.route_data_topic,
            String,
            self.route_data_callback,
            queue_size=1
        )
        self.current_pose_sub = rospy.Subscriber(
            self.current_pose_topic,
            Point,
            self.current_pose_callback,
            queue_size=10
        )
        self.current_link_sub = rospy.Subscriber(
            self.current_link_topic,
            CurrentLink,
            self.current_link_callback,
            queue_size=10
        )

        rospy.loginfo(
            'Waiting for map data from %s and route data from %s',
            self.map_data_topic,
            self.route_data_topic
        )

    def current_pose_callback(self, msg):
        self.publish_ego_vehicle_marker(
            float(msg.x),
            float(msg.y),
            float(msg.z)
        )

    def map_data_callback(self, msg):
        payload = self.loads(msg.data, 'map data')
        if payload is None:
            return

        links = payload.get('links', [])
        nodes = payload.get('nodes', [])

        self.link_by_idx = {
            str(link.get('idx')): link
            for link in links
            if link.get('idx') is not None
        }
        self.build_ego_z_lookup(links)
        self.publish_map_markers(
            links,
            nodes
        )

        if self.last_current_link_idx is not None:
            self.publish_current_link_marker(self.last_current_link_idx)

    def route_data_callback(self, msg):
        payload = self.loads(msg.data, 'route data')
        if payload is None:
            return

        self.publish_route_marker(payload.get('path', []))
        self.publish_dijkstra_route_markers(
            payload.get('route_links', []),
            payload.get('start_point'),
            payload.get('goal_point')
        )

    def current_link_callback(self, msg):
        link_idx = str(msg.link_idx)
        self.last_current_link_idx = link_idx
        self.publish_current_link_marker(link_idx)

    def loads(self, data, data_name):
        try:
            payload = json.loads(data)
        except ValueError as exc:
            rospy.logerr('Failed to parse %s: %s', data_name, exc)
            return None

        if not isinstance(payload, dict):
            rospy.logerr('Unsupported %s payload type', data_name)
            return None

        return payload

    def make_point(self, x, y, z=0.0):
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p

    def build_ego_z_lookup(self, links):
        starts = []
        vecs = []
        len_sq = []
        start_z = []
        delta_z = []
        grid = {}

        for link in links:
            points = link.get('points', [])

            for point_i in range(len(points) - 1):
                p0 = points[point_i]
                p1 = points[point_i + 1]

                x0 = float(p0.get('rviz_x', 0.0))
                y0 = float(p0.get('rviz_y', 0.0))
                z0 = float(p0.get('z', 0.0))
                x1 = float(p1.get('rviz_x', 0.0))
                y1 = float(p1.get('rviz_y', 0.0))
                z1 = float(p1.get('z', 0.0))

                vx = x1 - x0
                vy = y1 - y0
                segment_len_sq = vx * vx + vy * vy

                if segment_len_sq <= 1e-12:
                    continue

                starts.append((x0, y0))
                vecs.append((vx, vy))
                len_sq.append(segment_len_sq)
                start_z.append(z0)
                delta_z.append(z1 - z0)

                segment_i = len(starts) - 1
                self.add_segment_to_ego_z_grid(
                    grid,
                    segment_i,
                    x0,
                    y0,
                    x1,
                    y1
                )

        if starts:
            next_start_xy = np.asarray(starts, dtype=np.float64)
            next_vec_xy = np.asarray(vecs, dtype=np.float64)
            next_len_sq = np.asarray(len_sq, dtype=np.float64)
            next_start_z = np.asarray(start_z, dtype=np.float64)
            next_delta_z = np.asarray(delta_z, dtype=np.float64)
            next_grid = grid
        else:
            next_start_xy = np.empty((0, 2), dtype=np.float64)
            next_vec_xy = np.empty((0, 2), dtype=np.float64)
            next_len_sq = np.empty((0,), dtype=np.float64)
            next_start_z = np.empty((0,), dtype=np.float64)
            next_delta_z = np.empty((0,), dtype=np.float64)
            next_grid = {}

        with self.ego_z_lock:
            self.ego_segment_start_xy = next_start_xy
            self.ego_segment_vec_xy = next_vec_xy
            self.ego_segment_len_sq = next_len_sq
            self.ego_segment_start_z = next_start_z
            self.ego_segment_delta_z = next_delta_z
            self.ego_z_grid = next_grid

        rospy.loginfo(
            'Built ego z lookup segments: %d, grid cells: %d, cell size: %.1f m',
            next_len_sq.shape[0],
            len(next_grid),
            self.ego_z_grid_cell_size
        )

    def add_segment_to_ego_z_grid(self, grid, segment_i, x0, y0, x1, y1):
        if self.ego_z_grid_cell_size <= 0.0:
            return

        min_cell_x, min_cell_y = self.ego_z_grid_key(min(x0, x1), min(y0, y1))
        max_cell_x, max_cell_y = self.ego_z_grid_key(max(x0, x1), max(y0, y1))

        for cell_x in range(min_cell_x, max_cell_x + 1):
            for cell_y in range(min_cell_y, max_cell_y + 1):
                grid.setdefault((cell_x, cell_y), []).append(segment_i)

    def ego_z_grid_key(self, rviz_x, rviz_y):
        return (
            int(math.floor(rviz_x / self.ego_z_grid_cell_size)),
            int(math.floor(rviz_y / self.ego_z_grid_cell_size))
        )

    def publish_map_markers(self, links, nodes):
        marker_array = MarkerArray()
        stamp = rospy.Time.now()

        marker_id = 0

        for link in links:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'morai_links'
            marker.id = marker_id
            marker_id += 1

            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.15

            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 0.5

            for p in link.get('points', []):
                marker.points.append(
                    self.make_point(
                        p.get('rviz_x', 0.0),
                        p.get('rviz_y', 0.0),
                        p.get('z', 0.0)
                    )
                )

            marker_array.markers.append(marker)

        for node in nodes:
            marker = self.make_map_node_marker(marker_id, node, stamp)
            marker_id += 1
            marker_array.markers.append(marker)

        self.map_marker_pub.publish(marker_array)
        rospy.loginfo(
            'Published RViz map markers: links=%d, nodes=%d',
            len(links),
            len(nodes)
        )

    def make_map_node_marker(self, marker_id, node, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'morai_stop_nodes' if node.get('on_stop_line') else 'morai_nodes'
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.pose.position.x = node.get('rviz_x', 0.0)
        marker.pose.position.y = node.get('rviz_y', 0.0)
        marker.pose.position.z = node.get('z', 0.0)

        marker.scale.x = 1.4
        marker.scale.y = 1.4
        marker.scale.z = 1.4

        if node.get('on_stop_line'):
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            marker.color.r = 0.1
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.9

        return marker

    def publish_route_marker(self, path):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'global_path'
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.7

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for p in path:
            marker.points.append(
                self.make_point(
                    p.get('rviz_x', 0.0),
                    p.get('rviz_y', 0.0),
                    p.get('z', 0.0)
                )
            )

        self.route_marker_pub.publish(marker)
        rospy.loginfo('Published RViz route marker: points=%d', len(path))

    def publish_dijkstra_route_markers(self, route_links, start_point, goal_point):
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        self.dijkstra_marker_pub.publish(MarkerArray(markers=[clear_marker]))

        marker_array = MarkerArray()
        stamp = rospy.Time.now()

        marker_id = 0

        for link in route_links:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'dijkstra_route_links'
            marker.id = marker_id
            marker_id += 1

            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.35

            marker.color.r = 0.0
            marker.color.g = 0.35
            marker.color.b = 1.0
            marker.color.a = 0.95

            for p in link.get('points', []):
                marker.points.append(
                    self.make_point(
                        p.get('rviz_x', 0.0),
                        p.get('rviz_y', 0.0),
                        p.get('z', 0.0)
                    )
                )

            marker_array.markers.append(marker)

        if start_point is not None:
            start_marker = self.make_route_point_marker(
                marker_id,
                'dijkstra_route_points',
                start_point,
                0.0,
                1.0,
                0.0
            )
            marker_array.markers.append(start_marker)
            marker_id += 1

        if goal_point is not None:
            goal_marker = self.make_route_point_marker(
                marker_id,
                'dijkstra_route_points',
                goal_point,
                0.8,
                0.0,
                1.0
            )
            marker_array.markers.append(goal_marker)

        self.dijkstra_marker_pub.publish(marker_array)
        rospy.loginfo(
            'Published RViz Dijkstra markers: route_links=%d',
            len(route_links)
        )

    def publish_current_link_marker(self, link_idx):
        if not link_idx:
            self.clear_current_link_marker()
            return

        if not self.link_by_idx:
            return

        link = self.link_by_idx.get(str(link_idx))

        if link is None:
            self.clear_current_link_marker()
            rospy.logwarn_throttle(
                2.0,
                'Current link %s not found in MORAI map data',
                link_idx
            )
            return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'current_link'
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.9

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for p in link.get('points', []):
            marker.points.append(
                self.make_point(
                    p.get('rviz_x', 0.0),
                    p.get('rviz_y', 0.0),
                    p.get('z', 0.0) + self.current_link_marker_z_offset
                )
            )

        self.current_link_marker_pub.publish(marker)

    def clear_current_link_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'current_link'
        marker.id = 0
        marker.action = Marker.DELETE
        self.current_link_marker_pub.publish(marker)

    def publish_ego_vehicle_marker(self, utm_x, utm_y, yaw):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'ego_vehicle'
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        rviz_x, rviz_y = self.utm_to_rviz_xy(utm_x, utm_y)
        marker.pose.position.x = rviz_x
        marker.pose.position.y = rviz_y
        marker.pose.position.z = self.estimate_ego_z(rviz_x, rviz_y)

        half_yaw = yaw * 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(half_yaw)
        marker.pose.orientation.w = math.cos(half_yaw)

        marker.scale.x = 4.5
        marker.scale.y = 1.2
        marker.scale.z = 1.2

        marker.color.r = 1.0
        marker.color.g = 0.75
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.ego_marker_pub.publish(marker)

    def estimate_ego_z(self, rviz_x, rviz_y):
        with self.ego_z_lock:
            starts_all = self.ego_segment_start_xy
            vecs_all = self.ego_segment_vec_xy
            len_sq_all = self.ego_segment_len_sq
            start_z_all = self.ego_segment_start_z
            delta_z_all = self.ego_segment_delta_z
            grid = self.ego_z_grid

            segment_count = starts_all.shape[0]
            if (
                segment_count == 0
                or vecs_all.shape[0] != segment_count
                or len_sq_all.shape[0] != segment_count
                or start_z_all.shape[0] != segment_count
                or delta_z_all.shape[0] != segment_count
            ):
                return self.ego_marker_default_z

            candidate_indices = self.get_ego_z_candidate_indices(
                rviz_x,
                rviz_y,
                grid
            )

            if candidate_indices is None:
                starts = starts_all
                vecs = vecs_all
                len_sq = len_sq_all
                start_z = start_z_all
                delta_z = delta_z_all
            else:
                starts = starts_all[candidate_indices]
                vecs = vecs_all[candidate_indices]
                len_sq = len_sq_all[candidate_indices]
                start_z = start_z_all[candidate_indices]
                delta_z = delta_z_all[candidate_indices]

        rel_x = rviz_x - starts[:, 0]
        rel_y = rviz_y - starts[:, 1]

        dot = (
            rel_x * vecs[:, 0]
            + rel_y * vecs[:, 1]
        )
        ratios = np.clip(dot / len_sq, 0.0, 1.0)

        proj_x = (
            starts[:, 0]
            + ratios * vecs[:, 0]
        )
        proj_y = (
            starts[:, 1]
            + ratios * vecs[:, 1]
        )

        dx = proj_x - rviz_x
        dy = proj_y - rviz_y
        nearest_i = int(np.argmin(dx * dx + dy * dy))

        z = (
            start_z[nearest_i]
            + delta_z[nearest_i] * ratios[nearest_i]
        )

        return float(z + self.ego_marker_z_offset)

    def get_ego_z_candidate_indices(self, rviz_x, rviz_y, grid=None):
        if grid is None:
            grid = self.ego_z_grid

        if self.ego_z_grid_cell_size <= 0.0 or not grid:
            return None

        center_x, center_y = self.ego_z_grid_key(rviz_x, rviz_y)

        for radius in (1, 2, 3):
            candidates = set()

            for cell_x in range(center_x - radius, center_x + radius + 1):
                for cell_y in range(center_y - radius, center_y + radius + 1):
                    candidates.update(
                        grid.get((cell_x, cell_y), [])
                    )

            if candidates:
                return np.asarray(list(candidates), dtype=np.int64)

        return None

    def utm_to_rviz_xy(self, utm_x, utm_y):
        if self.use_utm_offset:
            return utm_x - self.origin_utm_x, utm_y - self.origin_utm_y

        return utm_x, utm_y

    def make_route_point_marker(self, marker_id, namespace, point, r, g, b):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.pose.position.x = point.get('rviz_x', 0.0)
        marker.pose.position.y = point.get('rviz_y', 0.0)
        marker.pose.position.z = point.get('z', 0.0)

        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        return marker


def main():
    rospy.init_node('morai_visualization')
    visualizer = MoraiGlobalPathVisualizer()
    rospy.spin()
    return visualizer


if __name__ == '__main__':
    main()