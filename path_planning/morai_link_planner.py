#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dijkstra import dijkstra_link


class MoraiMapPlanner:
    def __init__(
        self,
        link_path,
        origin_utm_x,
        origin_utm_y,
        node_path=None,
        use_utm_offset=True,
        heading_filter_deg=90.0,
        lane_change_penalty=30.0,
        lane_change_lookahead_m=20.0
    ):
        self.link_path = link_path
        self.node_path = node_path
        self.origin_utm_x = origin_utm_x
        self.origin_utm_y = origin_utm_y
        self.use_utm_offset = use_utm_offset
        self.heading_filter_deg = heading_filter_deg
        self.heading_filter_rad = math.radians(float(heading_filter_deg))
        self.lane_change_penalty = lane_change_penalty
        self.lane_change_lookahead_m = float(lane_change_lookahead_m)

        self.links = []
        self.nodes = []
        self.node_by_idx = {}

        self.segment_start_xy = np.empty((0, 2), dtype=np.float64)
        self.segment_vec_xy = np.empty((0, 2), dtype=np.float64)
        self.segment_len_sq = np.empty((0,), dtype=np.float64)
        self.segment_heading = np.empty((0,), dtype=np.float64)
        self.segment_refs = []

    @staticmethod
    def default_map_path(file_name):
        package_src = Path(__file__).resolve().parents[1]
        return str(package_src / 'path' / file_name)

    def load(self):
        self.load_link_set()
        self.load_node_set()
        self.build_nearest_lookup()

    def morai_to_utm(self, morai_x, morai_y):
        utm_x = self.origin_utm_x + morai_x
        utm_y = self.origin_utm_y + morai_y
        return utm_x, utm_y

    def utm_to_rviz_xy(self, utm_x, utm_y):
        if self.use_utm_offset:
            return utm_x - self.origin_utm_x, utm_y - self.origin_utm_y

        return utm_x, utm_y

    def morai_to_rviz_xy(self, morai_x, morai_y):
        utm_x, utm_y = self.morai_to_utm(morai_x, morai_y)
        rviz_x, rviz_y = self.utm_to_rviz_xy(utm_x, utm_y)
        return rviz_x, rviz_y, utm_x, utm_y

    def read_xyz(self, point):
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            return None

        try:
            x = float(point[0])
            y = float(point[1])
            z = float(point[2]) if len(point) > 2 else 0.0
        except (TypeError, ValueError):
            return None

        return x, y, z

    def load_json_records(self, path, data_name):
        if path == '' or not os.path.exists(path):
            return []

        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        if isinstance(data, list):
            return data

        if isinstance(data, dict):
            if data_name in data and isinstance(data[data_name], list):
                return data[data_name]

            records = []

            for key, value in data.items():
                if not isinstance(value, dict):
                    continue

                record = dict(value)
                record.setdefault('idx', key)
                records.append(record)

            if records:
                return records

        return []

    def load_link_set(self):
        self.links = []
        link_data = self.load_json_records(self.link_path, 'link_set')

        for link in link_data:
            if not isinstance(link, dict):
                continue

            points = []

            for p in link.get('points', []):
                xyz = self.read_xyz(p)

                if xyz is None:
                    continue

                morai_x, morai_y, morai_z = xyz
                rviz_x, rviz_y, utm_x, utm_y = self.morai_to_rviz_xy(
                    morai_x,
                    morai_y
                )

                points.append({
                    'rviz_x': rviz_x,
                    'rviz_y': rviz_y,
                    'utm_x': utm_x,
                    'utm_y': utm_y,
                    'morai_x': morai_x,
                    'morai_y': morai_y,
                    'morai_z': morai_z
                })

            if len(points) < 2:
                continue

            self.links.append({
                'idx': link.get('idx'),
                'from_node_idx': link.get('from_node_idx'),
                'to_node_idx': link.get('to_node_idx'),
                'points': points,
                'max_speed': link.get('max_speed'),
                'link_length': link.get('link_length'),
                'can_move_left_lane': bool(link.get('can_move_left_lane', False)),
                'can_move_right_lane': bool(link.get('can_move_right_lane', False)),
                'left_lane_change_dst_link_idx': link.get('left_lane_change_dst_link_idx'),
                'right_lane_change_dst_link_idx': link.get('right_lane_change_dst_link_idx'),
                'road_id': link.get('road_id'),
                'ego_lane': link.get('ego_lane'),
                'link_type': link.get('link_type'),
                'related_signal': link.get('related_signal')
            })

    def load_node_set(self):
        self.nodes = []
        self.node_by_idx = {}

        if self.node_path is None:
            return

        node_data = self.load_json_records(self.node_path, 'node_set')

        for node in node_data:
            if not isinstance(node, dict):
                continue

            xyz = self.read_xyz(node.get('point'))

            if xyz is None:
                continue

            morai_x, morai_y, morai_z = xyz
            rviz_x, rviz_y, utm_x, utm_y = self.morai_to_rviz_xy(
                morai_x,
                morai_y
            )

            node_info = {
                'idx': node.get('idx'),
                'node_type': node.get('node_type'),
                'on_stop_line': bool(node.get('on_stop_line', False)),
                'traffic_light_id': node.get('traffic_light_id'),
                'rviz_x': rviz_x,
                'rviz_y': rviz_y,
                'utm_x': utm_x,
                'utm_y': utm_y,
                'morai_x': morai_x,
                'morai_y': morai_y,
                'morai_z': morai_z
            }

            self.nodes.append(node_info)

            if node_info['idx'] is not None:
                self.node_by_idx[node_info['idx']] = node_info

    def get_stop_line_node_idxs(self):
        return {
            node_idx
            for node_idx, node in self.node_by_idx.items()
            if node.get('on_stop_line')
        }

    def build_nearest_lookup(self):
        starts = []
        vecs = []
        len_sq = []
        headings = []
        refs = []

        for link_i, link in enumerate(self.links):
            points = link['points']

            for segment_i in range(len(points) - 1):
                p0 = points[segment_i]
                p1 = points[segment_i + 1]

                vx = p1['utm_x'] - p0['utm_x']
                vy = p1['utm_y'] - p0['utm_y']
                segment_len_sq = vx * vx + vy * vy

                if segment_len_sq <= 1e-12:
                    continue

                starts.append((p0['utm_x'], p0['utm_y']))
                vecs.append((vx, vy))
                len_sq.append(segment_len_sq)
                headings.append(math.atan2(vy, vx))
                refs.append((link_i, segment_i))

        if starts:
            self.segment_start_xy = np.asarray(starts, dtype=np.float64)
            self.segment_vec_xy = np.asarray(vecs, dtype=np.float64)
            self.segment_len_sq = np.asarray(len_sq, dtype=np.float64)
            self.segment_heading = np.asarray(headings, dtype=np.float64)
        else:
            self.segment_start_xy = np.empty((0, 2), dtype=np.float64)
            self.segment_vec_xy = np.empty((0, 2), dtype=np.float64)
            self.segment_len_sq = np.empty((0,), dtype=np.float64)
            self.segment_heading = np.empty((0,), dtype=np.float64)

        self.segment_refs = refs

    def interpolate_point(self, link, segment_index, ratio):
        ratio = max(0.0, min(1.0, float(ratio)))

        p0 = link['points'][segment_index]
        p1 = link['points'][segment_index + 1]

        return {
            'rviz_x': p0['rviz_x'] + (p1['rviz_x'] - p0['rviz_x']) * ratio,
            'rviz_y': p0['rviz_y'] + (p1['rviz_y'] - p0['rviz_y']) * ratio,
            'utm_x': p0['utm_x'] + (p1['utm_x'] - p0['utm_x']) * ratio,
            'utm_y': p0['utm_y'] + (p1['utm_y'] - p0['utm_y']) * ratio,
            'morai_x': p0['morai_x'] + (p1['morai_x'] - p0['morai_x']) * ratio,
            'morai_y': p0['morai_y'] + (p1['morai_y'] - p0['morai_y']) * ratio,
            'morai_z': p0['morai_z'] + (p1['morai_z'] - p0['morai_z']) * ratio
        }

    def find_nearest_link_and_segment(self, utm_x, utm_y, yaw=None):
        if self.segment_start_xy.shape[0] == 0:
            return None

        rel_x = utm_x - self.segment_start_xy[:, 0]
        rel_y = utm_y - self.segment_start_xy[:, 1]

        dot = (
            rel_x * self.segment_vec_xy[:, 0]
            + rel_y * self.segment_vec_xy[:, 1]
        )
        ratios = np.clip(dot / self.segment_len_sq, 0.0, 1.0)

        proj_x = self.segment_start_xy[:, 0] + ratios * self.segment_vec_xy[:, 0]
        proj_y = self.segment_start_xy[:, 1] + ratios * self.segment_vec_xy[:, 1]

        dx = proj_x - utm_x
        dy = proj_y - utm_y

        dist_sq = dx * dx + dy * dy
        heading_error = None
        used_heading_filter = False

        if yaw is not None:
            heading_diff = np.arctan2(
                np.sin(self.segment_heading - yaw),
                np.cos(self.segment_heading - yaw)
            )
            heading_abs = np.abs(heading_diff)
            valid = heading_abs <= self.heading_filter_rad

            if np.any(valid):
                search_score = np.where(valid, dist_sq, np.inf)
                used_heading_filter = True
            else:
                search_score = dist_sq
        else:
            heading_abs = None
            search_score = dist_sq

        nearest_flat_i = int(np.argmin(search_score))

        if yaw is not None and heading_abs is not None:
            heading_error = float(heading_abs[nearest_flat_i])

        link_i, segment_i = self.segment_refs[nearest_flat_i]

        nearest_link = self.links[link_i]
        segment_ratio = float(ratios[nearest_flat_i])
        nearest_point = self.interpolate_point(
            nearest_link,
            segment_i,
            segment_ratio
        )
        nearest_distance = math.sqrt(float(dist_sq[nearest_flat_i]))

        return {
            'link': nearest_link,
            'point': nearest_point,
            'segment_index': segment_i,
            'segment_ratio': segment_ratio,
            'progress': segment_i + segment_ratio,
            'distance': nearest_distance,
            'heading_error': heading_error,
            'used_heading_filter': used_heading_filter
        }

    def find_nearest_route_link_and_segment(self, utm_x, utm_y, route_links):
        best_link = None
        best_segment_i = None
        best_ratio = 0.0
        best_dist_sq = None

        for link in route_links:
            points = link.get('points', [])

            for segment_i in range(len(points) - 1):
                p0 = points[segment_i]
                p1 = points[segment_i + 1]

                vx = p1['utm_x'] - p0['utm_x']
                vy = p1['utm_y'] - p0['utm_y']
                segment_len_sq = vx * vx + vy * vy

                if segment_len_sq <= 1e-12:
                    continue

                rel_x = utm_x - p0['utm_x']
                rel_y = utm_y - p0['utm_y']
                ratio = (rel_x * vx + rel_y * vy) / segment_len_sq
                ratio = max(0.0, min(1.0, ratio))

                proj_x = p0['utm_x'] + vx * ratio
                proj_y = p0['utm_y'] + vy * ratio
                dx = proj_x - utm_x
                dy = proj_y - utm_y
                dist_sq = dx * dx + dy * dy

                if best_dist_sq is None or dist_sq < best_dist_sq:
                    best_link = link
                    best_segment_i = segment_i
                    best_ratio = ratio
                    best_dist_sq = dist_sq

        if best_link is None:
            return None

        nearest_point = self.interpolate_point(
            best_link,
            best_segment_i,
            best_ratio
        )

        return {
            'link': best_link,
            'point': nearest_point,
            'segment_index': best_segment_i,
            'segment_ratio': best_ratio,
            'progress': best_segment_i + best_ratio,
            'distance': math.sqrt(best_dist_sq)
        }

    def plan_path(self, start_utm_x, start_utm_y, goal_utm_x, goal_utm_y, start_yaw=None):
        start_info = self.find_nearest_link_and_segment(
            float(start_utm_x),
            float(start_utm_y),
            yaw=start_yaw
        )

        goal_info = self.find_nearest_link_and_segment(
            float(goal_utm_x),
            float(goal_utm_y)
        )

        if start_info is None or goal_info is None:
            return {
                'success': False,
                'reason': 'nearest_failed',
                'path_array': None,
                'route_links': [],
                'start_info': start_info,
                'goal_info': goal_info,
                'dijkstra_result': None
            }

        path_array, route_links, dijkstra_result = self.make_global_path(
            start_info,
            goal_info
        )

        return {
            'success': path_array is not None and len(path_array) > 0,
            'reason': None if path_array is not None else 'dijkstra_failed',
            'path_array': path_array,
            'route_links': route_links,
            'start_info': start_info,
            'goal_info': goal_info,
            'dijkstra_result': dijkstra_result
        }

    def make_global_path(self, start_info, goal_info):
        start_link = start_info['link']
        goal_link = goal_info['link']

        if start_link.get('idx') == goal_link.get('idx'):
            if start_info['progress'] <= goal_info['progress']:
                route_points = self.make_same_link_points(
                    start_info,
                    goal_info
                )

                path_array = self.points_to_numpy(route_points)
                return path_array, [start_link], None

        dijkstra_result = dijkstra_link(
            self.links,
            start_link.get('idx'),
            goal_link.get('idx'),
            lane_change_penalty=self.lane_change_penalty,
            stop_line_node_idxs=self.get_stop_line_node_idxs(),
            lane_change_lookahead_m=self.lane_change_lookahead_m
        )

        if not dijkstra_result['success']:
            return None, [], dijkstra_result

        route_links = dijkstra_result['link_path']

        route_points = self.make_link_route_points(
            start_info,
            route_links,
            goal_info,
            dijkstra_result.get('edge_path', [])
        )

        path_array = self.points_to_numpy(route_points)

        return path_array, route_links, dijkstra_result

    def make_same_link_points(self, start_info, goal_info):
        link = start_info['link']
        start_segment_index = start_info['segment_index']
        goal_segment_index = goal_info['segment_index']

        route_points = [start_info['point']]

        route_points.extend(
            link['points'][start_segment_index + 1:goal_segment_index + 1]
        )

        route_points.append(goal_info['point'])

        return self.remove_duplicate_points(route_points)

    def make_link_route_points(self, start_info, route_links, goal_info, edge_path):
        if len(route_links) == 0:
            return []

        if len(route_links) == 1:
            if start_info['progress'] <= goal_info['progress']:
                return self.make_same_link_points(start_info, goal_info)

            return []

        start_link = start_info['link']
        goal_link = goal_info['link']

        current_start_index = start_info['segment_index']
        goal_segment_index = goal_info['segment_index']

        route_points = [start_info['point']]

        for route_i, edge in enumerate(edge_path):
            current_link = route_links[route_i]
            next_link = route_links[route_i + 1]

            if edge.get('edge_type') == 'forward':
                route_points.extend(current_link['points'][current_start_index + 1:])
                current_start_index = 0
                continue

            change_index, next_start_index = self.find_lane_change_indices(
                current_link,
                next_link,
                current_start_index,
                goal_link,
                goal_segment_index
            )
            route_points.extend(current_link['points'][current_start_index + 1:change_index + 1])
            route_points.extend(
                self.make_lane_change_curve(
                    current_link,
                    change_index,
                    next_link,
                    next_start_index
                )
            )
            current_start_index = next_start_index

        if route_links[-1].get('idx') == goal_link.get('idx'):
            route_points.extend(goal_link['points'][current_start_index + 1:goal_segment_index + 1])

        route_points.append(goal_info['point'])

        return self.remove_duplicate_points(route_points)

    def find_lane_change_indices(
        self,
        current_link,
        next_link,
        current_start_index,
        goal_link,
        goal_segment_index
    ):
        current_points = current_link['points']
        next_points = next_link['points']

        min_change_index = min(len(current_points) - 1, current_start_index + 1)
        preferred_index = int((len(current_points) - 1) * 0.1)
        change_index = max(min_change_index, preferred_index)
        change_index = min(change_index, len(current_points) - 1)

        change_point = current_points[change_index]
        nearest_next_index = self.find_nearest_point_index(next_link, change_point)
        next_start_index = self.find_forward_index_by_distance(
            next_link,
            nearest_next_index,
            self.lane_change_lookahead_m
        )

        if next_link.get('idx') == goal_link.get('idx'):
            next_start_index = min(next_start_index, goal_segment_index)
            target_point = next_points[next_start_index]
            change_index = self.find_nearest_point_index(current_link, target_point)
            change_index = max(min_change_index, change_index)
            change_index = min(change_index, len(current_points) - 1)

        return change_index, next_start_index

    def find_nearest_point_index(self, link, point):
        points = link['points']
        xy = np.asarray(
            [(p['utm_x'], p['utm_y']) for p in points],
            dtype=np.float64
        )
        dx = xy[:, 0] - point['utm_x']
        dy = xy[:, 1] - point['utm_y']
        return int(np.argmin(dx * dx + dy * dy))

    def find_forward_index_by_distance(self, link, start_index, forward_distance):
        points = link['points']
        target_index = max(0, min(int(start_index), len(points) - 1))
        remaining_distance = max(0.0, float(forward_distance))

        while target_index < len(points) - 1 and remaining_distance > 0.0:
            p0 = points[target_index]
            p1 = points[target_index + 1]
            segment_length = math.hypot(
                p1['utm_x'] - p0['utm_x'],
                p1['utm_y'] - p0['utm_y']
            )
            remaining_distance -= segment_length
            target_index += 1

        return target_index

    def make_lane_change_curve(self, current_link, current_index, next_link, next_index):
        p0 = current_link['points'][current_index]
        p3 = next_link['points'][next_index]

        heading0 = self.get_point_heading(current_link, current_index)
        heading1 = self.get_point_heading(next_link, next_index)
        distance = math.hypot(p3['utm_x'] - p0['utm_x'], p3['utm_y'] - p0['utm_y'])
        control_length = min(max(distance * 0.5, 2.0), 15.0)

        p1 = {
            'utm_x': p0['utm_x'] + math.cos(heading0) * control_length,
            'utm_y': p0['utm_y'] + math.sin(heading0) * control_length
        }
        p2 = {
            'utm_x': p3['utm_x'] - math.cos(heading1) * control_length,
            'utm_y': p3['utm_y'] - math.sin(heading1) * control_length
        }

        curve = []

        for step in range(1, 13):
            t = step / 13.0
            one_minus_t = 1.0 - t
            b0 = one_minus_t * one_minus_t * one_minus_t
            b1 = 3.0 * one_minus_t * one_minus_t * t
            b2 = 3.0 * one_minus_t * t * t
            b3 = t * t * t

            utm_x = (
                b0 * p0['utm_x']
                + b1 * p1['utm_x']
                + b2 * p2['utm_x']
                + b3 * p3['utm_x']
            )
            utm_y = (
                b0 * p0['utm_y']
                + b1 * p1['utm_y']
                + b2 * p2['utm_y']
                + b3 * p3['utm_y']
            )
            rviz_x, rviz_y = self.utm_to_rviz_xy(utm_x, utm_y)

            curve.append({
                'rviz_x': rviz_x,
                'rviz_y': rviz_y,
                'utm_x': utm_x,
                'utm_y': utm_y,
                'morai_x': utm_x - self.origin_utm_x,
                'morai_y': utm_y - self.origin_utm_y,
                'morai_z': p0['morai_z'] + (p3['morai_z'] - p0['morai_z']) * t
            })

        return curve

    def get_point_heading(self, link, index):
        points = link['points']

        if index < len(points) - 1:
            p0 = points[index]
            p1 = points[index + 1]
        else:
            p0 = points[index - 1]
            p1 = points[index]

        return math.atan2(p1['utm_y'] - p0['utm_y'], p1['utm_x'] - p0['utm_x'])

    def remove_duplicate_points(self, points, eps=1e-6):
        if len(points) == 0:
            return []

        if len(points) == 1:
            return list(points)

        xy = np.asarray(
            [(p['utm_x'], p['utm_y']) for p in points],
            dtype=np.float64
        )
        delta_xy = np.diff(xy, axis=0)
        keep_mask = np.concatenate((
            np.asarray([True]),
            np.linalg.norm(delta_xy, axis=1) > eps
        ))

        return [
            point
            for point, keep in zip(points, keep_mask)
            if keep
        ]

    def points_to_numpy(self, points):
        return np.asarray(
            [
                [p['utm_x'], p['utm_y'], p['morai_z']]
                for p in points
            ],
            dtype=np.float64
        )