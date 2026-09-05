#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os

import numpy as np


LEFT_LANE_CHANGE_DISABLED_LINK_IDXS = {
    'A2256W000418',
    'A2256W000431',
    'A2256W000435',
    'A2256W000423'
}
RIGHT_LANE_CHANGE_DISABLED_LINK_IDXS = {
    'A2256W000409',
    'A2256W000423',
    'A2256W000422',
    "A2256W000408"
}
LEFT_LANE_CHANGE_DST_OVERRIDES = {
    'A2256W000319': 'A2256W000318',
    'A2256W000849': 'A2256W000054'
}
RIGHT_LANE_CHANGE_DST_OVERRIDES = {
    'A2256W000318': 'A2256W000319',
    'A2256W000054': 'A2256W000849'
}
LINK_TYPE_OVERRIDES = {
    'A2256W000054': '4'
}
RELATED_SIGNAL_OVERRIDES = {
    'A2256W000182': 'left_unprotected'
}


class MoraiMapPlanner:
    def __init__(
        self,
        link_path,
        origin_utm_x,
        origin_utm_y,
        node_path=None,
        use_utm_offset=True,
        heading_filter_deg=90.0
    ):
        self.link_path = link_path
        self.node_path = node_path
        self.origin_utm_x = origin_utm_x
        self.origin_utm_y = origin_utm_y
        self.use_utm_offset = use_utm_offset
        self.heading_filter_deg = heading_filter_deg
        self.heading_filter_rad = math.radians(float(heading_filter_deg))

        self.links = []
        self.nodes = []
        self.link_by_idx = {}
        self.links_by_from_node = {}
        self.links_by_to_node = {}
        self.lane_neighbor_idxs_by_link_idx = {}
        self.nearby_link_idx_cache = {}
        self.node_by_idx = {}

        self.segment_start_xy = np.empty((0, 2), dtype=np.float64)
        self.segment_vec_xy = np.empty((0, 2), dtype=np.float64)
        self.segment_len_sq = np.empty((0,), dtype=np.float64)
        self.segment_heading = np.empty((0,), dtype=np.float64)
        self.segment_refs = []
        self.segment_indices_by_link_idx = {}

    def load(self, build_nearest_lookup=True):
        self.load_link_set()
        self.load_node_set()
        self.build_link_lookup()

        if build_nearest_lookup:
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

    def read_positive_float(self, value, default):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return float(default)

        if not math.isfinite(value) or value <= 0.0:
            return float(default)

        return value

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

            link_idx = link.get('idx')
            can_move_left_lane = bool(
                link.get('can_move_left_lane', False)
            )
            left_lane_change_dst_link_idx = link.get(
                'left_lane_change_dst_link_idx'
            )
            can_move_right_lane = bool(
                link.get('can_move_right_lane', False)
            )
            right_lane_change_dst_link_idx = link.get(
                'right_lane_change_dst_link_idx'
            )
            width_start = self.read_positive_float(
                link.get('width_start'),
                3.5
            )
            width_end = self.read_positive_float(
                link.get('width_end'),
                width_start
            )

            if str(link_idx) in LEFT_LANE_CHANGE_DISABLED_LINK_IDXS:
                can_move_left_lane = False
                left_lane_change_dst_link_idx = None

            if str(link_idx) in RIGHT_LANE_CHANGE_DISABLED_LINK_IDXS:
                can_move_right_lane = False
                right_lane_change_dst_link_idx = None

            if str(link_idx) in LEFT_LANE_CHANGE_DST_OVERRIDES:
                can_move_left_lane = True
                left_lane_change_dst_link_idx = (
                    LEFT_LANE_CHANGE_DST_OVERRIDES[str(link_idx)]
                )

            if str(link_idx) in RIGHT_LANE_CHANGE_DST_OVERRIDES:
                can_move_right_lane = True
                right_lane_change_dst_link_idx = (
                    RIGHT_LANE_CHANGE_DST_OVERRIDES[str(link_idx)]
                )

            self.links.append({
                'idx': link_idx,
                'from_node_idx': link.get('from_node_idx'),
                'to_node_idx': link.get('to_node_idx'),
                'points': points,
                'max_speed': link.get('max_speed'),
                'link_length': link.get('link_length'),
                'width_start': width_start,
                'width_end': width_end,
                'lane_mark_left': list(link.get('lane_mark_left') or []),
                'lane_mark_right': list(link.get('lane_mark_right') or []),
                'can_move_left_lane': can_move_left_lane,
                'can_move_right_lane': can_move_right_lane,
                'left_lane_change_dst_link_idx': left_lane_change_dst_link_idx,
                'right_lane_change_dst_link_idx': right_lane_change_dst_link_idx,
                'road_id': link.get('road_id'),
                'ego_lane': link.get('ego_lane'),
                'link_type': LINK_TYPE_OVERRIDES.get(
                    str(link_idx),
                    link.get('link_type')
                ),
                'related_signal': RELATED_SIGNAL_OVERRIDES.get(
                    str(link_idx),
                    link.get('related_signal')
                )
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

    def build_link_lookup(self):
        self.link_by_idx = {}
        self.links_by_from_node = {}
        self.links_by_to_node = {}
        self.lane_neighbor_idxs_by_link_idx = {}
        self.nearby_link_idx_cache = {}

        for link in self.links:
            link_idx = link.get('idx')

            if link_idx is None:
                continue

            link_idx = str(link_idx)
            self.link_by_idx[link_idx] = link

            from_node_idx = link.get('from_node_idx')
            if from_node_idx is not None:
                self.links_by_from_node.setdefault(
                    str(from_node_idx),
                    []
                ).append(link_idx)

            to_node_idx = link.get('to_node_idx')
            if to_node_idx is not None:
                self.links_by_to_node.setdefault(
                    str(to_node_idx),
                    []
                ).append(link_idx)

        for link_idx, link in self.link_by_idx.items():
            for key in (
                'left_lane_change_dst_link_idx',
                'right_lane_change_dst_link_idx'
            ):
                dst_link_idx = link.get(key)

                if dst_link_idx is None:
                    continue

                dst_link_idx = str(dst_link_idx)
                if dst_link_idx not in self.link_by_idx:
                    continue

                self.lane_neighbor_idxs_by_link_idx.setdefault(
                    link_idx,
                    set()
                ).add(dst_link_idx)
                self.lane_neighbor_idxs_by_link_idx.setdefault(
                    dst_link_idx,
                    set()
                ).add(link_idx)

    def build_nearest_lookup(self, candidate_link_idxs=None):
        starts = []
        vecs = []
        len_sq = []
        headings = []
        refs = []
        segment_indices_by_link_idx = {}
        candidate_link_idx_set = (
            None
            if candidate_link_idxs is None
            else {str(link_idx) for link_idx in candidate_link_idxs}
        )

        for link_i, link in enumerate(self.links):
            link_idx = link.get('idx')

            if (
                candidate_link_idx_set is not None
                and str(link_idx) not in candidate_link_idx_set
            ):
                continue

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

                if link_idx is not None:
                    segment_indices_by_link_idx.setdefault(
                        str(link_idx),
                        []
                    ).append(len(refs) - 1)

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
        self.segment_indices_by_link_idx = {
            link_idx: np.asarray(indices, dtype=np.int64)
            for link_idx, indices in segment_indices_by_link_idx.items()
        }

    def get_lane_change_link_closure(
        self,
        base_link_idxs,
        max_lane_change_count=None
    ):
        link_idxs = []
        link_idx_set = set()
        pending_links = []

        if max_lane_change_count is not None:
            max_lane_change_count = max(
                0,
                int(max_lane_change_count)
            )

        for link_idx in base_link_idxs:
            link_idx = str(link_idx)

            if (
                link_idx not in self.link_by_idx
                or link_idx in link_idx_set
            ):
                continue

            link_idx_set.add(link_idx)
            link_idxs.append(link_idx)
            pending_links.append((link_idx, 0))

        pending_i = 0

        while pending_i < len(pending_links):
            link_idx, lane_change_count = pending_links[pending_i]
            pending_i += 1

            if (
                max_lane_change_count is not None
                and lane_change_count >= max_lane_change_count
            ):
                continue

            link = self.link_by_idx[link_idx]

            for side in ('left', 'right'):
                if not bool(link.get('can_move_%s_lane' % side, False)):
                    continue

                dst_link_idx = link.get(
                    '%s_lane_change_dst_link_idx' % side
                )

                if dst_link_idx is None:
                    continue

                dst_link_idx = str(dst_link_idx)

                if (
                    dst_link_idx not in self.link_by_idx
                    or dst_link_idx in link_idx_set
                ):
                    continue

                link_idx_set.add(dst_link_idx)
                link_idxs.append(dst_link_idx)
                pending_links.append((
                    dst_link_idx,
                    lane_change_count + 1
                ))

        return link_idxs

    def get_direct_neighbor_link_idxs(self, link_idx):
        link_idx = str(link_idx)
        link = self.link_by_idx.get(link_idx)

        if link is None:
            return []

        neighbor_link_idxs = set(
            self.lane_neighbor_idxs_by_link_idx.get(link_idx, set())
        )

        for node_key in ('from_node_idx', 'to_node_idx'):
            node_idx = link.get(node_key)

            if node_idx is None:
                continue

            node_idx = str(node_idx)
            neighbor_link_idxs.update(
                self.links_by_from_node.get(node_idx, [])
            )
            neighbor_link_idxs.update(
                self.links_by_to_node.get(node_idx, [])
            )

        neighbor_link_idxs.discard(link_idx)
        return sorted(neighbor_link_idxs)

    def get_nearby_link_idxs(self, center_link_idx, max_hops=2):
        if center_link_idx is None:
            return []

        center_link_idx = str(center_link_idx)
        max_hops = max(0, int(max_hops))
        cache_key = (center_link_idx, max_hops)

        cached = self.nearby_link_idx_cache.get(cache_key)
        if cached is not None:
            return list(cached)

        center_link = self.link_by_idx.get(center_link_idx)

        if center_link is None:
            return []

        previous_link_idxs = set()
        from_node_idx = center_link.get('from_node_idx')

        if from_node_idx is not None:
            previous_link_idxs.update(
                self.links_by_to_node.get(str(from_node_idx), [])
            )

        previous_link_idxs.discard(center_link_idx)
        nearby_link_idxs = [center_link_idx]
        visited = {center_link_idx}
        frontier = [center_link_idx]

        for _ in range(max_hops):
            next_frontier = []

            for link_idx in frontier:
                for neighbor_link_idx in self.get_direct_neighbor_link_idxs(
                    link_idx
                ):
                    if (
                        neighbor_link_idx in visited
                        or neighbor_link_idx in previous_link_idxs
                    ):
                        continue

                    visited.add(neighbor_link_idx)
                    nearby_link_idxs.append(neighbor_link_idx)
                    next_frontier.append(neighbor_link_idx)

            if not next_frontier:
                break

            frontier = next_frontier

        self.nearby_link_idx_cache[cache_key] = tuple(nearby_link_idxs)
        return nearby_link_idxs

    def get_segment_indices_for_links(self, link_idxs):
        segment_indices = []

        for link_idx in link_idxs:
            indices = self.segment_indices_by_link_idx.get(str(link_idx))

            if indices is not None and indices.size > 0:
                segment_indices.append(indices)

        if not segment_indices:
            return np.empty((0,), dtype=np.int64)

        return np.concatenate(segment_indices)

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

    def find_nearest_link_and_segment(
        self,
        utm_x,
        utm_y,
        yaw=None,
        candidate_link_idxs=None
    ):
        if self.segment_start_xy.shape[0] == 0:
            return None

        segment_indices = None

        if candidate_link_idxs is not None:
            segment_indices = self.get_segment_indices_for_links(
                candidate_link_idxs
            )

            if segment_indices.size == 0:
                return None

            segment_start_xy = self.segment_start_xy[segment_indices]
            segment_vec_xy = self.segment_vec_xy[segment_indices]
            segment_len_sq = self.segment_len_sq[segment_indices]
            segment_heading = self.segment_heading[segment_indices]
        else:
            segment_start_xy = self.segment_start_xy
            segment_vec_xy = self.segment_vec_xy
            segment_len_sq = self.segment_len_sq
            segment_heading = self.segment_heading

        rel_x = utm_x - segment_start_xy[:, 0]
        rel_y = utm_y - segment_start_xy[:, 1]

        dot = (
            rel_x * segment_vec_xy[:, 0]
            + rel_y * segment_vec_xy[:, 1]
        )
        ratios = np.clip(dot / segment_len_sq, 0.0, 1.0)

        proj_x = segment_start_xy[:, 0] + ratios * segment_vec_xy[:, 0]
        proj_y = segment_start_xy[:, 1] + ratios * segment_vec_xy[:, 1]

        dx = proj_x - utm_x
        dy = proj_y - utm_y

        dist_sq = dx * dx + dy * dy
        heading_error = None
        used_heading_filter = False

        if yaw is not None:
            heading_diff = np.arctan2(
                np.sin(segment_heading - yaw),
                np.cos(segment_heading - yaw)
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

        nearest_search_i = int(np.argmin(search_score))
        nearest_flat_i = nearest_search_i

        if segment_indices is not None:
            nearest_flat_i = int(segment_indices[nearest_search_i])

        if yaw is not None and heading_abs is not None:
            heading_error = float(heading_abs[nearest_search_i])

        link_i, segment_i = self.segment_refs[nearest_flat_i]

        nearest_link = self.links[link_i]
        segment_ratio = float(ratios[nearest_search_i])
        nearest_point = self.interpolate_point(
            nearest_link,
            segment_i,
            segment_ratio
        )
        nearest_distance = math.sqrt(float(dist_sq[nearest_search_i]))

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

    def find_nearest_link_and_segment_near_link(
        self,
        utm_x,
        utm_y,
        center_link_idx,
        yaw=None,
        max_hops=2
    ):
        nearby_link_idxs = self.get_nearby_link_idxs(
            center_link_idx,
            max_hops=max_hops
        )

        return self.find_nearest_link_and_segment(
            utm_x,
            utm_y,
            yaw=yaw,
            candidate_link_idxs=nearby_link_idxs
        )
