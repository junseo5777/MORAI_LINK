# -*- coding: utf-8 -*-

import math

START_AHEAD_OF_VEHICLE_M = 10.0 # 몇 m 앞에서 차선변경 후보를 만들지
CANDIDATE_PATH_LENGTH_M = 20.0 # 후보의 길이
CANDIDATE_LENGTH_TOLERANCE_M = 0.05 # 후보 길이 허용 오차
CANDIDATE_CURVE_SAMPLES = 101 # 몇개의 점으로 만들지
MAX_CANDIDATE_HEADING_DEVIATION_DEG = 45.0 # 차선변경 후보의 각도 허용 범위 (시작점 기준)


class LaneChangeCandidateGenerator:
    def __init__(
        self,
        links=None,
        nodes=None,
        route_links=None,
        start_ahead_of_vehicle_m=START_AHEAD_OF_VEHICLE_M,
        candidate_path_length_m=CANDIDATE_PATH_LENGTH_M,
        length_tolerance_m=CANDIDATE_LENGTH_TOLERANCE_M,
        curve_samples=CANDIDATE_CURVE_SAMPLES
    ):
        self.start_ahead_of_vehicle_m = max(
            0.0,
            float(start_ahead_of_vehicle_m)
        )
        self.candidate_path_length_m = float(candidate_path_length_m)
        self.length_tolerance_m = max(0.0, float(length_tolerance_m))
        self.curve_samples = max(2, int(curve_samples))
        self.link_by_idx = {}
        self.links_by_from_node = {}
        self.route_link_idxs = []
        self.route_is_closed = False

        self.update_map(links or [], nodes or [])
        self.update_route(route_links or [])

    def update_map(self, links, nodes):
        self.link_by_idx = {
            str(link.get('idx')): link
            for link in links
            if link.get('idx') is not None
        }
        self.links_by_from_node = {}

        for link in links:
            from_node_idx = link.get('from_node_idx')

            if from_node_idx is None:
                continue

            self.links_by_from_node.setdefault(
                str(from_node_idx),
                []
            ).append(link)

    def update_route(self, route_links):
        route_link_idxs = [
            str(link.get('idx') if isinstance(link, dict) else link)
            for link in route_links
            if (
                link.get('idx') if isinstance(link, dict) else link
            ) is not None
        ]

        self.route_is_closed = (
            len(route_link_idxs) > 1
            and route_link_idxs[0] == route_link_idxs[-1]
        )

        if self.route_is_closed:
            route_link_idxs = route_link_idxs[:-1]

        self.route_link_idxs = route_link_idxs

    def make_candidates(
        self,
        current_link_idx,
        current_rviz_x=None,
        current_rviz_y=None
    ):
        if not current_link_idx:
            return self.make_empty_result(None)

        current_link_idx = str(current_link_idx)
        result = self.make_empty_result(current_link_idx)

        if not self.link_by_idx or not self.route_link_idxs:
            return result

        current_link = self.link_by_idx.get(current_link_idx)

        if current_link is None:
            return result

        if current_rviz_x is None or current_rviz_y is None:
            return result

        current_progress = self.find_nearest_link_progress(
            current_link,
            float(current_rviz_x),
            float(current_rviz_y)
        )

        if current_progress is None:
            return result

        next_link = self.find_next_forward_link(current_link)

        if next_link is not None:
            result['next_link_idx'] = next_link.get('idx')

        start_info = self.find_forward_point(
            current_link,
            current_progress,
            self.start_ahead_of_vehicle_m
        )

        if start_info is None:
            return result

        start_link = start_info['link']

        for side in ('left', 'right'):
            candidate = self.make_lane_change_candidate(
                start_link,
                start_info,
                side
            )

            if candidate is not None:
                result['candidates'].append(candidate)

        return result

    def make_empty_result(self, current_link_idx):
        return {
            'current_link_idx': current_link_idx,
            'next_link_idx': None,
            'candidates': []
        }

    def find_next_forward_link(self, current_link):
        current_link_idx = str(current_link.get('idx'))
        current_to_node_idx = current_link.get('to_node_idx')

        for route_i, route_link_idx in enumerate(self.route_link_idxs):
            if route_link_idx != current_link_idx:
                continue

            next_route_i = route_i + 1

            if next_route_i >= len(self.route_link_idxs):
                if not self.route_is_closed:
                    continue

                next_route_i = 0

            next_link = self.link_by_idx.get(
                self.route_link_idxs[next_route_i]
            )

            if next_link is None:
                continue

            if (
                current_to_node_idx is not None
                and str(next_link.get('from_node_idx'))
                == str(current_to_node_idx)
            ):
                return next_link

        if current_to_node_idx is None:
            return None

        return self.find_next_connected_link(current_link)

    def find_next_connected_link(self, current_link):
        current_link_idx = str(current_link.get('idx'))
        current_to_node_idx = current_link.get('to_node_idx')

        if current_to_node_idx is None:
            return None

        forward_links = self.links_by_from_node.get(
            str(current_to_node_idx),
            []
        )
        forward_links = [
            link
            for link in forward_links
            if str(link.get('idx')) != current_link_idx
        ]

        if not forward_links:
            return None

        if len(forward_links) == 1:
            return forward_links[0]

        current_heading = self.get_link_end_heading(current_link)

        if current_heading is None:
            return forward_links[0]

        return min(
            forward_links,
            key=lambda link: self.heading_difference_abs(
                current_heading,
                self.get_link_start_heading(link)
            )
        )

    def find_forward_point(self, current_link, current_progress, distance_ahead):
        link = current_link
        progress = max(0.0, float(current_progress))
        remaining = max(0.0, float(distance_ahead))
        max_link_count = max(1, len(self.link_by_idx) + 1)

        for _ in range(max_link_count):
            geometry = self.make_link_geometry(link)

            if geometry is None:
                return None

            progress = min(progress, geometry['total_length'])
            distance_to_link_end = geometry['total_length'] - progress

            if remaining <= distance_to_link_end:
                point_info = self.get_link_point_at_progress(
                    geometry,
                    progress + remaining
                )

                if point_info is None:
                    return None

                point_info['link'] = link
                point_info['progress'] = progress + remaining
                return point_info

            remaining -= distance_to_link_end
            next_link = self.find_next_forward_link(link)

            if next_link is None or next_link is link:
                return None

            link = next_link
            progress = 0.0

        return None

    def make_lane_change_candidate(self, start_link, start_info, side):
        dst_link = self.find_lane_change_destination(start_link, side)

        if dst_link is None:
            return None

        dst_link_idx = str(dst_link.get('idx'))

        destination_result = self.make_destination_geometry(
            start_link,
            dst_link,
            side,
            start_info,
            self.candidate_path_length_m
        )

        if destination_result is None:
            return None

        destination_geometry, nearest_dst_s, source_geometry = (
            destination_result
        )

        path = self.find_lane_change_curve_by_length(
            start_info,
            destination_geometry,
            nearest_dst_s,
            self.candidate_path_length_m
        )

        if path is None:
            return None

        if not self.is_candidate_curve_valid(
            path,
            start_info,
            source_geometry,
            side
        ):
            return None

        return {
            'side': side,
            'destination_link_idx': str(dst_link_idx),
            'path': path
        }

    def find_lane_change_destination(self, source_link, side):
        if side == 'left':
            can_change = bool(source_link.get('can_move_left_lane', False))
            dst_link_idx = source_link.get('left_lane_change_dst_link_idx')
        elif side == 'right':
            can_change = bool(source_link.get('can_move_right_lane', False))
            dst_link_idx = source_link.get('right_lane_change_dst_link_idx')
        else:
            return None

        if not can_change or not dst_link_idx:
            return None

        if str(dst_link_idx) == str(source_link.get('idx')):
            return None

        return self.link_by_idx.get(str(dst_link_idx))

    def make_destination_geometry(
        self,
        source_link,
        dst_link,
        side,
        start_info,
        target_length
    ):
        source_links = []
        destination_links = []
        visited_source_link_idxs = set()
        visited_destination_link_idxs = set()
        max_link_count = max(1, len(self.link_by_idx) + 1)
        latest_result = None
        nearest_dst_s = None

        for _ in range(max_link_count):
            source_link_idx = str(source_link.get('idx'))
            dst_link_idx = str(dst_link.get('idx'))

            if (
                source_link_idx in visited_source_link_idxs
                or dst_link_idx in visited_destination_link_idxs
            ):
                break

            visited_source_link_idxs.add(source_link_idx)
            visited_destination_link_idxs.add(dst_link_idx)
            source_links.append(source_link)
            destination_links.append(dst_link)
            source_geometry = self.make_link_chain_geometry(source_links)
            geometry = self.make_link_chain_geometry(destination_links)

            if source_geometry is None or geometry is None:
                return None

            if nearest_dst_s is None:
                nearest_dst_s = self.find_nearest_geometry_progress(
                    geometry,
                    start_info['point']['rviz_x'],
                    start_info['point']['rviz_y']
                )

            if nearest_dst_s is None:
                return None

            latest_result = (geometry, nearest_dst_s, source_geometry)
            high_curve = self.make_candidate_bezier_curve(
                start_info,
                self.get_link_point_at_progress(
                    geometry,
                    geometry['total_length']
                )
            )

            if (
                high_curve is not None
                and self.calc_curve_length(high_curve)
                >= target_length - self.length_tolerance_m
            ):
                return latest_result

            next_source_link = self.find_next_forward_link(source_link)

            if next_source_link is None:
                break

            next_dst_link = self.find_lane_change_destination(
                next_source_link,
                side
            )

            if next_dst_link is None:
                break

            if not self.links_are_forward_connected(dst_link, next_dst_link):
                break

            source_link = next_source_link
            dst_link = next_dst_link

        return latest_result

    def links_are_forward_connected(self, current_link, next_link):
        current_to_node_idx = current_link.get('to_node_idx')
        next_from_node_idx = next_link.get('from_node_idx')

        return (
            current_to_node_idx is not None
            and next_from_node_idx is not None
            and str(current_to_node_idx) == str(next_from_node_idx)
        )

    def make_link_chain_geometry(self, links):
        points = []

        for link in links:
            link_points = link.get('points', [])

            if len(link_points) < 2:
                return None

            if points and self.points_are_same(points[-1], link_points[0]):
                points.extend(link_points[1:])
            else:
                points.extend(link_points)

        return self.make_link_geometry({'points': points})

    def points_are_same(self, p0, p1, eps=1e-6):
        return (
            abs(
                float(p0.get('rviz_x', 0.0))
                - float(p1.get('rviz_x', 0.0))
            ) <= eps
            and abs(
                float(p0.get('rviz_y', 0.0))
                - float(p1.get('rviz_y', 0.0))
            ) <= eps
        )

    def find_nearest_link_progress(self, link, rviz_x, rviz_y):
        geometry = self.make_link_geometry(link)

        if geometry is None:
            return None

        return self.find_nearest_geometry_progress(
            geometry,
            rviz_x,
            rviz_y
        )

    def find_nearest_geometry_progress(self, geometry, rviz_x, rviz_y):
        best_s = None
        best_dist_sq = None

        for segment_i, segment_length in enumerate(geometry['segment_lengths']):
            if segment_length <= 1e-9:
                continue

            p0 = geometry['points'][segment_i]
            p1 = geometry['points'][segment_i + 1]
            vx = float(p1.get('rviz_x', 0.0)) - float(p0.get('rviz_x', 0.0))
            vy = float(p1.get('rviz_y', 0.0)) - float(p0.get('rviz_y', 0.0))
            rel_x = rviz_x - float(p0.get('rviz_x', 0.0))
            rel_y = rviz_y - float(p0.get('rviz_y', 0.0))
            ratio = (rel_x * vx + rel_y * vy) / (segment_length * segment_length)
            ratio = max(0.0, min(1.0, ratio))
            proj_x = float(p0.get('rviz_x', 0.0)) + vx * ratio
            proj_y = float(p0.get('rviz_y', 0.0)) + vy * ratio
            dx = proj_x - rviz_x
            dy = proj_y - rviz_y
            dist_sq = dx * dx + dy * dy

            if best_dist_sq is None or dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_s = (
                    geometry['cumulative_lengths'][segment_i]
                    + segment_length * ratio
                )

        return best_s

    def find_lane_change_curve_by_length(
        self,
        start_info,
        dst_geometry,
        dst_start_s,
        target_length
    ):
        geometry = dst_geometry

        if geometry is None or target_length <= 0.0:
            return None

        low_s = max(0.0, min(float(dst_start_s), geometry['total_length']))
        high_s = geometry['total_length']
        low_curve = self.make_candidate_bezier_curve(
            start_info,
            self.get_link_point_at_progress(geometry, low_s)
        )
        high_curve = self.make_candidate_bezier_curve(
            start_info,
            self.get_link_point_at_progress(geometry, high_s)
        )

        if low_curve is None or high_curve is None:
            return None

        low_length = self.calc_curve_length(low_curve)
        high_length = self.calc_curve_length(high_curve)

        if low_length > target_length + self.length_tolerance_m:
            return None

        if high_length < target_length - self.length_tolerance_m:
            return None

        best_curve = min(
            (low_curve, high_curve),
            key=lambda curve: abs(self.calc_curve_length(curve) - target_length)
        )

        for _ in range(30):
            mid_s = 0.5 * (low_s + high_s)
            curve = self.make_candidate_bezier_curve(
                start_info,
                self.get_link_point_at_progress(geometry, mid_s)
            )

            if curve is None:
                return None

            curve_length = self.calc_curve_length(curve)

            if abs(curve_length - target_length) < abs(
                self.calc_curve_length(best_curve) - target_length
            ):
                best_curve = curve

            if abs(curve_length - target_length) <= self.length_tolerance_m:
                return curve

            if curve_length < target_length:
                low_s = mid_s
            else:
                high_s = mid_s

        if abs(self.calc_curve_length(best_curve) - target_length) <= (
            self.length_tolerance_m
        ):
            return best_curve

        return None

    def is_candidate_curve_valid(
        self,
        curve,
        start_info,
        source_geometry,
        side
    ):
        if len(curve) < 2:
            return False

        start_heading = start_info['heading']
        cos_heading = math.cos(start_heading)
        sin_heading = math.sin(start_heading)
        max_heading_deviation = math.radians(
            MAX_CANDIDATE_HEADING_DEVIATION_DEG
        )

        for p0, p1 in zip(curve, curve[1:]):
            dx = p1['rviz_x'] - p0['rviz_x']
            dy = p1['rviz_y'] - p0['rviz_y']

            if dx * cos_heading + dy * sin_heading <= 0.0:
                return False

            segment_heading = math.atan2(dy, dx)
            heading_difference = abs(math.atan2(
                math.sin(segment_heading - start_heading),
                math.cos(segment_heading - start_heading)
            ))

            if heading_difference > max_heading_deviation:
                return False

        end_point = curve[-1]
        source_progress = self.find_nearest_geometry_progress(
            source_geometry,
            end_point['rviz_x'],
            end_point['rviz_y']
        )

        if source_progress is None:
            return False

        source_info = self.get_link_point_at_progress(
            source_geometry,
            source_progress
        )

        if source_info is None:
            return False

        source_point = source_info['point']
        source_heading = source_info['heading']
        end_dx = end_point['rviz_x'] - source_point['rviz_x']
        end_dy = end_point['rviz_y'] - source_point['rviz_y']
        lateral_offset = (
            -end_dx * math.sin(source_heading)
            + end_dy * math.cos(source_heading)
        )

        if side == 'left':
            return lateral_offset > 0.0

        return lateral_offset < 0.0

    def make_link_geometry(self, link):
        points = link.get('points', [])

        if len(points) < 2:
            return None

        segment_lengths = []
        cumulative_lengths = [0.0]

        for p0, p1 in zip(points, points[1:]):
            segment_length = math.hypot(
                float(p1.get('rviz_x', 0.0)) - float(p0.get('rviz_x', 0.0)),
                float(p1.get('rviz_y', 0.0)) - float(p0.get('rviz_y', 0.0))
            )
            segment_lengths.append(segment_length)
            cumulative_lengths.append(cumulative_lengths[-1] + segment_length)

        if cumulative_lengths[-1] <= 1e-9:
            return None

        return {
            'points': points,
            'segment_lengths': segment_lengths,
            'cumulative_lengths': cumulative_lengths,
            'total_length': cumulative_lengths[-1]
        }

    def get_link_point_at_progress(self, geometry, progress):
        progress = max(0.0, min(float(progress), geometry['total_length']))
        points = geometry['points']

        for segment_i, segment_length in enumerate(geometry['segment_lengths']):
            segment_start_s = geometry['cumulative_lengths'][segment_i]
            segment_end_s = geometry['cumulative_lengths'][segment_i + 1]

            if (
                progress > segment_end_s
                and segment_i < len(geometry['segment_lengths']) - 1
            ):
                continue

            ratio = 0.0 if segment_length <= 1e-9 else (
                progress - segment_start_s
            ) / segment_length
            ratio = max(0.0, min(1.0, ratio))
            p0 = points[segment_i]
            p1 = points[segment_i + 1]
            return {
                'point': {
                    'rviz_x': self.interpolate_value(p0, p1, 'rviz_x', ratio),
                    'rviz_y': self.interpolate_value(p0, p1, 'rviz_y', ratio),
                    'z': self.interpolate_value(p0, p1, 'z', ratio)
                },
                'heading': math.atan2(
                    float(p1.get('rviz_y', 0.0)) - float(p0.get('rviz_y', 0.0)),
                    float(p1.get('rviz_x', 0.0)) - float(p0.get('rviz_x', 0.0))
                )
            }

        return None

    def interpolate_value(self, p0, p1, key, ratio):
        value0 = self.get_point_value(p0, key)
        value1 = self.get_point_value(p1, key)
        return value0 + (value1 - value0) * ratio

    def get_point_value(self, point, key):
        if key == 'z':
            return float(point.get('z', point.get('morai_z', 0.0)))

        return float(point.get(key, 0.0))

    def make_candidate_bezier_curve(self, start_info, end_info):
        if start_info is None or end_info is None:
            return None

        p0 = start_info['point']
        p3 = end_info['point']
        heading0 = start_info['heading']
        heading1 = end_info['heading']
        chord_length = math.hypot(
            p3['rviz_x'] - p0['rviz_x'],
            p3['rviz_y'] - p0['rviz_y']
        )

        if chord_length <= 1e-9:
            return None

        control_length = min(max(chord_length * 0.5, 2.0), 15.0)
        p1_x = p0['rviz_x'] + math.cos(heading0) * control_length
        p1_y = p0['rviz_y'] + math.sin(heading0) * control_length
        p2_x = p3['rviz_x'] - math.cos(heading1) * control_length
        p2_y = p3['rviz_y'] - math.sin(heading1) * control_length
        curve = []

        for step in range(self.curve_samples):
            t = step / float(self.curve_samples - 1)
            one_minus_t = 1.0 - t
            b0 = one_minus_t ** 3
            b1 = 3.0 * one_minus_t * one_minus_t * t
            b2 = 3.0 * one_minus_t * t * t
            b3 = t ** 3
            curve.append({
                'rviz_x': (
                    b0 * p0['rviz_x'] + b1 * p1_x
                    + b2 * p2_x + b3 * p3['rviz_x']
                ),
                'rviz_y': (
                    b0 * p0['rviz_y'] + b1 * p1_y
                    + b2 * p2_y + b3 * p3['rviz_y']
                ),
                'z': p0['z'] + (p3['z'] - p0['z']) * t
            })

        return curve

    def calc_curve_length(self, curve):
        return sum(
            math.hypot(
                p1['rviz_x'] - p0['rviz_x'],
                p1['rviz_y'] - p0['rviz_y']
            )
            for p0, p1 in zip(curve, curve[1:])
        )

    def get_link_start_heading(self, link):
        geometry = self.make_link_geometry(link)
        info = None if geometry is None else self.get_link_point_at_progress(
            geometry,
            0.0
        )
        return None if info is None else info['heading']

    def get_link_end_heading(self, link):
        geometry = self.make_link_geometry(link)
        info = None if geometry is None else self.get_link_point_at_progress(
            geometry,
            geometry['total_length']
        )
        return None if info is None else info['heading']

    def heading_difference_abs(self, heading0, heading1):
        if heading1 is None:
            return float('inf')

        return abs(math.atan2(
            math.sin(heading1 - heading0),
            math.cos(heading1 - heading0)
        ))