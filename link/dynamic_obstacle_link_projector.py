# -*- coding: utf-8 -*-

import math
from collections import deque

import numpy as np


DEFAULT_MAX_OBSTACLE_DISTANCE_M = 50.0
DEFAULT_MAX_HEADING_ERROR_DEG = 45.0
DEFAULT_REVERSE_HEADING_TOLERANCE_DEG = 30.0
DEFAULT_MAX_LANE_CHANGE_COUNT = 2

MOTION_MOVING = 2


class DynamicObstacleLinkProjector:
    """진행 방향이 유효한 이동 객체를 도달 가능한 MORAI 링크에 투영한다."""

    def __init__(
        self,
        planner,
        route_link_idxs=None,
        allowed_link_idxs=None,
        max_lane_change_count=DEFAULT_MAX_LANE_CHANGE_COUNT,
        max_obstacle_distance_m=DEFAULT_MAX_OBSTACLE_DISTANCE_M,
        max_heading_error_deg=DEFAULT_MAX_HEADING_ERROR_DEG,
        reverse_heading_tolerance_deg=(
            DEFAULT_REVERSE_HEADING_TOLERANCE_DEG
        ),
    ):
        self.planner = planner
        self.route_link_idxs = [
            str(link_idx)
            for link_idx in (route_link_idxs or [])
            if str(link_idx) in self.planner.link_by_idx
        ]
        self.route_link_idx_set = set(self.route_link_idxs)
        self.allowed_link_idx_set = {
            str(link_idx)
            for link_idx in (
                allowed_link_idxs
                if allowed_link_idxs is not None
                else self.planner.link_by_idx.keys()
            )
            if str(link_idx) in self.planner.link_by_idx
        }
        self.next_route_link_idx_by_idx = {}

        for current_link_idx, next_link_idx in zip(
            self.route_link_idxs,
            self.route_link_idxs[1:],
        ):
            self.next_route_link_idx_by_idx.setdefault(
                current_link_idx,
                next_link_idx,
            )

        self.max_obstacle_distance_m = self._positive_float(
            max_obstacle_distance_m,
            DEFAULT_MAX_OBSTACLE_DISTANCE_M,
        )
        self.max_heading_error_rad = math.radians(
            self._bounded_float(
                max_heading_error_deg,
                DEFAULT_MAX_HEADING_ERROR_DEG,
                0.0,
                180.0,
            )
        )
        self.reverse_heading_tolerance_rad = math.radians(
            self._bounded_float(
                reverse_heading_tolerance_deg,
                DEFAULT_REVERSE_HEADING_TOLERANCE_DEG,
                0.0,
                180.0,
            )
        )
        self.max_lane_change_count = self._bounded_int(
            max_lane_change_count,
            DEFAULT_MAX_LANE_CHANGE_COUNT,
            0,
            DEFAULT_MAX_LANE_CHANGE_COUNT,
        )
        self._candidate_cache = {}

    def project_obstacles(
        self,
        obstacles,
        current_pose,
        current_link_idx,
    ):
        """ROS 입출력 없이 장애물별 링크 투영 결과를 반환한다."""
        if current_pose is None or current_link_idx is None:
            return []

        try:
            current_pose = (
                float(current_pose[0]),
                float(current_pose[1]),
            )
        except (IndexError, TypeError, ValueError):
            return []

        if not all(math.isfinite(value) for value in current_pose):
            return []

        current_link_idx = str(current_link_idx)

        candidate_link_idxs = self.build_candidate_link_idxs(
            current_link_idx
        )

        if not candidate_link_idxs:
            return []

        results = []

        for obstacle in obstacles or []:
            result = self.project_obstacle(
                obstacle,
                current_pose,
                candidate_link_idxs,
            )

            if result is not None:
                results.append(result)

        results.sort(
            key=lambda result: (
                float(result['ego_distance_m']),
                int(result['track_id']),
            )
        )
        return results

    def project_obstacle(
        self,
        obstacle,
        current_pose,
        candidate_link_idxs,
    ):
        try:
            track_id = int(obstacle.track_id)
            if int(obstacle.motion_state) != MOTION_MOVING:
                return None

            if not bool(obstacle.heading_valid):
                return None

            obstacle_x = float(obstacle.position_utm.x)
            obstacle_y = float(obstacle.position_utm.y)
            obstacle_heading = float(obstacle.heading_utm)
        except (AttributeError, TypeError, ValueError):
            return None

        if not all(math.isfinite(value) for value in (
            obstacle_x,
            obstacle_y,
            obstacle_heading,
        )):
            return None

        ego_distance_m = math.hypot(
            obstacle_x - current_pose[0],
            obstacle_y - current_pose[1],
        )

        if ego_distance_m > self.max_obstacle_distance_m:
            return None

        projection = self.find_nearest_projection(
            obstacle_x,
            obstacle_y,
            obstacle_heading,
            candidate_link_idxs,
        )

        if projection is None:
            return None

        return {
            'track_id': track_id,
            'nearest_link_idx': str(
                projection['link'].get('idx', '')
            ),
            'projected_position_utm': {
                'x': float(projection['point']['utm_x']),
                'y': float(projection['point']['utm_y']),
                'z': 0.0,
            },
            'ego_distance_m': ego_distance_m,
            'signed_link_distance_m': projection['signed_distance'],
            'signed_heading_error_rad': projection[
                'signed_heading_error'
            ],
        }

    def build_candidate_link_idxs(self, current_link_idx):
        current_link_idx = str(current_link_idx)
        cache_key = (
            current_link_idx,
            self.max_lane_change_count,
        )

        if cache_key in self._candidate_cache:
            return self._candidate_cache[cache_key]

        if (
            current_link_idx not in self.planner.link_by_idx
            or current_link_idx not in self.allowed_link_idx_set
        ):
            return ()

        candidate_link_idxs = []
        candidate_link_idx_set = set()
        best_lane_change_count_by_link_idx = {}
        pending = deque([(current_link_idx, 0)])

        while pending:
            link_idx, lane_change_count = pending.popleft()
            previous_count = best_lane_change_count_by_link_idx.get(
                link_idx
            )

            if (
                previous_count is not None
                and previous_count <= lane_change_count
            ):
                continue

            best_lane_change_count_by_link_idx[
                link_idx
            ] = lane_change_count

            if link_idx not in candidate_link_idx_set:
                candidate_link_idx_set.add(link_idx)
                candidate_link_idxs.append(link_idx)

            link = self.planner.link_by_idx[link_idx]

            for next_link_idx in self.find_forward_link_idxs(link):
                pending.append((next_link_idx, lane_change_count))

            if lane_change_count >= self.max_lane_change_count:
                continue

            for side in ('left', 'right'):
                if not bool(link.get('can_move_%s_lane' % side, False)):
                    continue

                lane_change_link_idx = link.get(
                    '%s_lane_change_dst_link_idx' % side
                )

                if lane_change_link_idx is None:
                    continue

                lane_change_link_idx = str(lane_change_link_idx)

                if lane_change_link_idx not in self.allowed_link_idx_set:
                    continue

                pending.append((
                    lane_change_link_idx,
                    lane_change_count + 1,
                ))

        result = tuple(candidate_link_idxs)
        self._candidate_cache[cache_key] = result
        return result

    def find_forward_link_idxs(self, link):
        link_idx = str(link.get('idx'))
        route_next_link_idx = self.next_route_link_idx_by_idx.get(
            link_idx
        )

        if self.links_are_forward_connected(link_idx, route_next_link_idx):
            return (route_next_link_idx,)

        to_node_idx = link.get('to_node_idx')

        if to_node_idx is None:
            return ()

        forward_link_idxs = [
            str(candidate_link_idx)
            for candidate_link_idx
            in self.planner.links_by_from_node.get(str(to_node_idx), [])
            if (
                str(candidate_link_idx) != link_idx
                and str(candidate_link_idx) in self.allowed_link_idx_set
            )
        ]
        route_forward_link_idxs = [
            candidate_link_idx
            for candidate_link_idx in forward_link_idxs
            if candidate_link_idx in self.route_link_idx_set
        ]

        if len(route_forward_link_idxs) == 1:
            return (route_forward_link_idxs[0],)

        if len(route_forward_link_idxs) > 1:
            return ()

        if len(forward_link_idxs) == 1:
            return (forward_link_idxs[0],)

        return ()

    def links_are_forward_connected(self, current_link_idx, next_link_idx):
        if next_link_idx is None:
            return False

        current_link = self.planner.link_by_idx.get(str(current_link_idx))
        next_link = self.planner.link_by_idx.get(str(next_link_idx))

        if current_link is None or next_link is None:
            return False

        current_to_node_idx = current_link.get('to_node_idx')
        next_from_node_idx = next_link.get('from_node_idx')

        return (
            current_to_node_idx is not None
            and next_from_node_idx is not None
            and str(current_to_node_idx) == str(next_from_node_idx)
            and str(next_link_idx) in self.allowed_link_idx_set
        )

    def find_nearest_projection(
        self,
        obstacle_x,
        obstacle_y,
        obstacle_heading,
        candidate_link_idxs,
    ):
        segment_indices = self.planner.get_segment_indices_for_links(
            candidate_link_idxs
        )

        if segment_indices.size == 0:
            return None

        segment_start_xy = self.planner.segment_start_xy[segment_indices]
        segment_vec_xy = self.planner.segment_vec_xy[segment_indices]
        segment_len_sq = self.planner.segment_len_sq[segment_indices]
        segment_heading = self.planner.segment_heading[segment_indices]

        rel_x = obstacle_x - segment_start_xy[:, 0]
        rel_y = obstacle_y - segment_start_xy[:, 1]
        ratios = np.clip(
            (
                rel_x * segment_vec_xy[:, 0]
                + rel_y * segment_vec_xy[:, 1]
            ) / segment_len_sq,
            0.0,
            1.0,
        )
        projected_x = (
            segment_start_xy[:, 0]
            + ratios * segment_vec_xy[:, 0]
        )
        projected_y = (
            segment_start_xy[:, 1]
            + ratios * segment_vec_xy[:, 1]
        )
        dx = projected_x - obstacle_x
        dy = projected_y - obstacle_y
        distance_sq = dx * dx + dy * dy

        # 먼저 기하학적으로 가장 가까운 도달 가능 링크를 확정한다. 해당 링크의
        # 방향이 맞지 않으면, 방향만 일치하는 더 먼 링크를 다시 찾지 않고
        # 장애물을 제외한다.
        nearest_search_index = int(np.argmin(distance_sq))
        nearest_heading = float(segment_heading[nearest_search_index])
        signed_heading_error = math.atan2(
            math.sin(nearest_heading - obstacle_heading),
            math.cos(nearest_heading - obstacle_heading),
        )
        reverse_heading_error = abs(math.atan2(
            math.sin(nearest_heading + math.pi - obstacle_heading),
            math.cos(nearest_heading + math.pi - obstacle_heading),
        ))

        if (
            abs(signed_heading_error) > self.max_heading_error_rad
            or reverse_heading_error
            <= self.reverse_heading_tolerance_rad
        ):
            return None

        nearest_flat_index = int(segment_indices[nearest_search_index])
        link_i, segment_i = self.planner.segment_refs[
            nearest_flat_index
        ]
        nearest_link = self.planner.links[link_i]
        nearest_segment_vec = segment_vec_xy[nearest_search_index]
        obstacle_offset_x = (
            obstacle_x - projected_x[nearest_search_index]
        )
        obstacle_offset_y = (
            obstacle_y - projected_y[nearest_search_index]
        )
        signed_distance = (
            nearest_segment_vec[0] * obstacle_offset_y
            - nearest_segment_vec[1] * obstacle_offset_x
        ) / math.sqrt(float(segment_len_sq[nearest_search_index]))
        nearest_point = self.planner.interpolate_point(
            nearest_link,
            segment_i,
            float(ratios[nearest_search_index]),
        )

        return {
            'link': nearest_link,
            'point': nearest_point,
            'distance': math.sqrt(
                float(distance_sq[nearest_search_index])
            ),
            'signed_distance': float(signed_distance),
            'signed_heading_error': signed_heading_error,
        }

    @staticmethod
    def _positive_float(value, default):
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = float(default)

        if not math.isfinite(value) or value <= 0.0:
            return float(default)

        return value

    @staticmethod
    def _bounded_float(value, default, minimum, maximum):
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = float(default)

        if not math.isfinite(value) or not minimum <= value <= maximum:
            return float(default)

        return value

    @staticmethod
    def _bounded_int(value, default, minimum, maximum):
        try:
            value = int(value)
        except (TypeError, ValueError):
            value = int(default)

        if not minimum <= value <= maximum:
            return int(default)

        return value
