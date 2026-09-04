#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np


DEFAULT_LANE_WIDTH_M = 3.5
CONNECTED_POINT_TOLERANCE_M = 1e-3


class VirtualLaneBoundaryGenerator:
    def __init__(
        self,
        target_link_idxs=None,
        lane_width_m=DEFAULT_LANE_WIDTH_M,
        target_link_idx_groups=None
    ):
        if target_link_idx_groups is None:
            target_link_idx_groups = (target_link_idxs or [],)

        self.target_link_idx_groups = tuple(
            tuple(dict.fromkeys(
                str(link_idx)
                for link_idx in group
            ))
            for group in target_link_idx_groups
            if group
        )
        self.target_link_idxs = {
            link_idx
            for group in self.target_link_idx_groups
            for link_idx in group
        }
        self.target_link_idx_group_by_link_idx = {
            link_idx: group
            for group in self.target_link_idx_groups
            for link_idx in group
        }
        self.lane_width_m = self.read_positive_float(
            lane_width_m,
            DEFAULT_LANE_WIDTH_M
        )

    def read_positive_float(self, value, default):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return float(default)

        if not math.isfinite(value) or value <= 0.0:
            return float(default)

        return value

    def make_boundary(self, link, link_by_idx=None):
        link_idx = '' if link is None else str(link.get('idx', ''))
        target_link_idx_group = self.get_target_link_idx_group(link_idx)
        is_target_link = bool(target_link_idx_group)
        points = []
        boundary_link_idxs = []

        if is_target_link:
            if link_by_idx is None:
                points = self.make_left_boundary(link)
                boundary_link_idxs = [link_idx] if points else []
            else:
                points, boundary_link_idxs = (
                    self.make_connected_left_boundary(
                        link_by_idx,
                        target_link_idx_group
                    )
                )

        return {
            'link_idx': link_idx,
            'active': is_target_link,
            'boundary_link_idxs': boundary_link_idxs,
            'points': points
        }

    def get_target_link_idx_group(self, link_idx):
        return self.target_link_idx_group_by_link_idx.get(str(link_idx), ())

    def make_connected_left_boundary(self, link_by_idx, target_link_idxs):
        """대상 링크들의 좌측 경계를 설정된 순서대로 하나의 선으로 잇는다."""
        connected_points = []
        boundary_link_idxs = []

        for link_idx in target_link_idxs:
            target_link = link_by_idx.get(link_idx)

            if target_link is None:
                continue

            boundary_points = self.make_left_boundary(target_link)

            if not boundary_points:
                continue

            if (
                connected_points
                and self.points_are_connected(
                    connected_points[-1],
                    boundary_points[0]
                )
            ):
                boundary_points = boundary_points[1:]

            connected_points.extend(boundary_points)
            boundary_link_idxs.append(link_idx)

        return connected_points, boundary_link_idxs

    def points_are_connected(self, first, second):
        dx = float(first['utm_x']) - float(second['utm_x'])
        dy = float(first['utm_y']) - float(second['utm_y'])
        tolerance_sq = CONNECTED_POINT_TOLERANCE_M ** 2
        return dx * dx + dy * dy <= tolerance_sq

    def make_left_boundary(self, link):
        """링크 중심선에서 차로 폭의 절반만큼 왼쪽으로 이동한 선을 만든다."""
        points = link.get('points', [])

        if len(points) < 2:
            return []

        rviz_xy = np.asarray([
            [float(point['rviz_x']), float(point['rviz_y'])]
            for point in points
        ], dtype=np.float64)
        utm_xy = np.asarray([
            [float(point['utm_x']), float(point['utm_y'])]
            for point in points
        ], dtype=np.float64)
        segment_vectors = rviz_xy[1:] - rviz_xy[:-1]
        segment_lengths = np.hypot(
            segment_vectors[:, 0],
            segment_vectors[:, 1]
        )
        total_length = float(sum(segment_lengths))

        if total_length <= 1e-9:
            return []

        tangents = np.empty_like(rviz_xy)
        tangents[0] = segment_vectors[0]
        tangents[-1] = segment_vectors[-1]

        if len(points) > 2:
            tangents[1:-1] = rviz_xy[2:] - rviz_xy[:-2]

        tangent_lengths = np.hypot(tangents[:, 0], tangents[:, 1])

        for point_i in np.flatnonzero(tangent_lengths <= 1e-9):
            replacement = None

            if point_i > 0 and segment_lengths[point_i - 1] > 1e-9:
                replacement = segment_vectors[point_i - 1]
            elif (
                point_i < len(segment_vectors)
                and segment_lengths[point_i] > 1e-9
            ):
                replacement = segment_vectors[point_i]

            if replacement is None:
                replacement = np.asarray([1.0, 0.0], dtype=np.float64)

            tangents[point_i] = replacement

        tangent_lengths = np.hypot(tangents[:, 0], tangents[:, 1])
        tangents /= tangent_lengths[:, None]
        left_normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))

        offset_distance_m = 0.5 * self.lane_width_m
        offsets = left_normals * offset_distance_m
        left_rviz_xy = rviz_xy + offsets
        left_utm_xy = utm_xy + offsets

        return [
            {
                'rviz_x': float(left_rviz_xy[point_i, 0]),
                'rviz_y': float(left_rviz_xy[point_i, 1]),
                'utm_x': float(left_utm_xy[point_i, 0]),
                'utm_y': float(left_utm_xy[point_i, 1]),
                'z': float(points[point_i].get('morai_z', 0.0))
            }
            for point_i in range(len(points))
        ]
