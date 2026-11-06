#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np


class VirtualLaneBoundaryGenerator:
    def __init__(
        self,
        lane_width_m,
        connected_point_tolerance_m,
        target_link_idxs=None,
        target_link_idx_groups=None,
        right_boundary_link_idxs=None
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
        self.right_boundary_link_idxs = {
            str(link_idx)
            for link_idx in (right_boundary_link_idxs or [])
        }
        self.lane_width_m = self.read_positive_float(lane_width_m)
        self.connected_point_tolerance_m = self.read_positive_float(
            connected_point_tolerance_m
        )

    def read_positive_float(self, value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError('Expected a positive finite float')

        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('Expected a positive finite float')

        return value

    def make_boundary(self, link, link_by_idx=None):
        link_idx = '' if link is None else str(link.get('idx', ''))
        target_link_idx_group = self.get_target_link_idx_group(link_idx)
        is_target_link = bool(target_link_idx_group)
        points = []
        boundary_link_idxs = []

        if is_target_link:
            if link_by_idx is None:
                points = self.make_link_boundary(link)
                boundary_link_idxs = [link_idx] if points else []
            else:
                points, boundary_link_idxs = (
                    self.make_connected_boundary(
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

    def make_connected_boundary(self, link_by_idx, target_link_idxs):
        """대상 링크들의 가상 경계점을 설정된 순서대로 모은다."""
        connected_points = []
        boundary_link_idxs = []

        for link_idx in target_link_idxs:
            target_link = link_by_idx.get(link_idx)

            if target_link is None:
                continue

            boundary_points = self.make_link_boundary(target_link)

            if not boundary_points:
                continue

            if (
                connected_points
                and connected_points[-1]['boundary_id'] == boundary_points[0]['boundary_id']
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
        tolerance_sq = self.connected_point_tolerance_m ** 2
        return dx * dx + dy * dy <= tolerance_sq

    def make_link_boundary(self, link):
        """링크 진행 방향 기준으로 차로 폭의 절반만큼 경계를 이동한다."""
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
        boundary_id = 0  # 왼쪽 경계
        if str(link.get('idx', '')) in self.right_boundary_link_idxs:
            offset_distance_m *= -1.0
            boundary_id = 1  # 오른쪽 경계
        offsets = left_normals * offset_distance_m
        boundary_rviz_xy = rviz_xy + offsets
        boundary_utm_xy = utm_xy + offsets

        return [
            {
                'rviz_x': float(boundary_rviz_xy[point_i, 0]),
                'rviz_y': float(boundary_rviz_xy[point_i, 1]),
                'utm_x': float(boundary_utm_xy[point_i, 0]),
                'utm_y': float(boundary_utm_xy[point_i, 1]),
                'z': float(points[point_i].get('morai_z', 0.0)),
                'boundary_id': boundary_id
            }
            for point_i in range(len(points))
        ]
