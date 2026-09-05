#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math


POINT_EPSILON_M = 1e-6


class ReverseLinkTargetFinder:
    def __init__(
        self,
        link_by_idx,
        links_by_to_node,
        route_link_idxs=None,
        max_previous_link_hops=100
    ):
        self.link_by_idx = link_by_idx
        self.links_by_to_node = links_by_to_node
        self.max_previous_link_hops = max(
            1,
            int(max_previous_link_hops)
        )
        self.route_previous_by_link_idx = {}
        self.observed_previous_by_link_idx = {}
        self.last_current_link_idx = None

        route_link_idxs = [
            str(link_idx)
            for link_idx in (route_link_idxs or [])
        ]

        for previous_link_idx, current_link_idx in zip(
            route_link_idxs,
            route_link_idxs[1:]
        ):
            if self.is_direct_predecessor(
                previous_link_idx,
                current_link_idx
            ):
                self.route_previous_by_link_idx[
                    current_link_idx
                ] = previous_link_idx

    def is_direct_predecessor(
        self,
        previous_link_idx,
        current_link_idx
    ):
        previous_link = self.link_by_idx.get(str(previous_link_idx))
        current_link = self.link_by_idx.get(str(current_link_idx))

        if previous_link is None or current_link is None:
            return False

        previous_to_node_idx = previous_link.get('to_node_idx')
        current_from_node_idx = current_link.get('from_node_idx')

        return (
            previous_to_node_idx is not None
            and current_from_node_idx is not None
            and str(previous_to_node_idx) == str(current_from_node_idx)
        )

    def observe_current_link(self, current_link_idx):
        current_link_idx = str(current_link_idx)

        if current_link_idx not in self.link_by_idx:
            self.last_current_link_idx = current_link_idx
            return

        previous_link_idx = self.last_current_link_idx

        if (
            previous_link_idx is not None
            and previous_link_idx != current_link_idx
            and self.is_direct_predecessor(
                previous_link_idx,
                current_link_idx
            )
        ):
            self.observed_previous_by_link_idx[
                current_link_idx
            ] = previous_link_idx

        self.last_current_link_idx = current_link_idx

    def find_target(
        self,
        vehicle_x,
        vehicle_y,
        current_link_idx,
        target_distance_m
    ):
        current_link_idx = str(current_link_idx)
        current_link = self.link_by_idx.get(current_link_idx)

        if current_link is None:
            return None

        current_points = current_link.get('points', [])

        if len(current_points) < 2:
            return None

        current_point_index = self.find_nearest_point_index(
            current_points,
            vehicle_x,
            vehicle_y
        )
        target_distance_m = max(0.0, float(target_distance_m))
        accumulated_distance_m = 0.0
        active_link = current_link
        active_link_idx = current_link_idx
        active_point_index = current_point_index
        forward_link = None
        visited_link_idxs = {current_link_idx}

        for _ in range(self.max_previous_link_hops + 1):
            active_points = active_link['points']

            while active_point_index > 0:
                previous_point_index = active_point_index - 1
                accumulated_distance_m += self.point_distance(
                    active_points[active_point_index],
                    active_points[previous_point_index]
                )
                active_point_index = previous_point_index

                if accumulated_distance_m >= target_distance_m:
                    return self.make_target(
                        target_link=active_link,
                        target_point_index=active_point_index,
                        forward_link=forward_link
                    )

            previous_link_idx = self.select_previous_link_idx(
                active_link_idx,
                visited_link_idxs
            )

            if previous_link_idx is None:
                return None

            previous_link = self.link_by_idx[previous_link_idx]
            previous_points = previous_link.get('points', [])

            if len(previous_points) < 2:
                visited_link_idxs.add(previous_link_idx)
                continue

            previous_point_index = len(previous_points) - 1
            accumulated_distance_m += self.point_distance(
                active_points[0],
                previous_points[previous_point_index]
            )
            forward_link = active_link
            active_link = previous_link
            active_link_idx = previous_link_idx
            active_point_index = previous_point_index
            visited_link_idxs.add(previous_link_idx)

            if accumulated_distance_m >= target_distance_m:
                return self.make_target(
                    target_link=active_link,
                    target_point_index=active_point_index,
                    forward_link=forward_link
                )

        return None

    def find_nearest_point_index(self, points, vehicle_x, vehicle_y):
        nearest_index = 0
        nearest_distance_sq = float('inf')

        for point_index, point in enumerate(points):
            point_x, point_y = self.point_xy(point)
            dx = point_x - vehicle_x
            dy = point_y - vehicle_y
            distance_sq = dx * dx + dy * dy

            if distance_sq < nearest_distance_sq:
                nearest_distance_sq = distance_sq
                nearest_index = point_index

        return nearest_index

    def select_previous_link_idx(
        self,
        current_link_idx,
        visited_link_idxs
    ):
        current_link_idx = str(current_link_idx)
        current_link = self.link_by_idx.get(current_link_idx)

        if current_link is None:
            return None

        from_node_idx = current_link.get('from_node_idx')

        if from_node_idx is None:
            return None

        candidates = [
            str(link_idx)
            for link_idx in self.links_by_to_node.get(
                str(from_node_idx),
                []
            )
            if (
                str(link_idx) != current_link_idx
                and str(link_idx) not in visited_link_idxs
                and str(link_idx) in self.link_by_idx
                and len(
                    self.link_by_idx[str(link_idx)].get('points', [])
                ) >= 2
            )
        ]

        if not candidates:
            return None

        observed_previous_link_idx = (
            self.observed_previous_by_link_idx.get(current_link_idx)
        )

        if observed_previous_link_idx in candidates:
            return observed_previous_link_idx

        route_previous_link_idx = self.route_previous_by_link_idx.get(
            current_link_idx
        )

        if route_previous_link_idx in candidates:
            return route_previous_link_idx

        if len(candidates) == 1:
            return candidates[0]

        current_heading = self.link_start_heading(current_link)

        return min(
            candidates,
            key=lambda link_idx: self.heading_difference(
                self.link_end_heading(self.link_by_idx[link_idx]),
                current_heading
            )
        )

    def make_target(
        self,
        target_link,
        target_point_index,
        forward_link
    ):
        target_point = target_link['points'][target_point_index]
        next_point = self.find_next_distinct_point(
            target_link,
            target_point_index,
            forward_link
        )

        if next_point is None:
            return None

        target_x, target_y = self.point_xy(target_point)
        next_x, next_y = self.point_xy(next_point)
        target_yaw = math.atan2(
            next_y - target_y,
            next_x - target_x
        )

        return target_x, target_y, target_yaw

    def find_next_distinct_point(
        self,
        target_link,
        target_point_index,
        forward_link
    ):
        target_points = target_link['points']
        target_point = target_points[target_point_index]

        for next_point in target_points[target_point_index + 1:]:
            if self.point_distance(
                target_point,
                next_point
            ) > POINT_EPSILON_M:
                return next_point

        if forward_link is not None:
            for next_point in forward_link.get('points', []):
                if self.point_distance(
                    target_point,
                    next_point
                ) > POINT_EPSILON_M:
                    return next_point

        return None

    def link_start_heading(self, link):
        points = link['points']
        return self.points_heading(points[0], points[1])

    def link_end_heading(self, link):
        points = link['points']
        return self.points_heading(points[-2], points[-1])

    def points_heading(self, point_a, point_b):
        point_a_x, point_a_y = self.point_xy(point_a)
        point_b_x, point_b_y = self.point_xy(point_b)
        return math.atan2(
            point_b_y - point_a_y,
            point_b_x - point_a_x
        )

    def heading_difference(self, heading_a, heading_b):
        return abs(math.atan2(
            math.sin(heading_a - heading_b),
            math.cos(heading_a - heading_b)
        ))

    def point_distance(self, point_a, point_b):
        point_a_x, point_a_y = self.point_xy(point_a)
        point_b_x, point_b_y = self.point_xy(point_b)
        return math.hypot(
            point_b_x - point_a_x,
            point_b_y - point_a_y
        )

    def point_xy(self, point):
        if isinstance(point, dict):
            return float(point['utm_x']), float(point['utm_y'])

        return float(point[0]), float(point[1])
