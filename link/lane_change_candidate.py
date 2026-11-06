import math
from bisect import bisect_left


class LaneChangeCandidateGenerator:
    def __init__(
        self,
        links,
        nodes,
        start_ahead_of_vehicle_m,
        candidate_path_length_m,
        length_tolerance_m,
        point_interval_m,
        minimum_candidate_path_length_m,
        candidate_path_extension_m,
        endpoint_no_next_link_idxs,
        path_length_overrides_m,
        route_link_idxs=None,
        lane_change_extension_link_idxs=None,
        candidate_allowed_link_idxs=None
    ):
        self.start_ahead_of_vehicle_m = max(
            0.0,
            float(start_ahead_of_vehicle_m)
        )
        self.candidate_path_length_m = float(candidate_path_length_m)
        self.minimum_candidate_path_length_m = min(
            self.candidate_path_length_m,
            max(0.0, float(minimum_candidate_path_length_m))
        )
        self.length_tolerance_m = max(0.0, float(length_tolerance_m))
        self.point_interval_m = max(1e-3, float(point_interval_m))
        self.candidate_path_extension_m = max(
            0.0,
            float(candidate_path_extension_m)
        )
        self.endpoint_no_next_link_idx_set = {
            str(link_idx) for link_idx in endpoint_no_next_link_idxs
        }
        self.path_length_overrides_m = {
            str(link_idx): max(0.0, float(path_length_m))
            for link_idx, path_length_m in path_length_overrides_m.items()
        }
        self.route_link_idxs = [
            str(link_idx)
            for link_idx in (route_link_idxs or [])
        ]
        self.route_link_idx_set = set(self.route_link_idxs)
        self.next_route_link_idx_by_idx = {
            current_link_idx: next_link_idx
            for current_link_idx, next_link_idx in zip(
                self.route_link_idxs,
                self.route_link_idxs[1:]
            )
        }
        self.lane_change_extension_link_idx_set = {
            str(link_idx)
            for link_idx in (lane_change_extension_link_idxs or [])
        }
        self.candidate_allowed_link_idx_set = {
            str(link_idx) for link_idx in (candidate_allowed_link_idxs or [])
        }
        self.link_by_idx = {}
        self.links_by_from_node = {}
        self.link_geometry_cache = {}
        self.link_chain_geometry_cache = {}

        self.update_map(links or [], nodes or [])

    def update_map(self, links, nodes):
        self.link_geometry_cache = {}
        self.link_chain_geometry_cache = {}
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

    def make_candidates(
        self,
        current_link_idx,
        current_rviz_x=None,
        current_rviz_y=None,
        current_progress=None
    ):
        """현재 차로 유지와 좌우 차선 변경 후보를 최대 3개 반환한다."""
        if not current_link_idx:
            return self.make_empty_result(None)

        current_link_idx = str(current_link_idx)
        result = self.make_empty_result(current_link_idx)

        if not self.link_by_idx:
            return result

        current_link = self.link_by_idx.get(current_link_idx)

        if current_link is None:
            return result

        if current_rviz_x is None or current_rviz_y is None:
            return result

        if current_progress is None:
            current_progress = self.find_nearest_link_progress(
                current_link,
                float(current_rviz_x),
                float(current_rviz_y)
            )
        else:
            current_progress = float(current_progress)

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
        is_highway_candidate = (
            current_link_idx in self.candidate_allowed_link_idx_set
        )
        center_path_length = self.candidate_path_length_m

        if is_highway_candidate:
            center_path_length += self.candidate_path_extension_m

        center_candidate = self.make_keep_lane_candidate(
            start_link,
            start_info,
            center_path_length
        )

        if center_candidate is not None:
            result['candidates'].append(center_candidate)

        # 중앙 후보는 전 구간에서 만들고, 좌우 후보 계산은 고속도로로 제한한다.
        if not is_highway_candidate:
            return result

        # 전방 시작점 설정으로 다음 링크에서 시작하는 후보도 예외 구간에서는 제외한다.
        if (
            current_link_idx in self.endpoint_no_next_link_idx_set
            and str(start_link.get('idx')) != current_link_idx
        ):
            return result

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

    def make_keep_lane_candidate(
        self,
        start_link,
        start_info,
        target_length=None
    ):
        if target_length is None:
            target_length = self.candidate_path_length_m

        forward_path = self.collect_forward_path(
            start_link,
            start_info['progress'],
            target_length
        )

        if forward_path is None:
            return None

        control_points = forward_path['points']
        path_length = self.calc_curve_length(control_points)

        if path_length < (
            self.minimum_candidate_path_length_m
            - self.length_tolerance_m
        ):
            return None

        return {
            'side': 'center',
            'destination_link_idx': str(start_link.get('idx')),
            'endpoint_link_idx': forward_path['endpoint_link_idx'],
            'path_length_m': path_length,
            'path': control_points
        }

    def collect_forward_path(self, start_link, start_progress, distance):
        """현재 링크부터 연결 관계를 따라 전방 중심선 점을 수집한다."""
        if start_link is None or distance <= 0.0:
            return None

        points = []
        link = start_link
        progress = max(0.0, float(start_progress))
        remaining = max(0.0, float(distance))
        visited = set()
        endpoint_link_idx = None

        while link is not None:
            link_idx = str(link.get('idx'))

            if link_idx in visited:
                break

            visited.add(link_idx)
            geometry = self.make_link_geometry(link)

            if geometry is None:
                break

            progress = min(progress, geometry['total_length'])
            target_progress = min(
                geometry['total_length'],
                progress + remaining
            )
            start_point_info = self.get_link_point_at_progress(
                geometry,
                progress
            )

            if start_point_info is not None:
                points.append(start_point_info['point'])

            for point_i, point_progress in enumerate(
                geometry['cumulative_lengths']
            ):
                if (
                    point_progress > progress + 1e-6
                    and point_progress < target_progress - 1e-6
                ):
                    point = geometry['points'][point_i]
                    points.append({
                        'rviz_x': float(point.get('rviz_x', 0.0)),
                        'rviz_y': float(point.get('rviz_y', 0.0)),
                        'z': self.get_point_value(point, 'z')
                    })

            end_point_info = self.get_link_point_at_progress(
                geometry,
                target_progress
            )

            if end_point_info is not None:
                points.append(end_point_info['point'])

            endpoint_link_idx = link_idx
            consumed = max(0.0, target_progress - progress)
            remaining = max(0.0, remaining - consumed)

            if remaining <= 1e-6:
                break

            if not self.can_extend_candidate_path_from_link(link):
                break

            next_link = self.find_next_forward_link(link)

            if (
                next_link is None
                or next_link is link
                or not self.can_extend_candidate_path_from_link(next_link)
            ):
                break

            link = next_link
            progress = 0.0

        points = self.remove_consecutive_duplicate_points(points)

        if len(points) < 2 or endpoint_link_idx is None:
            return None

        return {
            'endpoint_link_idx': endpoint_link_idx,
            'points': points
        }

    def remove_consecutive_duplicate_points(self, points, epsilon=1e-6):
        unique = []

        for point in points:
            normalized = {
                'rviz_x': float(point.get('rviz_x', 0.0)),
                'rviz_y': float(point.get('rviz_y', 0.0)),
                'z': self.get_point_value(point, 'z')
            }

            if unique and math.hypot(
                normalized['rviz_x'] - unique[-1]['rviz_x'],
                normalized['rviz_y'] - unique[-1]['rviz_y']
            ) <= float(epsilon):
                continue

            unique.append(normalized)

        return unique

    def find_next_forward_link(self, current_link):
        return self.find_next_connected_link(current_link)

    def can_extend_candidate_path_from_link(self, link):
        if link is None:
            return False

        return (
            str(link.get('idx'))
            in self.lane_change_extension_link_idx_set
        )

    def find_next_route_link(self, current_link):
        current_link_idx = str(current_link.get('idx'))
        next_link_idx = self.next_route_link_idx_by_idx.get(
            current_link_idx
        )

        if next_link_idx is None:
            return None

        next_link = self.link_by_idx.get(next_link_idx)

        if (
            next_link is None
            or not self.links_are_forward_connected(
                current_link,
                next_link
            )
        ):
            return None

        return next_link

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

        next_route_link = self.find_next_route_link(current_link)

        if next_route_link is not None:
            return next_route_link

        route_forward_links = [
            link
            for link in forward_links
            if str(link.get('idx')) in self.route_link_idx_set
        ]

        if len(route_forward_links) == 1:
            return route_forward_links[0]

        if len(route_forward_links) > 1:
            return None

        allowed_forward_links = [
            link
            for link in forward_links
            if self.can_extend_candidate_path_from_link(link)
        ]

        if len(allowed_forward_links) == 1:
            return allowed_forward_links[0]

        return None

    def find_next_source_link(self, current_link):
        next_link = self.find_next_forward_link(current_link)

        if next_link is not None:
            return next_link

        current_link_idx = str(current_link.get('idx'))
        current_to_node_idx = current_link.get('to_node_idx')

        if current_to_node_idx is None:
            return None

        forward_links = [
            link
            for link in self.links_by_from_node.get(
                str(current_to_node_idx),
                []
            )
            if str(link.get('idx')) != current_link_idx
        ]

        if len(forward_links) == 1:
            return forward_links[0]

        return None

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

        target_length = self.path_length_overrides_m.get(
            str(start_link.get('idx')),
            self.candidate_path_length_m
        )
        destination_result = self.make_destination_geometry(
            start_link,
            dst_link,
            side,
            start_info,
            target_length + self.candidate_path_extension_m
        )

        if destination_result is None:
            return None

        (
            destination_geometry,
            nearest_dst_s,
            _,
            destination_links
        ) = destination_result

        path = self.find_lane_change_straight_path_by_length(
            start_info,
            destination_geometry,
            nearest_dst_s,
            target_length,
            self.minimum_candidate_path_length_m
        )

        if path is None:
            return None

        if not path:
            return None

        lane_change_endpoint_link_idx = self.find_nearest_link_idx_to_point(
            destination_links,
            path[-1]
        )

        # 이 두 출발 링크에서는 현재 구간의 직접 인접 차로에서 끝나야 한다.
        # 다음 구간에 끝나는 후보를 짧게 잘라 대체하지 않고 그대로 제외한다.
        if (
            str(start_link.get('idx')) in self.endpoint_no_next_link_idx_set
            and str(lane_change_endpoint_link_idx) != dst_link_idx
        ):
            return None

        extension_links = destination_links

        if str(start_link.get('idx')) in self.endpoint_no_next_link_idx_set:
            # 기존 예외 구간은 연장선도 직접 인접 링크를 벗어나지 않는다.
            extension_links = [dst_link]

        extension_geometry = self.make_link_chain_geometry(extension_links)
        path = self.extend_path_along_geometry(
            path,
            extension_geometry,
            self.candidate_path_extension_m
        )
        endpoint_link_idx = self.find_nearest_link_idx_to_point(
            extension_links,
            path[-1]
        )
        used_destination_links = self.links_through_endpoint(
            extension_links,
            endpoint_link_idx
        )

        return {
            'side': side,
            'destination_link_idx': dst_link_idx,
            'destination_link_idxs': [
                str(link.get('idx'))
                for link in used_destination_links
                if link.get('idx') is not None
            ],
            'endpoint_link_idx': (
                dst_link_idx
                if endpoint_link_idx is None
                else str(endpoint_link_idx)
            ),
            'path_length_m': self.calc_curve_length(path),
            'path': path
        }

    def extend_path_along_geometry(self, path, geometry, extension_length):
        """후보 끝에서 목적 차선 중심선을 따라 지정 거리만큼 연장한다."""
        if not path or geometry is None or extension_length <= 0.0:
            return path

        endpoint = path[-1]
        start_progress = self.find_nearest_geometry_progress(
            geometry,
            float(endpoint.get('rviz_x', 0.0)),
            float(endpoint.get('rviz_y', 0.0))
        )

        if start_progress is None:
            return path

        end_progress = min(
            geometry['total_length'],
            start_progress + float(extension_length)
        )
        available_length = max(0.0, end_progress - start_progress)

        if available_length <= 1e-6:
            return path

        interval_count = max(
            1,
            int(math.ceil(available_length / self.point_interval_m))
        )
        extension_points = []

        for interval_i in range(interval_count + 1):
            progress = start_progress + (
                available_length * interval_i / interval_count
            )
            point_info = self.get_link_point_at_progress(geometry, progress)

            if point_info is not None:
                extension_points.append(point_info['point'])

        return self.remove_consecutive_duplicate_points(
            list(path) + extension_points
        )

    def links_through_endpoint(self, links, endpoint_link_idx):
        """후보 종점이 포함된 링크까지만 목적 링크 목록에 남긴다."""
        if endpoint_link_idx is None:
            return list(links)

        endpoint_link_idx = str(endpoint_link_idx)
        used_links = []

        for link in links:
            used_links.append(link)

            if str(link.get('idx')) == endpoint_link_idx:
                break

        return used_links

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

            latest_result = (
                geometry,
                nearest_dst_s,
                source_geometry,
                list(destination_links)
            )
            high_length = self.calc_candidate_straight_length(
                start_info,
                self.get_link_point_at_progress(
                    geometry,
                    geometry['total_length']
                )
            )

            if (
                high_length is not None
                and high_length >= (
                    target_length - self.length_tolerance_m
                )
            ):
                return latest_result

            next_source_link = self.find_next_source_link(source_link)

            if next_source_link is None:
                break

            next_dst_link = None

            if self.can_extend_candidate_path_from_link(dst_link):
                next_dst_link = self.find_next_forward_link(dst_link)

                if not self.can_extend_candidate_path_from_link(next_dst_link):
                    next_dst_link = None

            if next_dst_link is None:
                next_dst_link = self.find_lane_change_destination(
                    next_source_link,
                    side
                )

            if (
                next_dst_link is None
                or not self.can_extend_candidate_path_from_link(next_dst_link)
                or not self.links_are_forward_connected(
                    dst_link,
                    next_dst_link
                )
            ):
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
        cache_key = []

        for link in links:
            link_idx = link.get('idx')

            if (
                link_idx is None
                or self.link_by_idx.get(str(link_idx)) is not link
            ):
                cache_key = None
            elif cache_key is not None:
                cache_key.append(str(link_idx))

        if cache_key is not None:
            cache_key = tuple(cache_key)

            if cache_key in self.link_chain_geometry_cache:
                return self.link_chain_geometry_cache[cache_key]

        for link in links:
            link_points = link.get('points', [])

            if len(link_points) < 2:
                if cache_key is not None:
                    self.link_chain_geometry_cache[cache_key] = None
                return None

            if points and self.points_are_same(points[-1], link_points[0]):
                points.extend(link_points[1:])
            else:
                points.extend(link_points)

        geometry = self.make_link_geometry({'points': points})

        if cache_key is not None:
            self.link_chain_geometry_cache[cache_key] = geometry

        return geometry

    def find_nearest_link_idx_to_point(self, links, point):
        best_link_idx = None
        best_dist_sq = None

        for link in links:
            geometry = self.make_link_geometry(link)

            if geometry is None:
                continue

            progress = self.find_nearest_geometry_progress(
                geometry,
                float(point.get('rviz_x', 0.0)),
                float(point.get('rviz_y', 0.0))
            )

            if progress is None:
                continue

            nearest_info = self.get_link_point_at_progress(
                geometry,
                progress
            )

            if nearest_info is None:
                continue

            nearest_point = nearest_info['point']
            dx = (
                float(nearest_point.get('rviz_x', 0.0))
                - float(point.get('rviz_x', 0.0))
            )
            dy = (
                float(nearest_point.get('rviz_y', 0.0))
                - float(point.get('rviz_y', 0.0))
            )
            dist_sq = dx * dx + dy * dy

            if best_dist_sq is None or dist_sq <= best_dist_sq:
                best_dist_sq = dist_sq
                best_link_idx = link.get('idx')

        return best_link_idx

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

    def get_link_progress_at_segment(
        self,
        link,
        segment_index,
        segment_ratio
    ):
        """링크 segment 위치를 링크 시작점 기준 거리[m]로 변환한다."""
        geometry = self.make_link_geometry(link)

        if geometry is None:
            return None

        try:
            segment_index = int(segment_index)
            segment_ratio = float(segment_ratio)
        except (TypeError, ValueError):
            return None

        if (
            segment_index < 0
            or segment_index >= len(geometry['segment_lengths'])
            or not math.isfinite(segment_ratio)
        ):
            return None

        segment_ratio = max(0.0, min(1.0, segment_ratio))
        return (
            geometry['cumulative_lengths'][segment_index]
            + geometry['segment_lengths'][segment_index] * segment_ratio
        )

    def find_nearest_geometry_progress(self, geometry, rviz_x, rviz_y):
        best_s = None
        best_dist_sq = None
        point_x = geometry.get('_point_x')
        point_y = geometry.get('_point_y')
        segment_dx = geometry.get('_segment_dx')
        segment_dy = geometry.get('_segment_dy')

        if (
            point_x is None
            or point_y is None
            or segment_dx is None
            or segment_dy is None
        ):
            points = geometry['points']
            point_x = [
                float(point.get('rviz_x', 0.0))
                for point in points
            ]
            point_y = [
                float(point.get('rviz_y', 0.0))
                for point in points
            ]
            segment_dx = [
                point_x[point_i + 1] - point_x[point_i]
                for point_i in range(len(points) - 1)
            ]
            segment_dy = [
                point_y[point_i + 1] - point_y[point_i]
                for point_i in range(len(points) - 1)
            ]

        for segment_i, segment_length in enumerate(geometry['segment_lengths']):
            if segment_length <= 1e-9:
                continue

            vx = segment_dx[segment_i]
            vy = segment_dy[segment_i]
            rel_x = rviz_x - point_x[segment_i]
            rel_y = rviz_y - point_y[segment_i]
            ratio = (rel_x * vx + rel_y * vy) / (segment_length * segment_length)
            ratio = max(0.0, min(1.0, ratio))
            proj_x = point_x[segment_i] + vx * ratio
            proj_y = point_y[segment_i] + vy * ratio
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

    def find_lane_change_straight_path_by_length(
        self, start_info, dst_geometry, dst_start_s, target_length,
        minimum_length=None
    ):
        """목적지 중심선과 목표 반경 원의 첫 전방 교점을 직선으로 연결한다."""
        if dst_geometry is None or target_length <= 0.0:
            return None
        geometry = dst_geometry
        start_s = max(0.0, min(float(dst_start_s), geometry['total_length']))
        start = start_info['point']
        first = self.get_link_point_at_progress(geometry, start_s)
        if self.calc_candidate_straight_length(start_info, first) > (
            target_length + self.length_tolerance_m
        ):
            return None
        # 선분과 원의 교점을 해석적으로 계산해 반복 곡선 길이 탐색을 없앤다.
        for i, seg_length in enumerate(geometry['segment_lengths']):
            end_s = geometry['cumulative_lengths'][i + 1]
            if seg_length <= 1e-9 or end_s <= start_s:
                continue
            begin_s = max(start_s, geometry['cumulative_lengths'][i])
            a = self.get_link_point_at_progress(geometry, begin_s)['point']
            b = self.get_link_point_at_progress(geometry, end_s)['point']
            dx, dy = b['rviz_x'] - a['rviz_x'], b['rviz_y'] - a['rviz_y']
            ox, oy = a['rviz_x'] - start['rviz_x'], a['rviz_y'] - start['rviz_y']
            aa = dx * dx + dy * dy
            if aa <= 1e-12:
                continue
            bb = 2.0 * (ox * dx + oy * dy)
            cc = ox * ox + oy * oy - target_length * target_length
            discriminant = bb * bb - 4.0 * aa * cc
            if discriminant < -1e-9:
                continue
            root = math.sqrt(max(0.0, discriminant))
            for ratio in sorted(((-bb - root) / (2.0 * aa),
                                 (-bb + root) / (2.0 * aa))):
                if -1e-9 <= ratio <= 1.0 + 1e-9:
                    ratio = max(0.0, min(1.0, ratio))
                    end_info = self.get_link_point_at_progress(
                        geometry, begin_s + ratio * (end_s - begin_s)
                    )
                    return self.make_candidate_straight_path(start_info, end_info)
        end_info = self.get_link_point_at_progress(geometry, geometry['total_length'])
        length = self.calc_candidate_straight_length(start_info, end_info)
        if (minimum_length is not None
                and minimum_length - self.length_tolerance_m <= length
                <= target_length + self.length_tolerance_m):
            return self.make_candidate_straight_path(start_info, end_info)
        return None

    def make_link_geometry(self, link):
        cache_key = None
        link_idx = link.get('idx')

        if (
            link_idx is not None
            and self.link_by_idx.get(str(link_idx)) is link
        ):
            cache_key = str(link_idx)

            if cache_key in self.link_geometry_cache:
                return self.link_geometry_cache[cache_key]

        points = link.get('points', [])

        if len(points) < 2:
            if cache_key is not None:
                self.link_geometry_cache[cache_key] = None
            return None

        point_x = [
            float(point.get('rviz_x', 0.0))
            for point in points
        ]
        point_y = [
            float(point.get('rviz_y', 0.0))
            for point in points
        ]
        segment_lengths = []
        segment_dx = []
        segment_dy = []
        cumulative_lengths = [0.0]

        for point_i in range(len(points) - 1):
            dx = point_x[point_i + 1] - point_x[point_i]
            dy = point_y[point_i + 1] - point_y[point_i]
            segment_length = math.hypot(dx, dy)
            segment_dx.append(dx)
            segment_dy.append(dy)
            segment_lengths.append(segment_length)
            cumulative_lengths.append(cumulative_lengths[-1] + segment_length)

        if cumulative_lengths[-1] <= 1e-9:
            if cache_key is not None:
                self.link_geometry_cache[cache_key] = None
            return None

        geometry = {
            'points': points,
            'segment_lengths': segment_lengths,
            'cumulative_lengths': cumulative_lengths,
            'total_length': cumulative_lengths[-1],
            '_point_x': point_x,
            '_point_y': point_y,
            '_segment_dx': segment_dx,
            '_segment_dy': segment_dy
        }

        if cache_key is not None:
            self.link_geometry_cache[cache_key] = geometry

        return geometry

    def get_link_point_at_progress(self, geometry, progress):
        progress = max(0.0, min(float(progress), geometry['total_length']))
        points = geometry['points']
        cumulative_lengths = geometry['cumulative_lengths']
        segment_i = bisect_left(
            cumulative_lengths,
            progress,
            1
        ) - 1
        segment_i = max(
            0,
            min(segment_i, len(geometry['segment_lengths']) - 1)
        )
        segment_length = geometry['segment_lengths'][segment_i]
        segment_start_s = cumulative_lengths[segment_i]
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

    def interpolate_value(self, p0, p1, key, ratio):
        value0 = self.get_point_value(p0, key)
        value1 = self.get_point_value(p1, key)
        return value0 + (value1 - value0) * ratio

    def get_point_value(self, point, key):
        if key == 'z':
            return float(point.get('z', point.get('morai_z', 0.0)))

        return float(point.get(key, 0.0))

    def calc_candidate_straight_length(self, start_info, end_info):
        if start_info is None or end_info is None:
            return None
        start, end = start_info['point'], end_info['point']
        return math.hypot(end['rviz_x'] - start['rviz_x'],
                          end['rviz_y'] - start['rviz_y'])

    def make_candidate_straight_path(self, start_info, end_info):
        length = self.calc_candidate_straight_length(start_info, end_info)
        if length is None or length <= 1e-9:
            return None
        start, end = start_info['point'], end_info['point']
        intervals = max(1, int(math.ceil(length / self.point_interval_m)))
        return [
            {key: start[key] + (end[key] - start[key]) * i / intervals
             for key in ('rviz_x', 'rviz_y', 'z')}
            for i in range(intervals + 1)
        ]

    def calc_curve_length(self, curve):
        return sum(
            math.hypot(
                p1['rviz_x'] - p0['rviz_x'],
                p1['rviz_y'] - p0['rviz_y']
            )
            for p0, p1 in zip(curve, curve[1:])
        )
