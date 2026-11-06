#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
import threading
from pathlib import Path

import rospy

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from path_planning.link.morai_link_planner import (
    CURRENT_LINK_CHANGE_CONFIRM_COUNT,
    CURRENT_LINK_MAX_LANE_CHANGE_COUNT,
    CURRENT_LINK_REACQUIRE_DISTANCE_M,
    CURRENT_LINK_ROUTE_LINK_IDXS,
    MoraiMapPlanner,
)
from path_planning.link.lane_change_candidate import LaneChangeCandidateGenerator
from path_planning.link.reverse_link import ReverseLinkTargetFinder
from path_planning.link.tunnel_reverse import TUNNEL_LINK_BY_LANE, TUNNEL_LANE_BY_LINK
from path_planning.link.virtual_lane_boundary import VirtualLaneBoundaryGenerator
from macaron_8.msg import CurrentLink


# 현재 링크 발행 설정
DEFAULT_INTERSECTION_PREVIEW_DISTANCE_M = 15.0  # 교차로 link_type 사전 발행 거리 [m]

# 후방 목표점 설정
DEFAULT_REVERSE_TARGET_DISTANCE_M = 8.0  # 차량 뒤 목표 거리 [m]
DEFAULT_REVERSE_TARGET_PUBLISH_HZ = 10.0  # 후방 목표점 발행 주기 [Hz]
REVERSE_SEARCH_MAX_PREVIOUS_LINKS = 10  # 후방 탐색 최대 링크 수
REVERSE_POINT_EQUALITY_EPSILON_M = 1e-6  # 연결점 동일 판정 오차 [m]
REVERSE_PREVIOUS_LINK_OVERRIDE_BY_LINK_IDX = {  # 분리된 시작 노드의 이전 링크 지정
    'A2256W000009': 'A2256W000220',
    'A2256W000131': 'A2256W000448',
}

# 차선 변경 후보 설정
LANE_CHANGE_CANDIDATE_UPDATE_DISTANCE_M = 1.0  # 정적 목적 링크 결과의 캐시 재사용 거리 [m]
DEFAULT_LANE_CHANGE_CANDIDATE_PUBLISH_HZ = 12.0  # 후보 경로 발행 주기 [Hz]
LANE_CHANGE_CANDIDATE_START_AHEAD_M = 0.0  # 차량 투영점에서 후보 경로 시작점까지의 전방거리 [m]
LANE_CHANGE_CANDIDATE_PATH_LENGTH_M = 40.0  # 기본 후보 경로 길이 [m]
LANE_CHANGE_CANDIDATE_MIN_PATH_LENGTH_M = 16.0  # 허용할 최소 후보 경로 길이 [m]
LANE_CHANGE_CANDIDATE_LENGTH_TOLERANCE_M = 0.15  # 후보 길이 허용 오차 [m]
LANE_CHANGE_CANDIDATE_POINT_INTERVAL_M = 0.5  # 좌우 후보 표본 간격 [m]
HIGHWAY_CANDIDATE_PATH_EXTENSION_M = 40.0  # 고속도로 후보 끝의 차선 중심선 연장 길이 [m]
LANE_CHANGE_ENDPOINT_NO_NEXT_LINK_IDXS = {'A2256W000445', 'A2256W000422'} # 다음 링크로 차선 변경 후보 넘어가지 않도록
LANE_CHANGE_PATH_LENGTH_OVERRIDES_M = {'A2256W000445': 20.0}   # 차선 변경 후보 길이 조정 [m]

LANE_CHANGE_CANDIDATE_ID_BY_SIDE = {  # 후보 방향별 PointCloud 식별자
    'center': 0,
    'left': 1,
    'right': 2
}
LANE_CHANGE_MIN_PROGRESS_M_BY_LINK_AND_SIDE = {('A2256W000202', 'right'): 0.1}  # 링크·방향별 차선 변경 허용 시작거리 [m]

# 가상 차선 경계 설정
VIRTUAL_CENTER_LINE_LANE_WIDTH_M = 3.5  # 가상 차선 경계 기준 차선 폭 [m]
VIRTUAL_CONNECTED_POINT_TOLERANCE_M = 1e-3  # 링크 연결점 동일 판정 오차 [m]
VIRTUAL_LANE_BOUNDARY_LINK_IDX_GROUPS = (  # 함께 발행할 가상 차선 경계 링크 묶음
    (
        'A2256W000304',
        'A2256W000148'
    ),
    (
        'A2256W000213',
    ),
    (
        'A2256W000126',
        'A2256W000131',
    )
)
VIRTUAL_RIGHT_BOUNDARY_LINK_IDXS = {  # 보정된 링크 진행 방향 기준 오른쪽 경계
    'A2256W000126'
}



# current_link_data 관련 링크 속성
LINK_INFORMATION_BY_LINK_IDX = {  # 링크별 주행 구간·시설 정보
    'A2256W000219': 'three_light_traffic_signal',
    'A2256W000213': 'three_light_traffic_signal',
    'A2256W000202': 'four_light_traffic_signal',
    'A2256W000728': 'four_light_traffic_signal',
    'A2256W000308': 'four_light_traffic_signal',
    'A2256W000309': 'four_light_traffic_signal',
    'A2256W000128': 'ROUNDABOUT_AHEAD',
    'A2256W000333': 'ROUNDABOUT_ENTRY',
    'A2256W000411': 'HIGHWAY_ENTRY',
    'A2256W000409': 'LANE_3',
    'A2256W000410': 'LANE_2',
    'A2256W000418': 'LANE_1',
    'A2256W000420': 'LANE_3',
    'A2256W000430': 'LANE_2',
    'A2256W000431': 'LANE_1',
    'A2256W000408': 'LANE_3',
    'A2256W000434': 'LANE_2',
    'A2256W000435': 'LANE_1',
    'A2256W000153': 'TOLLGATE_ENTRY',
    'A2256W000148': 'guardrail_endpoint'
}

NOT_AVOID_LINK_IDXS = {  # 장애물 회피를 비활성화할 링크
    'A2256W000751',
    'A2256W000748',
    'A2256W000329',
    'A2256W000219',
    'A2256W000213',
    'A2256W000148',
    'A2256W000866',
    'A2256W000846',
    'A2256W000308'
}
NOT_AVOID_PREVIEW_DISTANCE_M_BY_LINK_IDX = {  # 링크 끝 전 not_avoid 선행 적용 거리 [m]
    'A2256W000304': 10.0,
    'A2256W000182': 5.0
}
LINK_TYPE_END_OVERRIDE_BY_LINK_IDX = {  # 링크 끝 구간의 link_type과 적용 거리 [m]
    'A2256W000182': ('6', 5.0)
}
LINK_TYPE_SECTION_OVERRIDES = (  # 정적 구간 설정
    {
        'link_type': '17',
        'link_idxs': ('A2256W000213','A2256W000222','A2256W000220','A2256W000202'),
        'start_remaining_distance_m': 15.0,  # 첫 링크 끝점까지 남은 거리 [m]
        'end_progress_m': 22.0,  # 마지막 링크 시작점부터 진행 거리 [m]
    },
)

DEFAULT_LINK_INFORMATION_PREVIEW_DISTANCE_M_BY_LINK_IDX = {  # 신호등 미리  [m]
    'A2256W000213': 25.0,
    'A2256W000308': 25.0,
    'A2256W000202' : 10.0
}



class MoraiMapPublisher:
    def __init__(self):
        self.link_path = self.default_map_path('link_set.json')
        self.node_path = self.default_map_path('node_set.json')
        self.origin_utm_x = 302595.0  # MORAI 원점 UTM X [m]
        self.origin_utm_y = 4124145.0  # MORAI 원점 UTM Y [m]
        self.use_utm_offset = True  # RViz 좌표에 MORAI 원점 오프셋 적용

        # 구독 토픽
        self.current_pose_topic = '/current_pose'
        # 발행 토픽
        self.map_data_topic = '/morai_map_data'
        self.current_link_topic = '/current_link_data'
        self.reverse_target_topic = '/reverse_link_point'
        self.lane_change_candidate_data_topic = '/lane_change_candidate_data'
        self.lane_change_candidate_link_data_topic = (
            '/lane_change_candidate_link'
        )
        self.virtual_lane_boundary_topic = '/current_link_virtual_lane_boundaries'
        self.heading_filter_deg = 90.0  # 링크 검색 허용 방향 오차 [deg]


        
        self.map_data_pub = rospy.Publisher(self.map_data_topic, String, queue_size=1, latch=True)
        self.current_link_pub = rospy.Publisher(self.current_link_topic, CurrentLink, queue_size=10)
        self.reverse_target_pub = rospy.Publisher(self.reverse_target_topic, Point, queue_size=10)
        self.tunnel_reverse_target_pub = rospy.Publisher('/tunnel_reverse_link_points', PointCloud2, queue_size=1)
        self.tunnel_reverse_targets_active = False
        self.lane_change_candidate_data_pub = rospy.Publisher(self.lane_change_candidate_data_topic, PointCloud2, queue_size=1, latch=True)
        self.lane_change_candidate_link_data_pub = rospy.Publisher(self.lane_change_candidate_link_data_topic, PointCloud2, queue_size=1, latch=True)
        self.virtual_lane_boundary_pub = rospy.Publisher(self.virtual_lane_boundary_topic, PointCloud2, queue_size=1, latch=True)


        self.planner = None
        self.current_link_idx = None
        self.pending_current_link_idx = None
        self.pending_current_link_count = 0
        self.current_pose = None
        self.current_link_allowed_link_idxs = []
        self.current_link_allowed_link_idx_set = set()
        self.lane_change_candidate_link_allowed_link_idxs = []
        self.lane_change_candidate_link_allowed_link_idx_set = set()
        self.lane_change_candidate_link_current_link_idx = None
        self.lane_change_extension_link_idxs = []
        self.lane_change_candidate_generator = None
        self.link_type_section_ranges_by_link_idx = {}
        self.lane_change_candidate_lock = threading.RLock()
        self.latest_lane_change_candidate_context = None
        self.lane_change_candidate_data_active = False
        self.lane_change_candidate_cache_by_topic = {
            'data': {
                'link_idx': None,
                'progress_m': None,
                'result': None
            },
            'link': {
                'link_idx': None,
                'progress_m': None,
                'result': None
            }
        }
        self.lane_change_candidate_link_points_by_idx = {}
        self.last_lane_change_candidate_link_signature = None
        self.virtual_lane_boundary_generator = (
            VirtualLaneBoundaryGenerator(
                target_link_idx_groups=VIRTUAL_LANE_BOUNDARY_LINK_IDX_GROUPS,
                right_boundary_link_idxs=VIRTUAL_RIGHT_BOUNDARY_LINK_IDXS,
                lane_width_m=VIRTUAL_CENTER_LINE_LANE_WIDTH_M,
                connected_point_tolerance_m=(
                    VIRTUAL_CONNECTED_POINT_TOLERANCE_M
                )
            )
        )
        self.active_virtual_center_line_link_idxs = ()
        self.current_pose_sub = None
        self.lane_change_candidate_timer = None

        self.load_morai_map()
        self.current_pose_sub = rospy.Subscriber(
            self.current_pose_topic,
            PointStamped,
            self.current_pose_callback,
            queue_size=1
        )
        self.reverse_target_timer = rospy.Timer(
            rospy.Duration(1.0 / DEFAULT_REVERSE_TARGET_PUBLISH_HZ),
            self.publish_reverse_target
        )
        self.lane_change_candidate_timer = rospy.Timer(
            rospy.Duration(1.0 / DEFAULT_LANE_CHANGE_CANDIDATE_PUBLISH_HZ),
            self.publish_lane_change_candidate_data
        )
        rospy.loginfo(
            'Reverse-link target enabled: distance=%.2fm, rate=%.1fHz, '
            'topic=%s',
            DEFAULT_REVERSE_TARGET_DISTANCE_M,
            DEFAULT_REVERSE_TARGET_PUBLISH_HZ,
            self.reverse_target_topic
        )
        rospy.loginfo(
            'Lane-change candidates enabled: rate=%.1fHz, topic=%s',
            DEFAULT_LANE_CHANGE_CANDIDATE_PUBLISH_HZ,
            self.lane_change_candidate_data_topic
        )
        rospy.loginfo(
            'Link-information preview distances: %s',
            ', '.join(
                '%s=%.2fm' % (link_idx, distance_m)
                for link_idx, distance_m in (
                    DEFAULT_LINK_INFORMATION_PREVIEW_DISTANCE_M_BY_LINK_IDX.items()
                )
            )
        )
        rospy.loginfo(
            'Current-link change confirmation count: %d',
            CURRENT_LINK_CHANGE_CONFIRM_COUNT
        )
    def default_map_path(self, file_name):
        package_src = Path(__file__).resolve().parents[2]
        return str(package_src / 'path' / 'MGeo' / file_name)

    def load_morai_map(self):
        self.planner = MoraiMapPlanner(
            link_path=self.link_path,
            origin_utm_x=self.origin_utm_x,
            origin_utm_y=self.origin_utm_y,
            node_path=self.node_path,
            use_utm_offset=self.use_utm_offset,
            heading_filter_deg=self.heading_filter_deg
        )

        rospy.loginfo('Loading MORAI map link_set: %s', self.link_path)
        rospy.loginfo('Loading MORAI map node_set: %s', self.node_path)
        self.planner.load(build_nearest_lookup=False)
        self.reverse_link_finder = ReverseLinkTargetFinder(
            link_by_idx=self.planner.link_by_idx,
            links_by_to_node=self.planner.links_by_to_node,
            route_link_idxs=CURRENT_LINK_ROUTE_LINK_IDXS,
            max_previous_link_hops=REVERSE_SEARCH_MAX_PREVIOUS_LINKS,
            point_epsilon_m=REVERSE_POINT_EQUALITY_EPSILON_M,
            previous_link_override_by_link_idx=(
                REVERSE_PREVIOUS_LINK_OVERRIDE_BY_LINK_IDX
            )
        )
        self.build_lane_change_candidate_link_point_cache()
        self.build_current_link_allowed_link_idxs()
        self.planner.build_nearest_lookup(
            self.current_link_allowed_link_idxs
        )
        highway_start = CURRENT_LINK_ROUTE_LINK_IDXS.index('A2256W000411')
        tollgate_end = CURRENT_LINK_ROUTE_LINK_IDXS.index('A2256W000153')
        highway_route_link_idxs = CURRENT_LINK_ROUTE_LINK_IDXS[
            highway_start:tollgate_end + 1
        ]
        self.lane_change_candidate_allowed_link_idxs = (
            self.planner.get_lane_change_link_closure(
                highway_route_link_idxs,
                max_lane_change_count=CURRENT_LINK_MAX_LANE_CHANGE_COUNT
            )
        )
        self.lane_change_candidate_allowed_link_idx_set = set(
            self.lane_change_candidate_allowed_link_idxs
        )
        self.lane_change_candidate_generator = LaneChangeCandidateGenerator(
            links=self.planner.links,
            nodes=self.planner.nodes,
            start_ahead_of_vehicle_m=LANE_CHANGE_CANDIDATE_START_AHEAD_M,
            candidate_path_length_m=LANE_CHANGE_CANDIDATE_PATH_LENGTH_M,
            minimum_candidate_path_length_m=(
                LANE_CHANGE_CANDIDATE_MIN_PATH_LENGTH_M
            ),
            length_tolerance_m=(
                LANE_CHANGE_CANDIDATE_LENGTH_TOLERANCE_M
            ),
            point_interval_m=LANE_CHANGE_CANDIDATE_POINT_INTERVAL_M,
            candidate_path_extension_m=(
                HIGHWAY_CANDIDATE_PATH_EXTENSION_M
            ),
            endpoint_no_next_link_idxs=(
                LANE_CHANGE_ENDPOINT_NO_NEXT_LINK_IDXS
            ),
            path_length_overrides_m=(
                LANE_CHANGE_PATH_LENGTH_OVERRIDES_M
            ),
            route_link_idxs=CURRENT_LINK_ROUTE_LINK_IDXS,
            lane_change_extension_link_idxs=(
                self.lane_change_extension_link_idxs
            ),
            candidate_allowed_link_idxs=(
                self.lane_change_candidate_allowed_link_idxs
            )
        )
        self.build_link_type_section_ranges()
        self.publish_map_data()
        rospy.loginfo('Loaded MORAI map links: %d', len(self.planner.links))
        rospy.loginfo('Loaded MORAI map nodes: %d', len(self.planner.nodes))
        rospy.loginfo(
            'Built nearest lookup segments: %d',
            len(self.planner.segment_refs)
        )

    def build_link_type_section_ranges(self):
        """연속 구간의 거리 경계를 차선 변경 대상의 중심선에도 적용한다."""
        self.link_type_section_ranges_by_link_idx = {}
        generator = self.lane_change_candidate_generator

        for section in LINK_TYPE_SECTION_OVERRIDES:
            link_idxs = section['link_idxs']

            for link_i, link_idx in enumerate(link_idxs):
                link = self.planner.link_by_idx.get(str(link_idx))

                if link is None:
                    rospy.logwarn('Link-type section link not found: %s', link_idx)
                    continue

                geometry = generator.make_link_geometry(link)

                if geometry is None:
                    continue

                total_length = geometry['total_length']
                start_progress = 0.0
                end_progress = total_length

                if link_i == 0:
                    start_progress = max(
                        0.0,
                        total_length - section['start_remaining_distance_m']
                    )

                if link_i == len(link_idxs) - 1:
                    end_progress = min(total_length, section['end_progress_m'])

                target_link_idxs = self.planner.get_lane_change_link_closure(
                    [link_idx],
                    max_lane_change_count=CURRENT_LINK_MAX_LANE_CHANGE_COUNT
                )

                for target_link_idx in target_link_idxs:
                    target_link = self.planner.link_by_idx[target_link_idx]
                    target_geometry = generator.make_link_geometry(target_link)

                    if target_geometry is None:
                        continue

                    target_start = 0.0
                    target_end = target_geometry['total_length']

                    if target_link_idx == str(link_idx):
                        target_start = start_progress
                        target_end = end_progress
                    else:
                        # 부분 구간의 경계점을 투영해 차선별 길이 차이를 반영한다.
                        if start_progress > 0.0:
                            point = generator.get_link_point_at_progress(
                                geometry, start_progress
                            )['point']
                            target_start = generator.find_nearest_geometry_progress(
                                target_geometry, point['rviz_x'], point['rviz_y']
                            )

                        if end_progress < total_length:
                            point = generator.get_link_point_at_progress(
                                geometry, end_progress
                            )['point']
                            target_end = generator.find_nearest_geometry_progress(
                                target_geometry, point['rviz_x'], point['rviz_y']
                            )

                    if (
                        target_start is None
                        or target_end is None
                        or target_start > target_end
                    ):
                        continue

                    self.link_type_section_ranges_by_link_idx.setdefault(
                        target_link_idx, []
                    ).append((str(section['link_type']), target_start, target_end))

    def build_lane_change_candidate_link_point_cache(self):
        self.lane_change_candidate_link_points_by_idx = {}
        self.last_lane_change_candidate_link_signature = None

        for link in self.planner.links:
            link_idx = link.get('idx')

            if link_idx is None:
                continue

            points = []

            for point in link.get('points', []):
                x = float(point.get('rviz_x', 0.0))
                y = float(point.get('rviz_y', 0.0))

                if self.use_utm_offset:
                    x += self.origin_utm_x
                    y += self.origin_utm_y

                points.append((x, y))

            self.lane_change_candidate_link_points_by_idx[
                str(link_idx)
            ] = tuple(points)

    def build_current_link_allowed_link_idxs(self):
        missing_link_idxs = [
            link_idx
            for link_idx in CURRENT_LINK_ROUTE_LINK_IDXS
            if str(link_idx) not in self.planner.link_by_idx
        ]

        if missing_link_idxs:
            rospy.logwarn(
                'Current-link route links not found: %s',
                ', '.join(sorted(set(missing_link_idxs)))
            )

        self.lane_change_candidate_link_allowed_link_idxs = []
        route_link_idx_set = set()

        for link_idx in CURRENT_LINK_ROUTE_LINK_IDXS:
            link_idx = str(link_idx)

            if (
                link_idx not in self.planner.link_by_idx
                or link_idx in route_link_idx_set
            ):
                continue

            route_link_idx_set.add(link_idx)
            self.lane_change_candidate_link_allowed_link_idxs.append(
                link_idx
            )

        self.lane_change_candidate_link_allowed_link_idx_set = (
            route_link_idx_set
        )
        self.current_link_allowed_link_idxs = (
            self.planner.get_lane_change_link_closure(
                self.lane_change_candidate_link_allowed_link_idxs,
                max_lane_change_count=(
                    CURRENT_LINK_MAX_LANE_CHANGE_COUNT
                )
            )
        )
        self.current_link_allowed_link_idx_set = set(
            self.current_link_allowed_link_idxs
        )
        self.lane_change_extension_link_idxs = list(
            self.current_link_allowed_link_idxs
        )

        rospy.loginfo(
            'Built current-link whitelist: '
            'route_only_links=%d, route_with_lane_changes=%d, '
            'max_lane_changes=%d',
            len(self.lane_change_candidate_link_allowed_link_idxs),
            len(self.current_link_allowed_link_idxs),
            CURRENT_LINK_MAX_LANE_CHANGE_COUNT
        )
        rospy.loginfo(
            'Lane-change topic link bases: '
            'candidate_data_links=%d, candidate_link_links=%d',
            len(self.current_link_allowed_link_idxs),
            len(self.lane_change_candidate_link_allowed_link_idxs)
        )

    def publish_map_data(self):
        payload = {
            'origin_utm_x': self.origin_utm_x,
            'origin_utm_y': self.origin_utm_y,
            'use_utm_offset': self.use_utm_offset,
            'links': [self.make_link_payload(link) for link in self.planner.links],
            'nodes': [self.make_node_payload(node) for node in self.planner.nodes]
        }

        self.map_data_pub.publish(self.dumps(payload))
        rospy.loginfo(
            'Published MORAI map data: links=%d, nodes=%d',
            len(payload['links']),
            len(payload['nodes'])
        )

    def virtual_lane_boundary_fields(self):
        names = ('x', 'y')
        fields = [
            PointField(
                name=name,
                offset=field_i * 8,
                datatype=PointField.FLOAT64,
                count=1
            )
            for field_i, name in enumerate(names)
        ]
        fields.append(PointField(
            name='boundary_id',
            offset=16,
            datatype=PointField.UINT8,
            count=1
        ))
        return fields

    def lane_change_candidate_fields(self):
        return [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT64,
                count=1
            ),
            PointField(
                name='y',
                offset=8,
                datatype=PointField.FLOAT64,
                count=1
            ),
            PointField(
                name='candidate_id',
                offset=16,
                datatype=PointField.UINT8,
                count=1
            )
        ]

    def lane_change_candidate_link_fields(self):
        return [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT64,
                count=1
            ),
            PointField(
                name='y',
                offset=8,
                datatype=PointField.FLOAT64,
                count=1
            ),
            PointField(
                name='candidate_id',
                offset=16,
                datatype=PointField.UINT8,
                count=1
            ),
            PointField(
                name='link_order',
                offset=18,
                datatype=PointField.UINT16,
                count=1
            )
        ]

    def make_lane_change_candidate_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        return header

    def publish_lane_change_candidate_cloud(self, result):
        rows = []
        # 가능 여부 계산에는 전체 후보를 사용하고, 송신할 좌우 경로만 제한한다.
        publish_lane_changes = (
            str(result.get('current_link_idx'))
            in self.lane_change_candidate_allowed_link_idx_set
        )

        for candidate in result.get('candidates', []):
            if candidate.get('side') in ('left', 'right') and not publish_lane_changes:
                continue
            candidate_id = LANE_CHANGE_CANDIDATE_ID_BY_SIDE.get(
                candidate.get('side')
            )

            if candidate_id is None:
                continue

            for point in candidate.get('path', []):
                x = float(point.get('rviz_x', 0.0))
                y = float(point.get('rviz_y', 0.0))

                if self.use_utm_offset:
                    x += self.origin_utm_x
                    y += self.origin_utm_y

                rows.append((
                    x,
                    y,
                    candidate_id
                ))

        cloud_msg = point_cloud2.create_cloud(
            self.make_lane_change_candidate_header(),
            self.lane_change_candidate_fields(),
            rows
        )
        self.lane_change_candidate_data_pub.publish(cloud_msg)

    def make_lane_change_candidate_link_signature(self, result):
        signature = []

        for candidate in result.get('candidates', []):
            candidate_id = LANE_CHANGE_CANDIDATE_ID_BY_SIDE.get(
                candidate.get('side')
            )

            if candidate_id not in (1, 2):
                continue

            link_idxs = candidate.get('destination_link_idxs')

            if not link_idxs:
                destination_link_idx = candidate.get(
                    'destination_link_idx'
                )
                link_idxs = (
                    []
                    if destination_link_idx is None
                    else [destination_link_idx]
                )

            cached_link_idxs = tuple(
                str(link_idx)
                for link_idx in link_idxs
                if str(link_idx) in (
                    self.lane_change_candidate_link_points_by_idx
                )
            )

            if cached_link_idxs:
                signature.append((candidate_id, cached_link_idxs))

        return tuple(sorted(signature))

    def publish_lane_change_candidate_link_cloud(self, result):
        signature = self.make_lane_change_candidate_link_signature(result)

        if signature == self.last_lane_change_candidate_link_signature:
            return

        rows = []

        for candidate_id, link_idxs in signature:
            for link_order, link_idx in enumerate(link_idxs):
                for x, y in (
                    self.lane_change_candidate_link_points_by_idx[link_idx]
                ):
                    rows.append((
                        x,
                        y,
                        candidate_id,
                        link_order
                    ))

        cloud_msg = point_cloud2.create_cloud(
            self.make_lane_change_candidate_header(),
            self.lane_change_candidate_link_fields(),
            rows
        )
        self.lane_change_candidate_link_data_pub.publish(cloud_msg)
        self.last_lane_change_candidate_link_signature = signature

    def make_virtual_lane_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        return header

    def publish_virtual_lane_boundaries(self, link):
        link_idx = '' if link is None else str(link.get('idx', ''))
        boundary_link_idxs = (
            self.virtual_lane_boundary_generator.get_target_link_idx_group(
                link_idx
            )
        )

        if boundary_link_idxs == self.active_virtual_center_line_link_idxs:
            return

        result = self.virtual_lane_boundary_generator.make_boundary(
            link,
            link_by_idx=self.planner.link_by_idx
        )
        link_idx = result['link_idx']
        is_target_link = result['active']

        cloud_rows = [
            (point['utm_x'], point['utm_y'], point['boundary_id'])
            for point in result['points']
        ]

        cloud_msg = point_cloud2.create_cloud(
            self.make_virtual_lane_header(),
            self.virtual_lane_boundary_fields(),
            cloud_rows
        )
        self.virtual_lane_boundary_pub.publish(cloud_msg)
        self.active_virtual_center_line_link_idxs = tuple(
            result['boundary_link_idxs']
        )
        rospy.loginfo(
            'Published virtual lane boundaries: '
            'current_link=%s boundary_links=%s active=%s',
            link_idx,
            ','.join(result['boundary_link_idxs']),
            is_target_link
        )

    def current_pose_callback(self, pose):
        utm_x = float(pose.point.x)
        utm_y = float(pose.point.y)

        if not math.isfinite(utm_x) or not math.isfinite(utm_y):
            return

        self.current_pose = (utm_x, utm_y)
        self.publish_current_link_info(
            utm_x,
            utm_y,
            float(pose.point.z)
        )

    def publish_reverse_target(self, _event):
        current_pose = self.current_pose
        current_link_idx = self.current_link_idx

        self.publish_tunnel_reverse_targets(current_pose, current_link_idx)

        if (
            current_pose is None
            or current_link_idx is None
        ):
            return

        target = self.reverse_link_finder.find_target(
            vehicle_x=current_pose[0],
            vehicle_y=current_pose[1],
            current_link_idx=current_link_idx,
            target_distance_m=DEFAULT_REVERSE_TARGET_DISTANCE_M
        )

        if target is None:
            rospy.logwarn_throttle(
                2.0,
                'Unable to find %.2fm reverse target from link %s',
                DEFAULT_REVERSE_TARGET_DISTANCE_M,
                current_link_idx
            )
            return

        target_x, target_y, target_yaw = target
        point_msg = Point(
            x=target_x,
            y=target_y,
            z=target_yaw
        )
        self.reverse_target_pub.publish(point_msg)

    def publish_tunnel_reverse_targets(self, current_pose, current_link_idx):
        in_tunnel = str(current_link_idx) in TUNNEL_LANE_BY_LINK
        if not in_tunnel and not self.tunnel_reverse_targets_active:
            return

        rows = []
        if in_tunnel and current_pose is not None:
            for lane_id, link_idx in TUNNEL_LINK_BY_LANE.items():
                target = self.reverse_link_finder.find_target(
                    vehicle_x=current_pose[0],
                    vehicle_y=current_pose[1],
                    current_link_idx=link_idx,
                    target_distance_m=DEFAULT_REVERSE_TARGET_DISTANCE_M,
                )
                if target is None or not all(math.isfinite(v) for v in target):
                    rospy.logwarn_throttle(
                        2.0, 'No tunnel reverse target: lane=%d link=%s',
                        lane_id, link_idx,
                    )
                    continue
                rows.append((lane_id, target[0], target[1], target[2]))

        # UTM 정밀도를 보존한다. yaw는 높이가 아니라 차량 목표 방향 [rad].
        fields = [
            PointField('lane_id', 0, PointField.UINT32, 1),
            PointField('x', 4, PointField.FLOAT64, 1),
            PointField('y', 12, PointField.FLOAT64, 1),
            PointField('yaw', 20, PointField.FLOAT64, 1),
        ]
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        self.tunnel_reverse_target_pub.publish(
            point_cloud2.create_cloud(header, fields, rows)
        )
        # 터널 진출 시 빈 cloud를 한 번 보내 수신측의 이전 후보를 제거한다.
        self.tunnel_reverse_targets_active = in_tunnel

    def publish_current_link_info(self, utm_x, utm_y, yaw=None):
        nearest = self.find_current_link(utm_x, utm_y, yaw)
        nearest = self.confirm_current_link_candidate(
            nearest,
            utm_x,
            utm_y,
            yaw
        )

        if nearest is None:
            self.latest_lane_change_candidate_context = None
            self.publish_lane_change_candidate_links(None)
            self.publish_virtual_lane_boundaries(None)
            return

        link = nearest['link']
        link_idx = link.get('idx')
        self.current_link_idx = (
            None if link_idx is None else str(link_idx)
        )

        if self.current_link_idx is not None:
            self.reverse_link_finder.observe_current_link(
                self.current_link_idx
            )

        node = self.planner.node_by_idx.get(link.get('to_node_idx'))

        on_stop_line = None
        traffic_light_id = None

        if node is not None:
            on_stop_line = node.get('on_stop_line')
            traffic_light_id = node.get('traffic_light_id')

        self.latest_lane_change_candidate_context = {
            'link_idx': link_idx,
            'rviz_x': nearest['point']['rviz_x'],
            'rviz_y': nearest['point']['rviz_y'],
            'segment_index': nearest['segment_index'],
            'segment_ratio': nearest['segment_ratio']
        }
        # JSON 값을 planner의 금지/목적 링크 보정까지 적용한 속성으로 발행한다.
        # 후보 경로의 생성 성공 여부나 발행 구간과는 독립적이다.
        lane_change_permissions = self.get_lane_change_permissions(
            link,
            nearest['segment_index'],
            nearest['segment_ratio']
        )
        can_change_left = lane_change_permissions['left']
        can_change_right = lane_change_permissions['right']

        self.publish_lane_change_candidate_links(
            link,
            nearest,
            lane_change_permissions
        )

        self.publish_virtual_lane_boundaries(link)

        self.publish_current_link(
            link_idx,
            self.get_published_link_type(
                link,
                nearest['segment_index'],
                nearest['segment_ratio']
            ),
            link.get('max_speed'),
            on_stop_line,
            traffic_light_id,
            link.get('related_signal'),
            can_change_left,
            can_change_right,
            self.get_published_not_avoid(
                link,
                nearest['segment_index'],
                nearest['segment_ratio']
            ),
            self.get_published_link_information(
                link,
                nearest['segment_index'],
                nearest['segment_ratio']
            )
        )

    def confirm_current_link_candidate(
        self,
        nearest,
        utm_x,
        utm_y,
        yaw=None
    ):
        """다른 링크가 설정 횟수만큼 연속 관측될 때만 현재 링크를 변경한다."""
        if nearest is None:
            self.pending_current_link_idx = None
            self.pending_current_link_count = 0
            return None

        candidate_link = nearest.get('link')
        candidate_link_idx = (
            None
            if candidate_link is None or candidate_link.get('idx') is None
            else str(candidate_link.get('idx'))
        )

        if candidate_link_idx is None:
            self.pending_current_link_idx = None
            self.pending_current_link_count = 0
            return None

        if self.current_link_idx is None:
            self.pending_current_link_idx = None
            self.pending_current_link_count = 0
            return nearest

        if candidate_link_idx == self.current_link_idx:
            self.pending_current_link_idx = None
            self.pending_current_link_count = 0
            return nearest

        if candidate_link_idx == self.pending_current_link_idx:
            self.pending_current_link_count += 1
        else:
            self.pending_current_link_idx = candidate_link_idx
            self.pending_current_link_count = 1

        if (
            self.pending_current_link_count
            >= CURRENT_LINK_CHANGE_CONFIRM_COUNT
        ):
            rospy.loginfo(
                'Current link changed after %d consecutive observations: '
                '%s -> %s',
                self.pending_current_link_count,
                self.current_link_idx,
                candidate_link_idx
            )
            self.pending_current_link_idx = None
            self.pending_current_link_count = 0
            return nearest

        rospy.logdebug(
            'Pending current-link change: %s -> %s (%d/%d)',
            self.current_link_idx,
            candidate_link_idx,
            self.pending_current_link_count,
            CURRENT_LINK_CHANGE_CONFIRM_COUNT
        )

        current_nearest = self.planner.find_nearest_link_and_segment(
            float(utm_x),
            float(utm_y),
            yaw=yaw,
            candidate_link_idxs=[self.current_link_idx]
        )

        if current_nearest is None:
            rospy.logwarn_throttle(
                2.0,
                'Unable to retain current link %s while candidate %s is '
                'pending',
                self.current_link_idx,
                candidate_link_idx
            )

        return current_nearest

    def get_published_link_type(
        self,
        current_link,
        current_segment_index,
        current_segment_ratio
    ):
        """연속 구간 보정, 링크 끝 보정, 교차로 미리보기 순으로 적용한다."""
        current_link_idx = current_link.get('idx')
        current_link_idx = (
            '' if current_link_idx is None else str(current_link_idx)
        )
        section_ranges = self.link_type_section_ranges_by_link_idx.get(
            current_link_idx, ()
        )

        if section_ranges:
            progress = self.lane_change_candidate_generator.get_link_progress_at_segment(
                current_link, current_segment_index, current_segment_ratio
            )

            if progress is not None:
                for link_type, start_progress, end_progress in section_ranges:
                    if start_progress - 1e-6 <= progress <= end_progress + 1e-6:
                        return link_type

        end_override = LINK_TYPE_END_OVERRIDE_BY_LINK_IDX.get(
            current_link_idx
        )

        if end_override is not None:
            override_link_type, preview_distance_m = end_override

            if self.is_near_link_end(
                current_link,
                current_segment_index,
                current_segment_ratio,
                preview_distance_m
            ):
                return override_link_type

        current_link_type = current_link.get('link_type')
        current_link_type = (
            '' if current_link_type is None else str(current_link_type).strip()
        )

        if current_link_type != '6':
            return current_link_type

        next_link = self.lane_change_candidate_generator.find_next_forward_link(
            current_link
        )

        if (
            next_link is None
            or str(next_link.get('link_type', '')).strip() != '1'
        ):
            return current_link_type

        if not self.is_near_link_end(
            current_link,
            current_segment_index,
            current_segment_ratio,
            DEFAULT_INTERSECTION_PREVIEW_DISTANCE_M
        ):
            return current_link_type

        return '1'

    def get_published_link_information(
        self,
        current_link,
        current_segment_index,
        current_segment_ratio
    ):
        """지정된 신호등 링크는 끝점 부근에서만 link_information을 발행한다."""
        link_idx = current_link.get('idx')
        link_idx = '' if link_idx is None else str(link_idx)
        link_information = LINK_INFORMATION_BY_LINK_IDX.get(link_idx, '')
        preview_distance_m = (
            DEFAULT_LINK_INFORMATION_PREVIEW_DISTANCE_M_BY_LINK_IDX.get(link_idx)
        )

        if preview_distance_m is None:
            return link_information

        if not self.is_near_link_end(
            current_link,
            current_segment_index,
            current_segment_ratio,
            preview_distance_m
        ):
            return ''

        return link_information

    def get_published_not_avoid(
        self,
        current_link,
        current_segment_index,
        current_segment_ratio
    ):
        """전 구간 또는 링크 끝 지정 거리 이내에서 회피 금지를 발행한다."""
        link_idx = current_link.get('idx')
        link_idx = '' if link_idx is None else str(link_idx)

        if link_idx in NOT_AVOID_LINK_IDXS:
            return True

        preview_distance_m = NOT_AVOID_PREVIEW_DISTANCE_M_BY_LINK_IDX.get(
            link_idx
        )

        if preview_distance_m is None:
            return False

        return self.is_near_link_end(
            current_link,
            current_segment_index,
            current_segment_ratio,
            preview_distance_m
        )

    def get_lane_change_permissions(
        self,
        current_link,
        current_segment_index,
        current_segment_ratio
    ):
        """링크 속성과 현재 진행거리를 반영한 좌우 차선 변경 가능 여부."""
        permissions = {
            side: bool(
                current_link.get('can_move_%s_lane' % side, False)
            )
            for side in ('left', 'right')
        }
        current_progress = (
            self.lane_change_candidate_generator.get_link_progress_at_segment(
                current_link,
                current_segment_index,
                current_segment_ratio
            )
        )
        link_idx = current_link.get('idx')
        link_idx = '' if link_idx is None else str(link_idx)

        for side in ('left', 'right'):
            if not self.is_lane_change_allowed_at_progress(
                link_idx,
                side,
                current_progress
            ):
                permissions[side] = False

        return permissions

    def is_lane_change_allowed_at_progress(
        self,
        link_idx,
        side,
        current_progress
    ):
        minimum_progress_m = (
            LANE_CHANGE_MIN_PROGRESS_M_BY_LINK_AND_SIDE.get(
                (str(link_idx), str(side))
            )
        )

        if minimum_progress_m is None:
            return True

        if current_progress is None:
            return False

        try:
            current_progress = float(current_progress)
        except (TypeError, ValueError):
            return False

        return (
            math.isfinite(current_progress)
            and current_progress >= minimum_progress_m
        )

    def filter_lane_change_candidates_by_progress(
        self,
        result,
        current_link_idx,
        current_progress
    ):
        filtered_candidates = [
            candidate
            for candidate in result.get('candidates', [])
            if self.is_lane_change_allowed_at_progress(
                current_link_idx,
                candidate.get('side'),
                current_progress
            )
        ]

        if len(filtered_candidates) == len(result.get('candidates', [])):
            return result

        filtered_result = dict(result)
        filtered_result['candidates'] = filtered_candidates
        return filtered_result

    def is_near_link_end(
        self,
        link,
        segment_index,
        segment_ratio,
        distance_m
    ):
        """링크 중심선을 따라 끝점까지 남은 거리가 기준 이내인지 확인한다."""
        points = link.get('points', [])

        try:
            segment_index = int(segment_index)
            segment_ratio = float(segment_ratio)
            distance_m = float(distance_m)
        except (TypeError, ValueError):
            return False

        if (
            len(points) < 2
            or segment_index < 0
            or segment_index >= len(points) - 1
            or not math.isfinite(segment_ratio)
            or not math.isfinite(distance_m)
            or distance_m < 0.0
        ):
            return False

        segment_ratio = max(0.0, min(1.0, segment_ratio))
        remaining_distance_m = 0.0

        for point_i in range(segment_index, len(points) - 1):
            point_a = points[point_i]
            point_b = points[point_i + 1]
            segment_length_m = math.hypot(
                float(point_b['utm_x']) - float(point_a['utm_x']),
                float(point_b['utm_y']) - float(point_a['utm_y'])
            )

            if point_i == segment_index:
                segment_length_m *= 1.0 - segment_ratio

            remaining_distance_m += segment_length_m

            if remaining_distance_m > distance_m:
                return False

        return True

    def find_current_link(self, utm_x, utm_y, yaw=None):
        excluded_link_idxs = (
            self.get_current_link_search_excluded_link_idxs(
                utm_x,
                utm_y
            )
        )
        allowed_link_idxs = [
            link_idx
            for link_idx in self.current_link_allowed_link_idxs
            if str(link_idx) not in excluded_link_idxs
        ]
        allowed_link_idx_set = set(allowed_link_idxs)

        return self.find_link_from_allowed_links(
            utm_x,
            utm_y,
            yaw,
            self.current_link_idx,
            allowed_link_idxs,
            allowed_link_idx_set
        )

    def get_current_link_search_excluded_link_idxs(self, utm_x, utm_y):
        """진행거리 제한 구간에서는 해당 방향의 목적 링크를 검색에서 뺀다."""
        excluded_link_idxs = set()

        for (
            source_link_idx,
            side
        ) in LANE_CHANGE_MIN_PROGRESS_M_BY_LINK_AND_SIDE:
            source_link = self.planner.link_by_idx.get(source_link_idx)

            if source_link is None:
                continue

            source_nearest = self.planner.find_nearest_link_and_segment(
                float(utm_x),
                float(utm_y),
                candidate_link_idxs=[source_link_idx]
            )

            if (
                source_nearest is None
                or source_nearest['distance'] > (
                    CURRENT_LINK_REACQUIRE_DISTANCE_M
                )
            ):
                continue

            source_progress = (
                self.lane_change_candidate_generator
                .get_link_progress_at_segment(
                    source_link,
                    source_nearest['segment_index'],
                    source_nearest['segment_ratio']
                )
            )

            if self.is_lane_change_allowed_at_progress(
                source_link_idx,
                side,
                source_progress
            ):
                continue

            destination_link_idx = source_link.get(
                '%s_lane_change_dst_link_idx' % side
            )

            if destination_link_idx is not None:
                excluded_link_idxs.add(str(destination_link_idx))

        return excluded_link_idxs

    def find_lane_change_candidate_link_current_link(
        self,
        utm_x,
        utm_y,
        yaw=None
    ):
        return self.find_link_from_allowed_links(
            utm_x,
            utm_y,
            yaw,
            self.lane_change_candidate_link_current_link_idx,
            self.lane_change_candidate_link_allowed_link_idxs,
            self.lane_change_candidate_link_allowed_link_idx_set
        )

    def find_link_from_allowed_links(
        self,
        utm_x,
        utm_y,
        yaw,
        current_link_idx,
        allowed_link_idxs,
        allowed_link_idx_set
    ):
        utm_x = float(utm_x)
        utm_y = float(utm_y)

        if current_link_idx is None:
            return self.planner.find_nearest_link_and_segment(
                utm_x,
                utm_y,
                yaw=yaw,
                candidate_link_idxs=allowed_link_idxs
            )

        candidate_link_idxs = self.make_link_search_candidate_link_idxs(
            current_link_idx,
            allowed_link_idx_set
        )
        nearest = self.planner.find_nearest_link_and_segment(
            utm_x,
            utm_y,
            yaw=yaw,
            candidate_link_idxs=candidate_link_idxs
        )

        should_reacquire = (
            nearest is None
            or nearest['distance'] > CURRENT_LINK_REACQUIRE_DISTANCE_M
            or (
                yaw is not None
                and not nearest['used_heading_filter']
            )
        )

        if should_reacquire:
            return self.planner.find_nearest_link_and_segment(
                utm_x,
                utm_y,
                yaw=yaw,
                candidate_link_idxs=allowed_link_idxs
            )

        return nearest

    def make_current_link_candidate_link_idxs(self):
        return self.make_link_search_candidate_link_idxs(
            self.current_link_idx,
            self.current_link_allowed_link_idx_set
        )

    def make_link_search_candidate_link_idxs(
        self,
        current_link_idx,
        allowed_link_idx_set
    ):
        current_link_idx = str(current_link_idx)
        current_link = self.planner.link_by_idx.get(current_link_idx)

        if (
            current_link is None
            or current_link_idx not in allowed_link_idx_set
        ):
            return []

        candidate_link_idxs = []
        candidate_link_idx_set = set()
        next_link_idxs = []
        next_link_idx_set = set()

        def add_candidate_link_idx(link_idx):
            if link_idx is None:
                return

            link_idx = str(link_idx)

            if (
                link_idx not in allowed_link_idx_set
                or link_idx in candidate_link_idx_set
            ):
                return

            candidate_link_idx_set.add(link_idx)
            candidate_link_idxs.append(link_idx)

        add_candidate_link_idx(current_link_idx)

        for node_key in ('from_node_idx', 'to_node_idx'):
            node_idx = current_link.get(node_key)

            if node_idx is None:
                continue

            for connected_link_idx in self.planner.links_by_from_node.get(
                str(node_idx),
                []
            ):
                connected_link_idx = str(connected_link_idx)
                add_candidate_link_idx(connected_link_idx)

                if (
                    node_key == 'to_node_idx'
                    and connected_link_idx
                    in allowed_link_idx_set
                    and connected_link_idx not in next_link_idx_set
                ):
                    next_link_idx_set.add(connected_link_idx)
                    next_link_idxs.append(connected_link_idx)

        lane_change_base_link_idxs = [current_link_idx]
        lane_change_base_link_idxs.extend(next_link_idxs)

        for base_link_idx in lane_change_base_link_idxs:
            lane_change_link_idxs = (
                self.planner.get_lane_change_link_closure(
                    [base_link_idx],
                    max_lane_change_count=(
                        CURRENT_LINK_MAX_LANE_CHANGE_COUNT
                    )
                )
            )

            for lane_change_link_idx in lane_change_link_idxs:
                add_candidate_link_idx(lane_change_link_idx)

        return candidate_link_idxs

    def publish_lane_change_candidate_data(self, _event):
        context = self.latest_lane_change_candidate_context
        is_active = context is not None

        if not is_active:
            if self.lane_change_candidate_data_active:
                self.publish_lane_change_candidate_cloud({
                    'current_link_idx': None,
                    'next_link_idx': None,
                    'candidates': []
                })
                self.lane_change_candidate_data_active = False
            return

        with self.lane_change_candidate_lock:
            result, _ = self.make_lane_change_candidate_result(
                'data',
                context['link_idx'],
                context['rviz_x'],
                context['rviz_y'],
                context['segment_index'],
                context['segment_ratio'],
                force_update=True
            )
            self.publish_lane_change_candidate_cloud(result)

        self.lane_change_candidate_data_active = True

    def publish_lane_change_candidate_links(
        self,
        link,
        nearest=None,
        lane_change_permissions=None
    ):
        """current_link_data와 같은 보정된 링크에서 변경 가능한 옆 링크를 발행한다."""
        result = {
            'current_link_idx': None if link is None else link.get('idx'),
            'candidates': []
        }
        if link is not None:
            for side in ('left', 'right'):
                if (
                    lane_change_permissions is not None
                    and not lane_change_permissions.get(side, False)
                ):
                    continue
                if not bool(link.get('can_move_%s_lane' % side, False)):
                    continue
                dst_idx = link.get('%s_lane_change_dst_link_idx' % side)
                if dst_idx is None or str(dst_idx) == str(link.get('idx')):
                    continue
                dst_idx = str(dst_idx)
                if len(self.lane_change_candidate_link_points_by_idx.get(dst_idx, ())) < 2:
                    continue
                destination_link_idxs = [dst_idx]
                generator = self.lane_change_candidate_generator
                # 후보 경로를 만들지 않고 기존 목적 링크 연장 규칙만 재사용한다.
                # 현재 투영점에서 전방 길이가 부족하면 연결된 다음 링크를 포함한다.
                point = link['points'][0] if nearest is None else nearest['point']
                with self.lane_change_candidate_lock:
                    geometry_result = generator.make_destination_geometry(
                        link,
                        self.planner.link_by_idx[dst_idx],
                        side,
                        {'point': point},
                        generator.candidate_path_length_m
                    )
                if geometry_result is not None:
                    destination_link_idxs = [
                        str(dst_link['idx'])
                        for dst_link in geometry_result[3]
                    ]
                result['candidates'].append({
                    'side': side,
                    'destination_link_idx': dst_idx,
                    'destination_link_idxs': destination_link_idxs
                })

        # 동일 링크 구성은 latch로 유지하고, 변경 불가/링크 손실 시 빈 cloud로 지운다.
        self.publish_lane_change_candidate_link_cloud(result)
        return result

    def make_lane_change_candidate_result(
        self,
        topic_key,
        current_link_idx,
        current_rviz_x,
        current_rviz_y,
        current_segment_index=None,
        current_segment_ratio=None,
        force_update=False
    ):
        link_idx = '' if current_link_idx is None else str(current_link_idx)

        if self.lane_change_candidate_generator is None:
            return {
                'current_link_idx': link_idx,
                'next_link_idx': None,
                'candidates': []
            }, False

        current_link = self.lane_change_candidate_generator.link_by_idx.get(
            link_idx
        )
        current_progress = None

        if current_link is not None:
            current_progress = (
                self.lane_change_candidate_generator.get_link_progress_at_segment(
                    current_link,
                    current_segment_index,
                    current_segment_ratio
                )
            )

        if current_link is not None and current_progress is None:
            current_progress = (
                self.lane_change_candidate_generator.find_nearest_link_progress(
                    current_link,
                    float(current_rviz_x),
                    float(current_rviz_y)
                )
            )

        def cache_matches(candidate_cache):
            progress_is_unchanged = (
                current_progress is None
                and candidate_cache['progress_m'] is None
            ) or (
                current_progress is not None
                and candidate_cache['progress_m'] is not None
                and abs(
                    current_progress - candidate_cache['progress_m']
                ) < LANE_CHANGE_CANDIDATE_UPDATE_DISTANCE_M
            )
            return (
                link_idx == candidate_cache['link_idx']
                and progress_is_unchanged
                and candidate_cache['result'] is not None
            )

        cache = self.lane_change_candidate_cache_by_topic[topic_key]

        if not force_update and cache_matches(cache):
            return cache['result'], False

        if not force_update:
            for other_topic_key, other_cache in (
                self.lane_change_candidate_cache_by_topic.items()
            ):
                if (
                    other_topic_key == topic_key
                    or not cache_matches(other_cache)
                ):
                    continue

                cache['link_idx'] = other_cache['link_idx']
                cache['progress_m'] = other_cache['progress_m']
                cache['result'] = other_cache['result']
                return cache['result'], True

        result = self.lane_change_candidate_generator.make_candidates(
            link_idx,
            float(current_rviz_x),
            float(current_rviz_y),
            current_progress=current_progress
        )
        result = self.filter_lane_change_candidates_by_progress(
            result,
            link_idx,
            current_progress
        )
        cache['link_idx'] = link_idx
        cache['progress_m'] = current_progress
        cache['result'] = result
        return result, True

    def publish_current_link(
        self,
        link_idx,
        link_type,
        max_speed,
        on_stop_line,
        traffic_light_id,
        related_signal,
        can_change_left,
        can_change_right,
        not_avoid,
        link_information
    ):
        msg = CurrentLink()
        msg.link_idx = '' if link_idx is None else str(link_idx)
        msg.not_avoid = bool(not_avoid)
        msg.link_type = '' if link_type is None else str(link_type)
        msg.max_speed = self.to_float_or_default(max_speed, 0.0)
        msg.on_stop_line = bool(on_stop_line)
        msg.traffic_light_id = '' if traffic_light_id is None else str(traffic_light_id)
        msg.related_signal = '' if related_signal is None else str(related_signal)
        msg.can_change_left = bool(can_change_left)
        msg.can_change_right = bool(can_change_right)
        msg.link_information = (
            '' if link_information is None else str(link_information)
        )
        self.current_link_pub.publish(msg)

    def to_float_or_default(self, value, default):
        if value is None:
            return float(default)

        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def make_link_payload(self, link):
        return {
            'idx': link.get('idx'),
            'from_node_idx': link.get('from_node_idx'),
            'to_node_idx': link.get('to_node_idx'),
            'points': [self.make_point_payload(point) for point in link['points']],
            'max_speed': link.get('max_speed'),
            'link_length': link.get('link_length'),
            'width_start': link.get('width_start'),
            'width_end': link.get('width_end'),
            'lane_mark_left': link.get('lane_mark_left', []),
            'lane_mark_right': link.get('lane_mark_right', []),
            'can_move_left_lane': bool(link.get('can_move_left_lane', False)),
            'can_move_right_lane': bool(link.get('can_move_right_lane', False)),
            'left_lane_change_dst_link_idx': link.get('left_lane_change_dst_link_idx'),
            'right_lane_change_dst_link_idx': link.get('right_lane_change_dst_link_idx'),
            'road_id': link.get('road_id'),
            'ego_lane': link.get('ego_lane'),
            'link_type': link.get('link_type'),
            'related_signal': link.get('related_signal')
        }

    def make_node_payload(self, node):
        return {
            'idx': node.get('idx'),
            'node_type': node.get('node_type'),
            'on_stop_line': node['on_stop_line'],
            'traffic_light_id': node.get('traffic_light_id'),
            'rviz_x': node['rviz_x'],
            'rviz_y': node['rviz_y'],
            'utm_x': node['utm_x'],
            'utm_y': node['utm_y'],
            'morai_x': node['morai_x'],
            'morai_y': node['morai_y'],
            'morai_z': node['morai_z'],
            'z': node['morai_z']
        }

    def make_point_payload(self, point):
        return {
            'rviz_x': point['rviz_x'],
            'rviz_y': point['rviz_y'],
            'utm_x': point['utm_x'],
            'utm_y': point['utm_y'],
            'morai_x': point['morai_x'],
            'morai_y': point['morai_y'],
            'morai_z': point['morai_z'],
            'z': point['morai_z']
        }

    def dumps(self, payload):
        return json.dumps(payload, ensure_ascii=False, separators=(',', ':'))


def main():
    rospy.init_node('morai_map')
    map_publisher = MoraiMapPublisher()
    rospy.spin()
    return map_publisher


if __name__ == '__main__':
    main()
