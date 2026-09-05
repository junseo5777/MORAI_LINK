#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
from pathlib import Path

import rospy

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from path_planning.link.morai_link_planner import MoraiMapPlanner
from path_planning.link.lane_change_candidate import LaneChangeCandidateGenerator
from path_planning.link.reverse_link import ReverseLinkTargetFinder
from path_planning.link.virtual_lane_boundary import VirtualLaneBoundaryGenerator
from path_planning.link.dynamic_obstacle_link_projector import DynamicObstacleLinkProjector
from macaron_8.msg import (
    CurrentLink,
    DynamicObstacleLink,
    DynamicObstacleLinkArray,
    FusedObjectArray,
)


# reverse_link.py 관련 기본 설정
DEFAULT_REVERSE_TARGET_DISTANCE_M = 8.0
DEFAULT_REVERSE_TARGET_PUBLISH_HZ = 10.0 # hz

# morai_link_planner.py 관련 기본 설정
CURRENT_LINK_REACQUIRE_DISTANCE_M = 4.0  # 기존 링크에서 이 이상 멀어지면 전체 후보에서 링크 재탐색 [m]
CURRENT_LINK_MAX_LANE_CHANGE_COUNT = 3  # 현재 링크 및 후보 경로 확장에 허용할 최대 좌우 차선 이동 횟수
DEFAULT_CURRENT_LINK_CHANGE_CONFIRM_COUNT = 10  # 다른 링크로 전환하기 위한 연속 관측 횟수
DEFAULT_INTERSECTION_PREVIEW_DISTANCE_M = 10.0  # 다음 링크가 교차로이면 이 거리 전부터 link_type=1 발행 [m]

# lane_change_candidate.py 관련 기본 설정
LANE_CHANGE_CANDIDATE_UPDATE_DISTANCE_M = 1.0  # 후보 경로를 다시 생성하는 차량 진행간격 [m]
LANE_CHANGE_CANDIDATE_START_AHEAD_M = 0.0  # 차량 투영점에서 후보 경로 시작점까지의 전방거리 [m]
LANE_CHANGE_CANDIDATE_ID_BY_SIDE = {
    'center': 0,
    'left': 1,
    'right': 2
}

# virtual_lane_boundary.py 관련 기본 설정
VIRTUAL_CENTER_LINE_LANE_WIDTH_M = 2.3 # 가상 좌측 중앙선 생성 시 기준 차선 폭 [m]

# dynamic_obstacle_link_projector.py 관련 기본 설정
DYNAMIC_OBSTACLE_MAX_DISTANCE_M = 50.0
DYNAMIC_OBSTACLE_MAX_HEADING_ERROR_DEG = 45.0
DYNAMIC_OBSTACLE_REVERSE_HEADING_TOLERANCE_DEG = 30.0
DYNAMIC_OBSTACLE_MAX_LANE_CHANGE_COUNT = 2


INTERSECTION_TYPE_BY_LINK_IDX = {
    'A2256W000219': 'three_light_traffic_signal',
    'A2256W000213': 'three_light_traffic_signal',
    'A2256W000202': 'four_light_traffic_signal',
    'A2256W000728': 'four_light_traffic_signal',
    'A2256W000315': 'four_light_traffic_signal',
    'A2256W000310': 'four_light_traffic_signal',
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

DEFAULT_INTERSECTION_TYPE_PREVIEW_DISTANCE_M_BY_LINK_IDX = {
    'A2256W000213': 25.0,
    'A2256W000308': 20.0,
    'A2256W000315': 20.0
}

VIRTUAL_LEFT_CENTER_LINE_LINK_IDX_GROUPS = (
    (
        'A2256W000304',
        'A2256W000148'
    ),
    (
        'A2256W000213',
    )
)

CURRENT_LINK_ROUTE_LINK_IDXS = [
    'A2256W000748',
    'A2256W000182',
    'A2256W000329',
    'A2256W000236',
    'A2256W000219',
    'A2256W000215',
    'A2256W000213',
    'A2256W000222',
    'A2256W000220',
    'A2256W000202',
    'A2256W000728',
    'A2256W000599',
    'A2256W000207',
    'A2256W000205',
    'A2256W000315',
    'A2256W000318',
    'A2256W000308',
    'A2256W000083',
    'A2256W000304',
    'A2256W000148',
    'A2256W000146',
    'A2256W000151',
    'A2256W000866',
    'A2256W000054',
    'A2256W000846',
    'A2256W000411',
    'A2256W000420',
    'A2256W000408',
    'A2256W000445',
    'A2256W000153',
    'A2256W000451',
    'A2256W000446',
    'A2256W000448',
    'A2256W000126',
    'A2256W000128',
    'A2256W000333',
    'A2256W000154',
    'A2256W000332',
    'A2256W000751',
    'A2256W000748'
]


class MoraiMapPublisher:
    def __init__(self):
        self.link_path = self.default_map_path('link_set.json')
        self.node_path = self.default_map_path('node_set.json')
        self.origin_utm_x = 302595.0
        self.origin_utm_y = 4124145.0
        self.use_utm_offset = True  #True면 rviz 띄울 때 origin 좌표를 빼고 띄움

        # 구독 토픽
        self.current_pose_topic = '/current_pose'
        self.dynamic_obstacle_input_topic = (
            '/yolo_lidar_fusion/dynamic_objects'
        )

        # 발행 토픽
        self.map_data_topic = '/morai_map_data'
        self.current_link_topic = '/current_link_data'
        self.reverse_target_topic = '/reverse_link_point'
        self.lane_change_candidate_data_topic = '/lane_change_candidate_data'
        self.lane_change_candidate_link_data_topic = (
            '/lane_change_candidate_link'
        )
        self.virtual_lane_boundary_topic = '/current_link_virtual_lane_boundaries'
        self.dynamic_obstacle_link_data_topic = (
            '/dynamic_obstacle_link_data'
        )

        self.heading_filter_deg = 90.0  # 현재 링크 검색 시 허용할 차량-링크 방향 오차 [deg]

        self.reverse_target_distance_m = self.positive_float_param(
            '~reverse_target_distance_m',
            DEFAULT_REVERSE_TARGET_DISTANCE_M
        )
        self.reverse_target_publish_hz = self.positive_float_param(
            '~reverse_target_publish_hz',
            DEFAULT_REVERSE_TARGET_PUBLISH_HZ
        )
        self.intersection_preview_distance_m = self.positive_float_param(
            '~intersection_preview_distance_m',
            DEFAULT_INTERSECTION_PREVIEW_DISTANCE_M
        )
        self.current_link_change_confirm_count = self.bounded_int_param(
            '~current_link_change_confirm_count',
            DEFAULT_CURRENT_LINK_CHANGE_CONFIRM_COUNT,
            1,
            1000
        )
        self.intersection_type_preview_distance_m_by_link_idx = {
            link_idx: self.positive_float_param(
                '~intersection_type_preview_distance_m/%s' % link_idx,
                default_distance_m
            )
            for link_idx, default_distance_m in (
                DEFAULT_INTERSECTION_TYPE_PREVIEW_DISTANCE_M_BY_LINK_IDX.items()
            )
        }
        self.dynamic_obstacle_max_distance_m = self.positive_float_param(
            '~dynamic_obstacle_max_distance_m',
            DYNAMIC_OBSTACLE_MAX_DISTANCE_M
        )
        self.dynamic_obstacle_max_heading_error_deg = (
            self.bounded_float_param(
                '~dynamic_obstacle_max_heading_error_deg',
                DYNAMIC_OBSTACLE_MAX_HEADING_ERROR_DEG,
                0.0,
                180.0
            )
        )
        self.dynamic_obstacle_reverse_heading_tolerance_deg = (
            self.bounded_float_param(
                '~dynamic_obstacle_reverse_heading_tolerance_deg',
                DYNAMIC_OBSTACLE_REVERSE_HEADING_TOLERANCE_DEG,
                0.0,
                180.0
            )
        )
        self.dynamic_obstacle_max_lane_change_count = (
            self.bounded_int_param(
                '~dynamic_obstacle_max_lane_change_count',
                DYNAMIC_OBSTACLE_MAX_LANE_CHANGE_COUNT,
                0,
                DYNAMIC_OBSTACLE_MAX_LANE_CHANGE_COUNT
            )
        )
        self.map_data_pub = rospy.Publisher(
            self.map_data_topic,
            String,
            queue_size=1,
            latch=True
        )
        self.current_link_pub = rospy.Publisher(
            self.current_link_topic,
            CurrentLink,
            queue_size=10
        )
        self.reverse_target_pub = rospy.Publisher(
            self.reverse_target_topic,
            Point,
            queue_size=10
        )
        self.lane_change_candidate_data_pub = rospy.Publisher(
            self.lane_change_candidate_data_topic,
            PointCloud2,
            queue_size=1,
            latch=True
        )
        self.lane_change_candidate_link_data_pub = rospy.Publisher(
            self.lane_change_candidate_link_data_topic,
            PointCloud2,
            queue_size=1,
            latch=True
        )
        self.virtual_lane_boundary_pub = rospy.Publisher(
            self.virtual_lane_boundary_topic,
            PointCloud2,
            queue_size=1,
            latch=True
        )
        self.dynamic_obstacle_link_data_pub = rospy.Publisher(
            self.dynamic_obstacle_link_data_topic,
            DynamicObstacleLinkArray,
            queue_size=1
        )

        """
        버스 전용 도로

        link_idx: "A2256W000362"
        max_speed: 50.0
        on_stop_line: True
        traffic_light_id: "C1256W000054"
        related_signal: ''

        link_idx: "A2256W000368"
        max_speed: 50.0
        on_stop_line: False
        traffic_light_id: ''
        related_signal: "straight"

        link_idx: "A2256W000367"
        max_speed: 45.0
        on_stop_line: True
        traffic_light_id: "C1256W000066"
        related_signal: ''
        """

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
        self.dynamic_obstacle_link_projector = None
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
                target_link_idx_groups=VIRTUAL_LEFT_CENTER_LINE_LINK_IDX_GROUPS,
                lane_width_m=VIRTUAL_CENTER_LINE_LANE_WIDTH_M
            )
        )
        self.active_virtual_center_line_link_idxs = ()
        self.current_pose_sub = None
        self.dynamic_obstacle_sub = None

        self.load_morai_map()
        self.current_pose_sub = rospy.Subscriber(
            self.current_pose_topic,
            PointStamped,
            self.current_pose_callback,
            queue_size=1
        )
        self.dynamic_obstacle_sub = rospy.Subscriber(
            self.dynamic_obstacle_input_topic,
            FusedObjectArray,
            self.dynamic_obstacles_callback,
            queue_size=1
        )
        self.reverse_target_timer = rospy.Timer(
            rospy.Duration(1.0 / self.reverse_target_publish_hz),
            self.publish_reverse_target
        )
        rospy.loginfo(
            'Reverse-link target enabled: distance=%.2fm, rate=%.1fHz, '
            'topic=%s',
            self.reverse_target_distance_m,
            self.reverse_target_publish_hz,
            self.reverse_target_topic
        )
        rospy.loginfo(
            'Intersection-type preview distances: %s',
            ', '.join(
                '%s=%.2fm' % (link_idx, distance_m)
                for link_idx, distance_m in (
                    self.intersection_type_preview_distance_m_by_link_idx.items()
                )
            )
        )
        rospy.loginfo(
            'Current-link change confirmation count: %d',
            self.current_link_change_confirm_count
        )
        rospy.loginfo(
            'Dynamic obstacle link projector enabled: '
            'input=%s output=%s range=%.1fm heading=%.1fdeg '
            'lane_changes=%d',
            self.dynamic_obstacle_input_topic,
            self.dynamic_obstacle_link_data_topic,
            self.dynamic_obstacle_max_distance_m,
            self.dynamic_obstacle_max_heading_error_deg,
            self.dynamic_obstacle_max_lane_change_count
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
            route_link_idxs=CURRENT_LINK_ROUTE_LINK_IDXS
        )
        self.build_lane_change_candidate_link_point_cache()
        self.build_current_link_allowed_link_idxs()
        self.planner.build_nearest_lookup(
            self.current_link_allowed_link_idxs
        )
        self.lane_change_candidate_generator = LaneChangeCandidateGenerator(
            links=self.planner.links,
            nodes=self.planner.nodes,
            start_ahead_of_vehicle_m=LANE_CHANGE_CANDIDATE_START_AHEAD_M,
            route_link_idxs=CURRENT_LINK_ROUTE_LINK_IDXS,
            lane_change_extension_link_idxs=(
                self.lane_change_extension_link_idxs
            )
        )
        self.dynamic_obstacle_link_projector = DynamicObstacleLinkProjector(
            planner=self.planner,
            route_link_idxs=CURRENT_LINK_ROUTE_LINK_IDXS,
            allowed_link_idxs=self.current_link_allowed_link_idxs,
            max_lane_change_count=(
                self.dynamic_obstacle_max_lane_change_count
            ),
            max_obstacle_distance_m=(
                self.dynamic_obstacle_max_distance_m
            ),
            max_heading_error_deg=(
                self.dynamic_obstacle_max_heading_error_deg
            ),
            reverse_heading_tolerance_deg=(
                self.dynamic_obstacle_reverse_heading_tolerance_deg
            )
        )
        self.publish_map_data()
        rospy.loginfo('Loaded MORAI map links: %d', len(self.planner.links))
        rospy.loginfo('Loaded MORAI map nodes: %d', len(self.planner.nodes))
        rospy.loginfo(
            'Built nearest lookup segments: %d',
            len(self.planner.segment_refs)
        )

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
        return [
            PointField(
                name=name,
                offset=field_i * 8,
                datatype=PointField.FLOAT64,
                count=1
            )
            for field_i, name in enumerate(names)
        ]

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

        for candidate in result.get('candidates', []):
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
            (point['utm_x'], point['utm_y'])
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
            'Published virtual left center line: '
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

    def dynamic_obstacles_callback(self, msg):
        results = self.dynamic_obstacle_link_projector.project_obstacles(
            obstacles=msg.objects,
            current_pose=self.current_pose,
            current_link_idx=self.current_link_idx
        )
        output = DynamicObstacleLinkArray()
        output.header = msg.header
        output.objects = [
            self.make_dynamic_obstacle_link_message(result)
            for result in results
        ]
        self.dynamic_obstacle_link_data_pub.publish(output)

    def make_dynamic_obstacle_link_message(self, result):
        msg = DynamicObstacleLink()
        msg.track_id = result['track_id']
        msg.nearest_link_idx = result['nearest_link_idx']

        projected_position = result['projected_position_utm']
        msg.projected_position_utm.x = projected_position['x']
        msg.projected_position_utm.y = projected_position['y']
        msg.projected_position_utm.z = projected_position['z']
        msg.ego_distance_m = result['ego_distance_m']
        msg.signed_link_distance_m = result['signed_link_distance_m']
        msg.signed_heading_error_rad = result[
            'signed_heading_error_rad'
        ]
        return msg

    def publish_reverse_target(self, _event):
        current_pose = self.current_pose
        current_link_idx = self.current_link_idx

        if (
            current_pose is None
            or current_link_idx is None
        ):
            return

        target = self.reverse_link_finder.find_target(
            vehicle_x=current_pose[0],
            vehicle_y=current_pose[1],
            current_link_idx=current_link_idx,
            target_distance_m=self.reverse_target_distance_m
        )

        if target is None:
            rospy.logwarn_throttle(
                2.0,
                'Unable to find %.2fm reverse target from link %s',
                self.reverse_target_distance_m,
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

    def publish_current_link_info(self, utm_x, utm_y, yaw=None):
        nearest = self.find_current_link(utm_x, utm_y, yaw)
        nearest = self.confirm_current_link_candidate(
            nearest,
            utm_x,
            utm_y,
            yaw
        )

        if nearest is None:
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

        candidate_result = self.publish_lane_change_candidate_data(
            link_idx,
            nearest['point']['rviz_x'],
            nearest['point']['rviz_y'],
            current_segment_index=nearest['segment_index'],
            current_segment_ratio=nearest['segment_ratio']
        )
        self.publish_lane_change_candidate_link_for_pose(
            utm_x,
            utm_y,
            yaw
        )
        candidates = candidate_result.get('candidates', [])
        can_change_left = any(
            candidate.get('side') == 'left'
            for candidate in candidates
        )
        can_change_right = any(
            candidate.get('side') == 'right'
            for candidate in candidates
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
            self.get_published_intersection_type(
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
            >= self.current_link_change_confirm_count
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
            self.current_link_change_confirm_count
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
        """다음 교차로 링크 진입 전부터 link_type을 1로 발행한다."""
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
            self.intersection_preview_distance_m
        ):
            return current_link_type

        return '1'

    def get_published_intersection_type(
        self,
        current_link,
        current_segment_index,
        current_segment_ratio
    ):
        """지정된 신호등 링크는 끝점 부근에서만 intersection_type을 발행한다."""
        link_idx = current_link.get('idx')
        link_idx = '' if link_idx is None else str(link_idx)
        intersection_type = INTERSECTION_TYPE_BY_LINK_IDX.get(link_idx, '')
        preview_distance_m = (
            self.intersection_type_preview_distance_m_by_link_idx.get(link_idx)
        )

        if preview_distance_m is None:
            return intersection_type

        if not self.is_near_link_end(
            current_link,
            current_segment_index,
            current_segment_ratio,
            preview_distance_m
        ):
            return ''

        return intersection_type

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
        return self.find_link_from_allowed_links(
            utm_x,
            utm_y,
            yaw,
            self.current_link_idx,
            self.current_link_allowed_link_idxs,
            self.current_link_allowed_link_idx_set
        )

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

    def publish_lane_change_candidate_data(
        self,
        current_link_idx,
        current_rviz_x,
        current_rviz_y,
        current_segment_index=None,
        current_segment_ratio=None
    ):
        result, is_updated = self.make_lane_change_candidate_result(
            'data',
            current_link_idx,
            current_rviz_x,
            current_rviz_y,
            current_segment_index,
            current_segment_ratio
        )

        if is_updated:
            self.publish_lane_change_candidate_cloud(result)

        return result

    def publish_lane_change_candidate_link_for_pose(
        self,
        utm_x,
        utm_y,
        yaw=None
    ):
        nearest = self.find_lane_change_candidate_link_current_link(
            utm_x,
            utm_y,
            yaw
        )

        if nearest is None:
            self.lane_change_candidate_link_current_link_idx = None
            result, is_updated = self.make_lane_change_candidate_result(
                'link',
                None,
                0.0,
                0.0
            )
        else:
            link = nearest['link']
            link_idx = link.get('idx')
            self.lane_change_candidate_link_current_link_idx = (
                None if link_idx is None else str(link_idx)
            )
            result, is_updated = self.make_lane_change_candidate_result(
                'link',
                link_idx,
                nearest['point']['rviz_x'],
                nearest['point']['rviz_y'],
                nearest['segment_index'],
                nearest['segment_ratio']
            )

        if is_updated:
            self.publish_lane_change_candidate_link_cloud(result)

        return result

    def make_lane_change_candidate_result(
        self,
        topic_key,
        current_link_idx,
        current_rviz_x,
        current_rviz_y,
        current_segment_index=None,
        current_segment_ratio=None
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

        if cache_matches(cache):
            return cache['result'], False

        for other_topic_key, other_cache in (
            self.lane_change_candidate_cache_by_topic.items()
        ):
            if other_topic_key == topic_key or not cache_matches(other_cache):
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
        intersection_type
    ):
        msg = CurrentLink()
        msg.link_idx = '' if link_idx is None else str(link_idx)
        msg.link_type = '' if link_type is None else str(link_type)
        msg.max_speed = self.to_float_or_default(max_speed, 0.0)
        msg.on_stop_line = bool(on_stop_line)
        msg.traffic_light_id = '' if traffic_light_id is None else str(traffic_light_id)
        msg.related_signal = '' if related_signal is None else str(related_signal)
        msg.can_change_left = bool(can_change_left)
        msg.can_change_right = bool(can_change_right)
        msg.intersection_type = (
            '' if intersection_type is None else str(intersection_type)
        )
        self.current_link_pub.publish(msg)

    def to_float_or_default(self, value, default):
        if value is None:
            return float(default)

        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def positive_float_param(self, name, default):
        value = rospy.get_param(name, default)

        try:
            value = float(value)
        except (TypeError, ValueError):
            rospy.logwarn(
                'Invalid %s=%r; using %.3f',
                name,
                value,
                default
            )
            return float(default)

        if not math.isfinite(value) or value <= 0.0:
            rospy.logwarn(
                'Invalid %s=%r; using %.3f',
                name,
                value,
                default
            )
            return float(default)

        return value

    def bounded_float_param(self, name, default, minimum, maximum):
        value = rospy.get_param(name, default)

        try:
            value = float(value)
        except (TypeError, ValueError):
            rospy.logwarn(
                'Invalid %s=%r; using %.3f',
                name,
                value,
                default
            )
            return float(default)

        if not math.isfinite(value) or not minimum <= value <= maximum:
            rospy.logwarn(
                'Invalid %s=%r; using %.3f',
                name,
                value,
                default
            )
            return float(default)

        return value

    def bounded_int_param(self, name, default, minimum, maximum):
        value = rospy.get_param(name, default)

        try:
            value = int(value)
        except (TypeError, ValueError):
            rospy.logwarn(
                'Invalid %s=%r; using %d',
                name,
                value,
                default
            )
            return int(default)

        if not minimum <= value <= maximum:
            rospy.logwarn(
                'Invalid %s=%r; using %d',
                name,
                value,
                default
            )
            return int(default)

        return value

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
