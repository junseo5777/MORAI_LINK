#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
from pathlib import Path

import rospy
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from morai_link_planner import MoraiMapPlanner
from lane_change_candidate import LaneChangeCandidateGenerator
from macaron_8.msg import CurrentLink


MAP_CREATE = False
LANE_CHANGE_LOOKAHEAD_M = 20.0
CURRENT_LINK_LOOKAHEAD_ROUTE_COUNT = 1
LANE_CHANGE_CANDIDATE_START_AHEAD_M = 10.0
LANE_CHANGE_CANDIDATE_UPDATE_DISTANCE_M = 1.0

FIXED_ROUTE_LINK_IDXS = [
    'A2256W000748',
    'A2256W000329',
    'A2256W000219',
    'A2256W000202',
    'A2256W000308',
    'A2256W000304',
    'A2256W000146',
    'A2256W000846',
    'A2256W000153',
    'A2256W000128',
    'A2256W000332',
    'A2256W000748'
]

ROUTE_NPY_PATH = str(
    Path(__file__).resolve().parents[1]
    / 'path'
    / 'donggul.npy'
)


class MoraiMapPublisher:
    def __init__(self):
        self.link_path = MoraiMapPlanner.default_map_path('link_set.json')
        self.node_path = MoraiMapPlanner.default_map_path('node_set.json')
        self.origin_utm_x = 302595.0
        self.origin_utm_y = 4124145.0
        self.use_utm_offset = True  #True면 rviz 띄울 때 origin 좌표를 빼고 띄움


        self.current_pose_topic = '/current_pose'
        self.map_data_topic = '/morai_map_data'
        self.route_data_topic = '/morai_route_data'
        self.current_link_topic = '/current_link_data'
        self.lane_change_candidate_data_topic = '/lane_change_candidate_data'
        self.map_create = MAP_CREATE
        self.route_npy_path = ROUTE_NPY_PATH

        self.heading_filter_deg = 90.0  #시작 링크를 찾을 때 허용할 오차
        self.lane_change_penalty = 20.0  #차선 변경에 추가되는 패널티
        self.lane_change_lookahead_m = LANE_CHANGE_LOOKAHEAD_M  #차선 변경 목표점을 몇 m 앞에 둘지
        self.current_link_lookahead_route_count = CURRENT_LINK_LOOKAHEAD_ROUTE_COUNT

        self.map_data_pub = rospy.Publisher(
            self.map_data_topic,
            String,
            queue_size=1,
            latch=True
        )
        self.route_data_pub = rospy.Publisher(
            self.route_data_topic,
            String,
            queue_size=1,
            latch=True
        )
        self.current_link_pub = rospy.Publisher(
            self.current_link_topic,
            CurrentLink,
            queue_size=10
        )
        self.lane_change_candidate_data_pub = rospy.Publisher(
            self.lane_change_candidate_data_topic,
            String,
            queue_size=1,
            latch=True
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
        self.route_links = []
        self.link_by_idx = {}
        self.lane_change_candidate_generator = None
        self.last_lane_change_candidate_link_idx = None
        self.last_lane_change_candidate_progress_m = None
        self.last_lane_change_candidate_result = None
        self.current_pose_sub = None

        self.load_morai_map()

        start_pose = self.wait_for_first_current_pose()
        if start_pose is None:
            return

        if self.plan_and_publish_route_data(start_pose):
            self.current_pose_sub = rospy.Subscriber(
                self.current_pose_topic,
                Point,
                self.current_pose_callback,
                queue_size=1
            )

    def load_morai_map(self):
        self.planner = MoraiMapPlanner(
            link_path=self.link_path,
            origin_utm_x=self.origin_utm_x,
            origin_utm_y=self.origin_utm_y,
            node_path=self.node_path,
            use_utm_offset=self.use_utm_offset,
            heading_filter_deg=self.heading_filter_deg,
            lane_change_penalty=self.lane_change_penalty,
            lane_change_lookahead_m=self.lane_change_lookahead_m
        )

        rospy.loginfo('Loading MORAI map link_set: %s', self.link_path)
        rospy.loginfo('Loading MORAI map node_set: %s', self.node_path)
        self.planner.load()
        self.link_by_idx = {
            link.get('idx'): link
            for link in self.planner.links
            if link.get('idx') is not None
        }
        self.lane_change_candidate_generator = LaneChangeCandidateGenerator(
            links=self.planner.links,
            nodes=self.planner.nodes,
            start_ahead_of_vehicle_m=LANE_CHANGE_CANDIDATE_START_AHEAD_M
        )
        self.publish_map_data()
        rospy.loginfo('Loaded MORAI map links: %d', len(self.planner.links))
        rospy.loginfo('Loaded MORAI map nodes: %d', len(self.planner.nodes))
        rospy.loginfo(
            'Built nearest lookup segments: %d',
            len(self.planner.segment_refs)
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

    def wait_for_first_current_pose(self):
        rospy.loginfo(
            'Waiting for first current pose from %s',
            self.current_pose_topic
        )

        try:
            pose = rospy.wait_for_message(self.current_pose_topic, Point)
        except rospy.ROSInterruptException:
            return None

        start_pose = {
            'utm_x': float(pose.x),
            'utm_y': float(pose.y),
            'yaw': float(pose.z)
        }

        rospy.loginfo(
            'Start UTM fixed from first current_pose: x=%.3f, y=%.3f, yaw=%.3f',
            start_pose['utm_x'],
            start_pose['utm_y'],
            start_pose['yaw']
        )

        return start_pose

    def plan_and_publish_route_data(self, start_pose):
        result = self.planner.plan_fixed_link_route(FIXED_ROUTE_LINK_IDXS)

        if not result['success']:
            rospy.logerr('Failed to make global path: %s', result['reason'])
            return False

        self.log_path_result(result)
        self.route_links = result['route_links']
        self.lane_change_candidate_generator.update_route(self.route_links)
        self.last_lane_change_candidate_link_idx = None
        self.last_lane_change_candidate_progress_m = None
        self.last_lane_change_candidate_result = None

        if self.map_create:
            self.save_route_npy(result['path_array'])

        payload = {
            'path': self.make_path_payload(result['path_array']),
            'route_links': [
                self.make_link_payload(link) for link in result['route_links']
            ],
            'start_point': self.make_point_payload(result['start_info']['point']),
            'goal_point': self.make_point_payload(result['goal_info']['point'])
        }

        self.route_data_pub.publish(self.dumps(payload))
        rospy.loginfo(
            'Published route data: route_links=%d, path_points=%d',
            len(payload['route_links']),
            len(payload['path'])
        )
        self.publish_current_link_info(
            start_pose['utm_x'],
            start_pose['utm_y']
        )

        return True

    def save_route_npy(self, path_array):
        save_path = Path(self.route_npy_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)

        route_xy = np.asarray(path_array[:, :2], dtype=np.float64)
        np.save(str(save_path), route_xy)
        rospy.loginfo(
            'Saved Dijkstra route npy: %s, shape=%s',
            str(save_path),
            route_xy.shape
        )

    def current_pose_callback(self, pose):
        if not self.route_links:
            return

        self.publish_current_link_info(
            float(pose.x),
            float(pose.y)
        )

    def publish_current_link_info(self, utm_x, utm_y):
        nearest_route = self.planner.find_nearest_route_link_and_segment(
            float(utm_x),
            float(utm_y),
            self.route_links
        )

        if nearest_route is None:
            return

        candidates = self.make_current_link_candidates(nearest_route['link'])
        nearest = self.planner.find_nearest_route_link_and_segment(
            float(utm_x),
            float(utm_y),
            candidates
        )

        if nearest is None:
            nearest = nearest_route

        link = nearest['link']
        link_idx = link.get('idx')

        node = self.planner.node_by_idx.get(link.get('to_node_idx'))

        on_stop_line = None
        traffic_light_id = None

        if node is not None:
            on_stop_line = node.get('on_stop_line')
            traffic_light_id = node.get('traffic_light_id')

        candidate_result = self.publish_lane_change_candidate_data(
            link_idx,
            nearest['point']['rviz_x'],
            nearest['point']['rviz_y']
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

        self.publish_current_link(
            link_idx,
            link.get('link_type'),
            link.get('max_speed'),
            on_stop_line,
            traffic_light_id,
            link.get('related_signal'),
            can_change_left,
            can_change_right
        )

    def publish_lane_change_candidate_data(
        self,
        current_link_idx,
        current_rviz_x,
        current_rviz_y
    ):
        link_idx = '' if current_link_idx is None else str(current_link_idx)

        if self.lane_change_candidate_generator is None:
            return {
                'current_link_idx': link_idx,
                'next_link_idx': None,
                'candidates': []
            }

        current_link = self.lane_change_candidate_generator.link_by_idx.get(
            link_idx
        )
        current_progress = None

        if current_link is not None:
            current_progress = (
                self.lane_change_candidate_generator.find_nearest_link_progress(
                    current_link,
                    float(current_rviz_x),
                    float(current_rviz_y)
                )
            )

        if (
            link_idx == self.last_lane_change_candidate_link_idx
            and current_progress is not None
            and self.last_lane_change_candidate_progress_m is not None
            and self.last_lane_change_candidate_result is not None
            and abs(
                current_progress - self.last_lane_change_candidate_progress_m
            ) < LANE_CHANGE_CANDIDATE_UPDATE_DISTANCE_M
        ):
            return self.last_lane_change_candidate_result

        result = self.lane_change_candidate_generator.make_candidates(
            link_idx,
            float(current_rviz_x),
            float(current_rviz_y)
        )
        self.lane_change_candidate_data_pub.publish(self.dumps(result))
        self.last_lane_change_candidate_link_idx = link_idx
        self.last_lane_change_candidate_progress_m = current_progress
        self.last_lane_change_candidate_result = result
        return result

    def make_current_link_candidates(self, nearest_route_link):
        route_index = self.find_route_link_index(nearest_route_link)

        if route_index is None:
            return [nearest_route_link]

        end_index = min(
            len(self.route_links),
            route_index + self.current_link_lookahead_route_count + 1
        )
        base_links = self.route_links[route_index:end_index]

        candidates = []
        seen_link_idxs = set()

        for link in base_links:
            self.add_candidate_link(candidates, seen_link_idxs, link)

            for side in ('left', 'right'):
                lane_link = self.get_lane_change_link(link, side)

                if lane_link is not None:
                    self.add_candidate_link(
                        candidates,
                        seen_link_idxs,
                        lane_link
                    )

        return candidates or [nearest_route_link]

    def find_route_link_index(self, target_link):
        target_idx = target_link.get('idx')

        for route_i, route_link in enumerate(self.route_links):
            if route_link is target_link:
                return route_i

            if target_idx is not None and route_link.get('idx') == target_idx:
                return route_i

        return None

    def add_candidate_link(self, candidates, seen_link_idxs, link):
        link_idx = link.get('idx')
        key = link_idx if link_idx is not None else id(link)

        if key in seen_link_idxs:
            return

        seen_link_idxs.add(key)
        candidates.append(link)

    def get_lane_change_link(self, link, side):
        if not self.can_change_lane(link, side):
            return None

        if side == 'left':
            dst_link_idx = link.get('left_lane_change_dst_link_idx')
        else:
            dst_link_idx = link.get('right_lane_change_dst_link_idx')

        return self.link_by_idx.get(dst_link_idx)

    def can_change_lane(self, link, side):
        if side == 'left':
            can_change = link.get('can_move_left_lane')
            dst_link_idx = link.get('left_lane_change_dst_link_idx')
        else:
            can_change = link.get('can_move_right_lane')
            dst_link_idx = link.get('right_lane_change_dst_link_idx')

        if not can_change or not dst_link_idx:
            return False

        return dst_link_idx in self.link_by_idx

    def publish_current_link(
        self,
        link_idx,
        link_type,
        max_speed,
        on_stop_line,
        traffic_light_id,
        related_signal,
        can_change_left,
        can_change_right
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
        self.current_link_pub.publish(msg)

    def to_float_or_default(self, value, default):
        if value is None:
            return float(default)

        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def log_path_result(self, result):
        start_info = result['start_info']
        goal_info = result['goal_info']
        dijkstra_result = result['dijkstra_result']

        rospy.loginfo('Start link: %s', start_info['link'].get('idx'))
        rospy.loginfo('Goal link : %s', goal_info['link'].get('idx'))
        rospy.loginfo(
            'Start nearest segment: %d, ratio: %.3f, distance: %.3f',
            start_info['segment_index'],
            start_info['segment_ratio'],
            start_info['distance']
        )
        rospy.loginfo(
            'Goal nearest segment : %d, ratio: %.3f, distance: %.3f',
            goal_info['segment_index'],
            goal_info['segment_ratio'],
            goal_info['distance']
        )

        if start_info['heading_error'] is not None:
            rospy.loginfo(
                'Start heading error: %.1f deg',
                math.degrees(start_info['heading_error'])
            )

            if not start_info['used_heading_filter']:
                rospy.logwarn(
                    'No segment matched heading filter %.1f deg. Used distance only.',
                    self.heading_filter_deg
                )

        if dijkstra_result is not None:
            rospy.loginfo(
                'Dijkstra total cost: %.3f',
                dijkstra_result['total_cost']
            )
            rospy.loginfo(
                'Dijkstra link count: %d',
                len(dijkstra_result['link_path'])
            )
            rospy.loginfo(
                'Dijkstra lane change count: %d',
                sum(
                    1 for edge in dijkstra_result.get('edge_path', [])
                    if edge.get('edge_type') != 'forward'
                )
            )

        rospy.loginfo('Global path generated')
        rospy.loginfo('Route link count: %d', len(result['route_links']))
        rospy.loginfo('Path point count: %d', len(result['path_array']))

    def make_path_payload(self, path_array):
        path = []

        for utm_x, utm_y, z in path_array:
            rviz_x, rviz_y = self.planner.utm_to_rviz_xy(utm_x, utm_y)
            path.append({
                'rviz_x': rviz_x,
                'rviz_y': rviz_y,
                'utm_x': float(utm_x),
                'utm_y': float(utm_y),
                'z': float(z)
            })

        return path

    def make_link_payload(self, link):
        return {
            'idx': link.get('idx'),
            'from_node_idx': link.get('from_node_idx'),
            'to_node_idx': link.get('to_node_idx'),
            'points': [self.make_point_payload(point) for point in link['points']],
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
