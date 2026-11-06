#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""터널 차선 변경 이력과 충돌 복구 차선 선택 (ROS와 독립)."""

import math
from collections import deque
from dataclasses import dataclass


TUNNEL_LINK_BY_LANE = {1: 'A2256W000131', 2: 'A2256W000126'}
TUNNEL_LANE_BY_LINK = {link: lane for lane, link in TUNNEL_LINK_BY_LANE.items()}
TUNNEL_LANE_CHANGE_S_WINDOW_M = 6.0
TUNNEL_REVERSE_TARGET_MAX_AGE_S = 0.5


@dataclass(frozen=True)
class TunnelLaneChange:
    from_lane: int
    to_lane: int
    s: float


class TunnelReversePolicy:
    def __init__(self, s_window_m=TUNNEL_LANE_CHANGE_S_WINDOW_M):
        self.s_window_m = float(s_window_m)
        self.current_lane = None
        self.last_change = None
        self.lane_changes = deque(maxlen=20)

    def observe_link(self, link_idx, current_s, reversing=False):
        """확정된 링크 전환을 기록한다. 후진으로 바뀐 차선은 이력을 덮어쓰지 않는다."""
        lane = TUNNEL_LANE_BY_LINK.get(str(link_idx))
        previous_lane = self.current_lane
        self.current_lane = lane
        if lane is None:
            # 이전 터널 통과의 이력은 다음 진입의 복구 판단에 사용하지 않는다.
            self.last_change = None
            return None

        if (previous_lane is not None and lane != previous_lane
                and not reversing and math.isfinite(current_s)):
            change = TunnelLaneChange(previous_lane, lane, float(current_s))
            self.last_change = change
            self.lane_changes.append(change)
            return change
        return None

    def select_reverse_lane(self, collision_s):
        """최근 변경의 도착 차선에서 ±6m 내 충돌이면 출발 차선을 선택한다."""
        change = self.last_change
        if (change is not None and self.current_lane == change.to_lane
                and math.isfinite(collision_s)
                and abs(float(collision_s) - change.s) <= self.s_window_m):
            return change.from_lane
        return self.current_lane
