# dijkstra_path.py
import heapq

import numpy as np


def calc_link_length_from_points(link):
    points = link.get('points', [])

    if len(points) < 2:
        return float('inf')

    try:
        # {'utm_x': ..., 'utm_y': ...}
        if isinstance(points[0], dict):
            xy = np.asarray(
                [
                    (
                        p.get('utm_x', p.get('morai_x')),
                        p.get('utm_y', p.get('morai_y'))
                    )
                    for p in points
                ],
                dtype=np.float64
            )

        # 원본 link_set.json 형식:
        # [x, y, z]
        else:
            xy = np.asarray(points, dtype=np.float64)[:, :2]
    except (TypeError, ValueError, IndexError):
        return float('inf')

    if xy.shape[0] < 2 or not np.all(np.isfinite(xy)):
        return float('inf')

    segment_xy = np.diff(xy, axis=0)
    return float(np.linalg.norm(segment_xy, axis=1).sum())


def link_points_to_xy(link):
    points = link.get('points', [])

    if len(points) < 2:
        return None

    try:
        if isinstance(points[0], dict):
            xy = np.asarray(
                [
                    (
                        p.get('utm_x', p.get('morai_x')),
                        p.get('utm_y', p.get('morai_y'))
                    )
                    for p in points
                ],
                dtype=np.float64
            )
        else:
            xy = np.asarray(points, dtype=np.float64)[:, :2]
    except (TypeError, ValueError, IndexError):
        return None

    if xy.shape[0] < 2 or not np.all(np.isfinite(xy)):
        return None

    return xy


def distance_from_index_to_end(xy, start_index):
    start_index = max(0, min(int(start_index), xy.shape[0] - 1))

    if start_index >= xy.shape[0] - 1:
        return 0.0

    segment_xy = np.diff(xy[start_index:], axis=0)
    return float(np.linalg.norm(segment_xy, axis=1).sum())


def find_nearest_xy_index(xy, point_xy):
    diff_xy = xy - point_xy
    dist_sq = diff_xy[:, 0] * diff_xy[:, 0] + diff_xy[:, 1] * diff_xy[:, 1]
    return int(np.argmin(dist_sq))


def has_lane_change_distance(link, dst_link, lane_change_lookahead_m):
    if lane_change_lookahead_m is None or lane_change_lookahead_m <= 0.0:
        return True

    current_xy = link_points_to_xy(link)
    dst_xy = link_points_to_xy(dst_link)

    if current_xy is None or dst_xy is None:
        return False

    min_change_index = min(current_xy.shape[0] - 1, 1)
    preferred_index = int((current_xy.shape[0] - 1) * 0.1)
    change_index = max(min_change_index, preferred_index)
    change_index = min(change_index, current_xy.shape[0] - 1)

    if distance_from_index_to_end(current_xy, change_index) < lane_change_lookahead_m:
        return False

    nearest_dst_index = find_nearest_xy_index(dst_xy, current_xy[change_index])

    return (
        distance_from_index_to_end(dst_xy, nearest_dst_index)
        >= lane_change_lookahead_m
    )


def get_link_cost(link):
    """
    Dijkstra에서 사용할 link 비용 계산.
    항상 link 길이 기준 최단거리만 사용한다.
    """

    link_length = link.get('link_length', None)

    if link_length is None:
        link_length = calc_link_length_from_points(link)

    link_length = float(link_length)
    return link_length



def build_link_graph(
    links,
    lane_change_penalty=30.0,
    lane_change_lookahead_m=20.0
):
    """
    links 리스트를 link 기반 방향 그래프로 변환.

    graph[link_idx] = [
        {
            'to_link': next_link_idx,
            'from_link': current_link,
            'link': next_link,
            'edge_type': 'forward' / 'lane_change_left' / 'lane_change_right',
            'cost': cost
        },
        ...
    ]
    """

    graph = {}
    link_by_idx = {}
    links_by_from_node = {}

    for link in links:
        link_idx = link.get('idx')
        from_node = link.get('from_node_idx')

        if link_idx is None:
            continue

        link_by_idx[link_idx] = link
        graph.setdefault(link_idx, [])

        if from_node is not None:
            links_by_from_node.setdefault(from_node, []).append(link)

    for link in links:
        link_idx = link.get('idx')
        to_node = link.get('to_node_idx')

        if link_idx is None:
            continue

        for next_link in links_by_from_node.get(to_node, []):
            next_link_idx = next_link.get('idx')

            if next_link_idx is None or next_link_idx == link_idx:
                continue

            cost = get_link_cost(next_link)

            if cost == float('inf'):
                continue

            graph[link_idx].append({
                'to_link': next_link_idx,
                'from_link': link,
                'link': next_link,
                'edge_type': 'forward',
                'cost': cost
            })

        add_lane_change_edge(
            graph,
            link_by_idx,
            link,
            'left',
            lane_change_penalty,
            lane_change_lookahead_m
        )
        add_lane_change_edge(
            graph,
            link_by_idx,
            link,
            'right',
            lane_change_penalty,
            lane_change_lookahead_m
        )

    return graph, link_by_idx


def add_lane_change_edge(
    graph,
    link_by_idx,
    link,
    side,
    lane_change_penalty,
    lane_change_lookahead_m
):
    link_idx = link.get('idx')

    if side == 'left':
        can_change = link.get('can_move_left_lane')
        dst_link_idx = link.get('left_lane_change_dst_link_idx')
        edge_type = 'lane_change_left'
    else:
        can_change = link.get('can_move_right_lane')
        dst_link_idx = link.get('right_lane_change_dst_link_idx')
        edge_type = 'lane_change_right'

    if not can_change or not dst_link_idx:
        return

    if dst_link_idx == link_idx or dst_link_idx not in link_by_idx:
        return

    dst_link = link_by_idx[dst_link_idx]
    dst_cost = get_link_cost(dst_link)

    if dst_cost == float('inf'):
        return

    if not has_lane_change_distance(link, dst_link, lane_change_lookahead_m):
        return

    graph[link_idx].append({
        'to_link': dst_link_idx,
        'from_link': link,
        'link': dst_link,
        'edge_type': edge_type,
        'cost': float(lane_change_penalty) + dst_cost
    })




def dijkstra_link(
    links,
    start_link_idx,
    goal_link_idx,
    lane_change_penalty=30.0,
    lane_change_lookahead_m=20.0
):
    """
    일반 진행 edge:
    - current_link.to_node_idx == next_link.from_node_idx

    차선변경 edge:
    - current_link.left/right_lane_change_dst_link_idx가 가리키는 link
    """

    graph, link_by_idx = build_link_graph(
        links,
        lane_change_penalty=lane_change_penalty,
        lane_change_lookahead_m=lane_change_lookahead_m
    )

    if start_link_idx not in link_by_idx or goal_link_idx not in link_by_idx:
        return {
            'success': False,
            'total_cost': float('inf'),
            'node_path': [],
            'link_idx_path': [],
            'link_path': [],
            'edge_path': []
        }

    queue = []
    heapq.heappush(queue, (0.0, start_link_idx))

    distances = {
        start_link_idx: 0.0
    }

    previous = {
        start_link_idx: {
            'prev_link': None,
            'edge': None
        }
    }

    visited = set()

    while queue:
        current_cost, current_link_idx = heapq.heappop(queue)

        if current_link_idx in visited:
            continue

        visited.add(current_link_idx)

        if current_link_idx == goal_link_idx:
            break

        for edge in graph.get(current_link_idx, []):
            next_link_idx = edge['to_link']
            edge_cost = edge['cost']
            new_cost = current_cost + edge_cost

            if next_link_idx not in distances or new_cost < distances[next_link_idx]:
                distances[next_link_idx] = new_cost
                previous[next_link_idx] = {
                    'prev_link': current_link_idx,
                    'edge': edge
                }
                heapq.heappush(queue, (new_cost, next_link_idx))

    if goal_link_idx not in distances:
        return {
            'success': False,
            'total_cost': float('inf'),
            'node_path': [],
            'link_idx_path': [],
            'link_path': [],
            'edge_path': []
        }

    link_idx_path = []
    edge_path = []
    current_link_idx = goal_link_idx

    while current_link_idx is not None:
        link_idx_path.append(current_link_idx)
        prev_info = previous[current_link_idx]
        edge = prev_info['edge']

        if edge is not None:
            edge_path.append(edge)

        current_link_idx = prev_info['prev_link']

    link_idx_path.reverse()
    edge_path.reverse()

    return {
        'success': True,
        'total_cost': distances[goal_link_idx],
        'node_path': [],
        'link_idx_path': link_idx_path,
        'link_path': [link_by_idx[link_idx] for link_idx in link_idx_path],
        'edge_path': edge_path
    }