# Architecture

## 1. 책임 범위

`morai_link`는 localization과 최종 trajectory planner 사이에서 MGeo 도로망을 주행 가능한 형태로 변환하는 계층이다.

```text
Localization                  MGeo map                 Object fusion
 /current_pose          link_set / node_set            dynamic_points
       │                       │                             │
       └──────────────┬────────┴───────────────┬─────────────┘
                      ▼                        ▼
              current-link matching    obstacle projection
                      │
            ┌─────────┼───────────┐
            ▼         ▼           ▼
        candidates  reverse    virtual boundary
            │       target
            └─────────┬───────────┘
                      ▼
             downstream planner
```

이 계층은 장애물 회피 경로를 최종 선택하지 않는다. 지도 기반 후보와 metadata를 만들고, 충돌 검사와 후보 선택은 하위 planning node가 담당한다.

## 2. 초기화 순서

`MoraiMapPublisher`가 생성되면 다음 순서로 초기화한다.

1. `path/MGeo/link_set.json`, `node_set.json` 경로 결정
2. `MoraiMapPlanner.load(build_nearest_lookup=False)`로 지도 정규화
3. `ReverseLinkTargetFinder` 생성
4. 차선변경 목적 링크의 원본 point cache 생성
5. route 39개를 기준으로 최대 3회 차선 이동 closure 계산
6. closure에 포함된 55개 링크만 최근접 검색 배열로 변환
7. 후보 생성기와 동적 객체 투영기 생성
8. 약 17MB의 전체 지도 JSON을 latched publish
9. pose/object subscriber와 10Hz reverse target timer 시작

## 3. MoraiMapPlanner

### 지도 정규화

`morai_link_planner.py`는 JSON record가 list 또는 `idx -> object` dictionary인 형식을 모두 읽는다. 점이 2개 미만인 링크와 올바르지 않은 좌표는 제외한다.

각 point는 다음 좌표를 동시에 보관한다.

| 필드 | 의미 |
| --- | --- |
| `morai_x/y/z` | 원본 MGeo local 좌표 |
| `utm_x/y` | UTM 원점을 더한 전역 좌표 |
| `rviz_x/y` | 설정에 따라 원점을 뺀 표시·곡선 계산 좌표 |

MGeo 오류를 보정하기 위한 링크별 override도 적용한다.

- 일부 링크의 좌우 차선변경 금지
- `A2256W000319 ↔ A2256W000318`, `A2256W000849 ↔ A2256W000054` 목적 링크 보정
- `A2256W000054`의 link type을 `4`로 보정
- `A2256W000182`의 related signal을 `left_unprotected`로 보정

### 그래프 index

다음 lookup을 구축한다.

- `link_by_idx`
- `links_by_from_node`
- `links_by_to_node`
- `lane_neighbor_idxs_by_link_idx`
- link별 NumPy segment index

### 최근접 링크

모든 후보 선분에 대해 차량 점을 선분 위에 투영한다.

```text
t = clamp(dot(vehicle - p0, p1 - p0) / |p1 - p0|², 0, 1)
projection = p0 + t(p1 - p0)
```

yaw가 있으면 방향 차이가 90° 이하인 선분만 우선 검색한다. 방향 조건을 만족하는 선분이 하나도 없으면 거리만으로 fallback한다.

## 4. 현재 링크 상태기계

첫 pose에서는 route와 lane-change closure 전체를 검색한다. 이후에는 다음 범위만 우선 검색한다.

- 현재 링크
- 현재 링크의 from/to node에서 시작하는 링크
- 현재 링크과 다음 링크에서 최대 3회 차선 이동으로 도달하는 링크

지역 검색 결과가 없거나 거리가 4m보다 크거나 방향 filter가 적용되지 않았다면 전체 55개 whitelist에서 재검색한다.

선택된 링크로부터 `CurrentLink`를 구성한다.

| 필드 | 계산 기준 |
| --- | --- |
| `link_idx` | 최근접 링크 ID |
| `link_type` | MGeo type, 필요 시 교차로 10m preview |
| `max_speed` | MGeo link metadata |
| `on_stop_line` | link의 `to_node` metadata |
| `traffic_light_id` | link의 `to_node` metadata |
| `related_signal` | MGeo link metadata/override |
| `can_change_left/right` | 해당 방향 후보 곡선이 실제 생성됐는지 |
| `intersection_type` | 링크 ID별 수동 mapping |

## 5. 차선변경 후보

### Center

차량 투영점부터 현재 링크 중심선을 따라 전방 40m를 수집한다. 링크 끝에 도달하면 route 우선순위에 따라 다음 연결 링크로 넘어간다. 16m 미만밖에 만들 수 없으면 후보를 버린다.

### Left / Right

MGeo의 `*_lane_change_dst_link_idx`가 유효할 때만 생성한다.

1. source 시작점과 heading 결정
2. source point를 destination 중심선에 투영
3. destination 전방 link chain 확장
4. source와 destination endpoint 사이 3차 Bezier 생성
5. 최대 12회의 이분 탐색으로 곡선 길이를 `40.0 ± 0.15m`에 맞춤
6. destination chain이 짧으면 16m까지 축소 허용

Bezier control point 거리는 chord 길이의 절반을 사용하되 2~15m로 제한한다. 표본 수는 chord 길이를 약 0.5m 간격으로 나눈 값이다.

후보 결과는 link와 진행거리가 이전 결과에서 1m 미만으로 변했으면 cache를 재사용한다.

## 6. 후방 목표점

`ReverseLinkTargetFinder`는 현재 링크에서 가장 가까운 원본 point index를 찾고 링크 point를 역순으로 누적한다. 기본 8m에 도달하면 해당 point와 전방 point로 yaw를 만든다.

분기점의 이전 링크 우선순위는 다음과 같다.

1. pose 처리 중 실제로 관측된 직전 링크
2. route에 정의된 이전 링크
3. 후보가 하나뿐인 이전 링크
4. 현재 링크 시작 heading과 가장 자연스럽게 연결되는 링크

출력은 timestamp가 없는 `geometry_msgs/Point`이며 `z`를 yaw로 사용한다.

## 7. 가상 좌측 경계

현재 적용 대상은 다음 두 그룹이다.

```text
A2256W000304 -> A2256W000148
A2256W000213
```

각 링크 중심선의 접선을 계산하고 왼쪽 법선 방향으로 `2.3 / 2 = 1.15m` 이동한다. 연속 링크 끝점이 1mm 이내이면 중복 point를 제거하고 한 선으로 연결한다.

## 8. 동적 객체 링크 투영

다음 조건을 모두 만족하는 객체만 처리한다.

- `motion_state == 2 (MOVING)`
- `heading_valid == true`
- 자차와 평면 거리 50m 이하

현재 링크에서 전방 route와 최대 2회 차선 이동으로 도달할 수 있는 링크들을 후보로 만든다. 각 객체를 기하학적으로 가장 가까운 후보 선분에 먼저 투영한 뒤 방향 차이가 45°를 넘으면 제외한다.

출력의 부호 있는 횡거리는 링크 진행 방향 기준 왼쪽이 양수, 오른쪽이 음수다.

```text
signed_distance = cross(link_vector, obstacle - projection) / |link_vector|
```

## 9. downstream 연결

MACARON 8 전체 시스템에서는 다음 연결을 사용한다.

```text
/lane_change_candidate_data
        │
        ▼
   cubic_spline
        │  x, y, yaw, curvature, candidate_id
        ▼
/lane_change_candidate_spline
        │
        ├── /current_link_data
        ├── /lane_change_candidate_link
        ├── /current_link_virtual_lane_boundaries
        └── /reverse_link_point
                    │
                    ▼
       tracking_planner_occupancy
```

`traffic.py`와 후방 카메라 노드도 `CurrentLink.intersection_type`, `can_change_left/right`를 사용한다.

