# morai_link

`morai_link`는 MORAI MGeo 도로망과 차량 위치를 결합해, MACARON 8 자율주행 시스템이 사용할 수 있는 **현재 도로 상태와 주행 후보 경로**를 만드는 모듈입니다.

MGeo에는 도로 중심선, 링크 연결 관계, 제한속도, 차선변경 가능 여부, 정지선과 신호등 정보가 들어 있습니다. 이 모듈은 그 정보를 실시간 차량 위치에 맞춰 해석하여 다음과 같은 결과를 생성합니다.

- 현재 차량이 위치한 링크와 도로 속성
- 현재 차로를 유지하는 경로
- 왼쪽·오른쪽 차선변경 후보 경로
- 차선변경 후 따라갈 목적 링크
- 차량 뒤쪽의 경로 기준점
- 중앙선이 필요한 구간의 가상 차선 경계
- 동적 장애물이 속한 링크와 링크 중심선 기준 위치

## 전체 동작 흐름

| 단계 | 입력 | 처리 | 생성 결과 | MACARON 8에서의 사용 |
| ---: | --- | --- | --- | --- |
| 1. 지도 구성 | MGeo link/node | 좌표 변환, 링크 그래프와 최근접 검색 배열 생성 | 정규화된 도로망 | 이후 모든 링크 기반 계산의 기준 |
| 2. 현재 위치 반영 | `/current_pose`의 UTM x, y, yaw | 차량 위치를 도로 선분에 투영 | 현재 링크와 도로 속성 | 경로계획 및 mission 구간 판단 |
| 3. 주행 후보 생성 | 현재 링크와 좌우 차로 연결 관계 | 중심선 연결 및 Bezier 차선변경 곡선 계산 | center/left/right 후보 | spline 보간 후 후보 평가와 경로 선택 |
| 4. 목적 차로 구성 | 차선변경 목적 링크 | 여러 목적 링크의 중심선을 순서대로 결합 | 좌우 목적 링크 경로 | 차선변경 시작 후 목적 차로 추종 |
| 5. 후방 기준 생성 | 현재 위치와 이전 링크 | 링크를 역방향으로 따라가며 목표점 계산 | 8m 후방 목표 pose | 후진 및 주차 경로 생성 |
| 6. 가상 경계 생성 | 중앙선 보강 대상 링크 | 중심선을 왼쪽으로 1.15m 이동 | 가상 차선 경계 | 경로와 장애물 평가의 중앙선 제약 |
| 7. 장애물 링크 변환 | YOLO-LiDAR 동적 객체 | 객체를 진행 가능한 링크 중심선에 투영 | 링크 ID, 투영점, 횡거리, 방향 오차 | 객체와 차로의 관계 표현 |
| 8. 최종 경로 선택 | 위에서 생성된 링크·후보·경계 정보 | 점유격자 및 주행 조건과 함께 후보 평가 | 최종 경로, 속도, 조향 | 차량 제어 |

## 링크란?

MGeo의 link는 차선의 중앙선을 나타냅니다. 각 link에는 다음 정보가 연결되어 있습니다.

| 정보 | 의미 |
| --- | --- |
| `idx` | 링크 고유 ID |
| `from_node_idx` | 링크 시작 노드 |
| `to_node_idx` | 링크 종료 노드 |
| `points` | 도로 중심선을 구성하는 점 |
| `max_speed` | 제한속도 |
| `link_type` | 일반도로·교차로 등 도로 종류 |
| `can_move_left_lane` | 왼쪽 차로로 이동 가능한지 |
| `can_move_right_lane` | 오른쪽 차로로 이동 가능한지 |
| `*_lane_change_dst_link_idx` | 차선변경 목적 링크 |
| `related_signal` | 해당 링크의 신호 진행 방향 |

노드는 여러 링크를 연결하며 정지선 여부와 신호등 ID를 제공합니다. 따라서 현재 link를 알면 차량이 어느 차로에 있는지뿐 아니라 앞으로 만날 교차로와 신호 정보도 함께 알 수 있습니다.

## 1. 지도 구성

[morai_link_planner.py](path_planning/link/morai_link_planner.py)는 `link_set.json`과 `node_set.json`을 읽어 도로망을 구성합니다.

### 좌표 변환

MGeo의 로컬 좌표에 지도 원점을 더해 UTM 좌표를 만듭니다.

```text
origin_utm_x = 302595.0
origin_utm_y = 4124145.0

utm_x = morai_x + origin_utm_x
utm_y = morai_y + origin_utm_y
```

내부 후보 곡선 계산에는 원점을 뺀 로컬 좌표를 사용하고, 경로계획기로 발행할 때는 다시 UTM 좌표로 변환합니다.

### 검색 자료구조

차량 위치가 들어올 때마다 모든 지도 점을 순차 탐색하지 않도록 다음 자료를 미리 만듭니다.

- 링크 ID에서 링크를 찾는 `link_by_idx`
- 노드에서 시작하는 링크 목록 `links_by_from_node`
- 노드로 들어오는 링크 목록 `links_by_to_node`
- 좌우 차로 관계 `lane_neighbor_idxs_by_link_idx`
- 모든 링크 선분의 시작점, 방향벡터, 길이, heading을 담은 NumPy 배열

포함된 지도는 link 634개, node 480개, 선분 74,782개로 구성됩니다.

## 2. GLOBAL PATH와 검색범위

[morai_link.py](path_planning/link/morai_link.py)의 `CURRENT_LINK_ROUTE_LINK_IDXS`는 차량이 주행할 링크 순서를 정의합니다.

- route 항목: 40개
- 고유 route link: 39개
- 전체 길이: 약 2.185km
- 시작과 끝: `A2256W000748`

첫 링크와 마지막 링크가 같기 때문에 경로는 한 바퀴를 순환하는 폐회로입니다.

차량이 실제 route 옆 차로로 이동할 수 있으므로 route link만 검색하지 않습니다. 각 route link에서 최대 세 번의 좌우 차선 이동으로 도달할 수 있는 링크까지 확장해 55개의 현재 링크 후보를 구성합니다. 이 후보에 포함된 7,560개 선분이 실시간 위치 검색에 사용됩니다.

## 3. 현재 링크 생성

`/current_pose`는 다음 정보를 제공합니다.

```text
point.x = 차량 UTM Easting
point.y = 차량 UTM Northing
point.z = 차량 yaw [rad]
```

차량 점을 각 후보 선분 위에 직교 투영하고 투영점까지의 거리가 가장 짧은 선분을 찾습니다. yaw가 있으면 차량과 링크의 진행 방향도 함께 비교합니다.

```text
차량 위치 P
     │
     │ 최단거리
     ▼
A────────────Q────────────B   선택된 link segment
             ▲
          투영점
```

선택된 링크와 종료 노드의 정보를 조합해 `CurrentLink` 메시지를 만듭니다.

| 필드 | 만들어지는 값 | MACARON 8에서의 사용 |
| --- | --- | --- |
| `link_idx` | 현재 링크 ID | 현재 차로와 mission 구간 식별 |
| `link_type` | 도로 종류 | 교차로 여부와 경로 생성 범위 결정 |
| `max_speed` | 링크 제한속도 | 도로 속도 정보로 보관 |
| `on_stop_line` | 종료 노드의 정지선 여부 | 정지선 상태 정보 |
| `traffic_light_id` | 종료 노드의 신호등 ID | 신호등과 링크 연결 |
| `related_signal` | 직진·좌회전 등 관련 신호 | 진행 방향별 신호 정보 |
| `can_change_left/right` | 실제 후보 곡선 생성 가능 여부 | 불가능한 방향의 경로 생성과 후방 탐색 제외 |
| `intersection_type` | 신호등·회전교차로·고속도로·톨게이트 등의 수동 label | traffic 및 mission planner 상태 전환 |

`can_change_left/right`는 지도 플래그를 그대로 복사하지 않습니다. 현재 위치에서 해당 방향으로 충분한 후보 곡선을 실제로 만들 수 있을 때만 `true`가 됩니다.

MACARON 8에서는 이 메시지를 여러 모듈이 함께 사용합니다.

- `tracking_planner_occupancy.py`: 차선변경 방향 제한, 교차로 판정, 가드레일 끝과 톨게이트 진입 상태 판단
- `highway_planner.py`: `LANE_1`, `LANE_2`, `LANE_3` label을 이용한 의무 차선변경
- `traffic.py`: 3구·4구 신호등 인식 방식 선택
- `back.py`: 차선변경이 불가능한 방향의 후방 차량 검출 결과 제외

## 4. 주행 후보 경로

모듈은 차량 위치를 기준으로 최대 세 종류의 경로를 만듭니다.

| Candidate ID | 이름 | 의미 |
| ---: | --- | --- |
| 0 | center | 현재 차로 유지 |
| 1 | left | 왼쪽 목적 링크로 차선변경 |
| 2 | right | 오른쪽 목적 링크로 차선변경 |

### Center 후보

차량 투영점에서 시작해 현재 링크 중심선을 따라갑니다. 링크 끝에 도달하면 route의 다음 링크로 넘어가며 총 40m의 중심선 경로를 수집합니다.

```text
현재 link             다음 link
──────●───────────────●──────────────────>
      차량 투영점        중심선 연결
```

### Left/Right 후보

[lane_change_candidate.py](path_planning/link/lane_change_candidate.py)는 현재 링크의 차선변경 목적 링크를 찾고 시작점과 목적 링크의 진행 방향을 연결하는 3차 Bezier curve를 생성합니다.

```text
source link       P0 ───────────────>
                    ╲
                     ╲  Bezier candidate
                      ╲
destination link       ╲──── P3 ───>
```

후보의 기본 특성은 다음과 같습니다.

- 목표 길이 40m
- 목적 링크가 짧으면 다음 연결 링크까지 확장
- 확보 가능한 거리가 부족하면 최소 16m까지 축소
- 목표 길이 오차 0.15m 이내
- 약 0.5m 간격으로 Bezier 점 생성
- 차량이 같은 링크에서 1m 이상 진행하면 후보 갱신

목적 링크의 끝점을 바꿔 가며 Bezier 길이를 계산하고, 이분 탐색으로 약 40m가 되는 끝점을 찾습니다.

## 5. 후보 경로가 최종 경로로 사용되는 과정

`morai_link`가 만드는 후보는 지도 중심선과 Bezier 점으로 구성된 1차 경로입니다. MACARON 8에서는 `cubic_spline.cpp`가 이를 약 0.1m 간격의 부드러운 spline으로 바꿉니다.

```text
/lane_change_candidate_data
  x, y, candidate_id
            │
            ▼
       cubic_spline
            │
            ▼
/lane_change_candidate_spline
  x, y, yaw, curvature, candidate_id
```

`tracking_planner_occupancy.py`는 spline을 candidate ID별로 나누어 center, left, right 경로로 저장합니다. 이후 각 경로를 동적 점유격자와 비교하고 다음 조건을 함께 적용합니다.

- 정적·동적 장애물과의 충돌 가능성
- 현재 링크에서 허용되는 차선변경 방향
- 차량의 횡방향 위치와 속도
- 교차로·터널·고속도로 등 현재 구간 상태
- 후방 차량 상태
- 링크 기반 목적 차로

이 평가를 통과한 후보가 실제 조향과 속도 계산에 사용할 최종 경로가 됩니다.

## 6. 차선변경 목적 링크

`/lane_change_candidate_link`는 Bezier 곡선이 아니라 차선변경 후 따라갈 **목적 링크의 원본 중심선**을 제공합니다.

```text
x, y, candidate_id, link_order
```

`link_order`는 목적 경로가 여러 링크로 이어질 때 링크 순서를 나타냅니다. `tracking_planner_occupancy.py`는 left/right별 중심선을 복원해 다음 목적으로 사용합니다.

- 차선변경 후보 생성
- 차선변경이 시작된 뒤 목적 링크 중심선 추종
- 일반 lattice 경로가 적합하지 않을 때 link 기반 경로 유지

## 7. 후방 목표점

[reverse_link.py](path_planning/link/reverse_link.py)는 현재 위치에서 링크 진행 방향을 거슬러 기본 8m 뒤의 목표점을 찾습니다.

현재 링크 안에서 거리가 부족하면 이전 링크로 넘어갑니다. 분기점에서는 실제로 관측한 직전 링크, route에 정의된 이전 링크, heading 연결 순서로 자연스러운 이전 링크를 선택합니다.

```text
이전 link                  현재 link
──── target ◀──────────── vehicle ────────────>
       8m
```

결과는 다음 형태입니다.

```text
Point.x = 목표 UTM x
Point.y = 목표 UTM y
Point.z = 목표 진행 yaw
```

MACARON 8의 parking planner는 후진 상태에서 이 점을 목표 pose로 사용합니다.

## 8. 가상 차선 경계

[virtual_lane_boundary.py](path_planning/link/virtual_lane_boundary.py)는 중앙선 정보가 추가로 필요한 지정 구간에서 링크 중심선을 왼쪽으로 1.15m 이동한 가상 경계를 만듭니다.

```text
가상 좌측 경계  ─────────────────────
                  1.15m
링크 중심선      ─────────────────────>
```

현재 적용 구간은 다음과 같습니다.

```text
A2256W000304 → A2256W000148
A2256W000213
```

`mppsl_sl_occupancy.py`는 이 경계를 Frenet `s-q` 좌표로 변환합니다. 이후 장애물 필터와 후보 경로 평가에서 차량이 넘어가면 안 되는 가상의 노란 중앙선으로 사용합니다.

## 9. 동적 장애물의 링크 투영

[dynamic_obstacle_link_projector.py](path_planning/link/dynamic_obstacle_link_projector.py)는 YOLO-LiDAR fusion이 추적한 이동 객체를 도로 링크 위에 투영합니다.

처리 대상은 다음 조건을 만족하는 객체입니다.

- 이동 상태가 `MOVING`
- 진행 방향이 유효함
- 자차에서 50m 이내

객체와 가장 가까운 주행 가능 링크 선분을 찾고 객체 heading과 링크 heading을 비교합니다. 결과에는 다음 정보가 포함됩니다.

| 결과 | 의미 |
| --- | --- |
| `track_id` | 객체 추적 ID |
| `nearest_link_idx` | 객체가 속한 것으로 판단한 링크 |
| `projected_position_utm` | 링크 중심선 위의 투영점 |
| `ego_distance_m` | 자차와 객체 사이 거리 |
| `signed_link_distance_m` | 링크 중심선 기준 횡방향 거리 |
| `signed_heading_error_rad` | 링크와 객체 진행 방향 차이 |

횡방향 거리는 링크 진행 방향을 기준으로 왼쪽이 양수, 오른쪽이 음수입니다.

```text
          object (+distance)
              ●
              │
link  ────────●────────────────>
          projection
```

이 데이터는 단순한 객체 UTM 좌표를 “어느 차로의 어느 쪽에 있는 객체인가”라는 도로 기준 정보로 변환한 결과입니다.

## 생성되는 ROS 데이터

| Topic | Type | 만들어지는 정보 | 주요 사용처 |
| --- | --- | --- | --- |
| `/morai_map_data` | `std_msgs/String` | 전체 link/node와 좌표·속성 | 지도 및 RViz 시각화 |
| `/current_link_data` | `macaron_8/CurrentLink` | 현재 도로 상태 | tracking, highway, traffic, rear vision |
| `/lane_change_candidate_data` | `sensor_msgs/PointCloud2` | center/left/right 기하 경로 | cubic spline 보간 |
| `/lane_change_candidate_link` | `sensor_msgs/PointCloud2` | 목적 링크 중심선 | 차선변경 및 목적 차로 추종 |
| `/reverse_link_point` | `geometry_msgs/Point` | 링크 기준 후방 목표 pose | 후진·주차 경로 |
| `/current_link_virtual_lane_boundaries` | `sensor_msgs/PointCloud2` | 가상 좌측 중앙선 | 경로·장애물 제약 |
| `/dynamic_obstacle_link_data` | `macaron_8/DynamicObstacleLinkArray` | 객체의 링크 소속과 상대 위치 | 객체-차로 관계 표현 |

## 모듈 구성

```text
path_planning/link/
├── morai_link.py
│   └── ROS 입출력과 전체 처리 순서
├── morai_link_planner.py
│   └── 지도 로드, 좌표 변환, 그래프와 최근접 검색
├── lane_change_candidate.py
│   └── center/left/right 후보 경로
├── reverse_link.py
│   └── 링크를 따라가는 후방 목표점
├── virtual_lane_boundary.py
│   └── 중심선 offset 기반 가상 경계
└── dynamic_obstacle_link_projector.py
    └── 동적 객체의 링크 투영
```

`msg/`에는 위 결과를 표현하는 ROS 메시지 구조가 있고, `path/MGeo/`에는 분석에 사용한 link/node 지도가 있습니다.

더 세부적인 함수 단위 처리 순서는 [Architecture](docs/ARCHITECTURE.md)에 정리되어 있습니다.

## 시각화 예시

아래 이미지는 MGeo 도로망, 생성 경로와 차량 위치를 MORAI/RViz에서 비교한 결과입니다.

![RViz 전역 경로](docs/images/rviz-global-path-cropped.png)

![MORAI와 RViz 주행 위치 비교](docs/images/morai-rviz-driving-cropped.png)

![회전교차로 링크 경로](docs/images/morai-rviz-roundabout-cropped.png)
