# Architecture

## 1. 프로젝트 책임 범위

`morai_link`는 MACARON 8의 localization, 정밀도로지도, 경로계획 사이에서 **link 기반 주행 문맥과 후보 경로를 만드는 계층**이다.

```text
Localization                  MGeo map                 Object fusion
 /current_pose          link_set / node_set           dynamic objects
       │                       │                             │
       └──────────────┬────────┴───────────────┬─────────────┘
                      ▼                        ▼
              current-link matching    obstacle projection
                      │
            ┌─────────┼──────────────┐
            ▼         ▼              ▼
        candidates  reverse target  virtual boundary
            │
            ▼
      road-context metadata
            │
            ▼
   spline / occupancy / mission planner
```

이 계층이 담당하는 것은 다음과 같다.

- MGeo link/node 정규화와 검색 구조 생성
- 차량 위치에 해당하는 현재 link 추정 및 전환 안정화
- 차로 유지와 좌·우 차선변경 후보 생성
- 차선변경 목적 link 중심선 구성
- 정지선·신호·교차로·mission 구간 metadata 생성
- 후진 기준점, 가상 중앙선, 동적 객체의 link 상대 정보 생성

생성한 후보와 도로 정보를 MACARON 8의 경로계획·제어 계층에 전달해 최종 경로 선택과 차량 주행에 활용한다.

## 2. 모듈 의존 관계

```text
MoraiMapPublisher (morai_link.py)
│
├── MoraiMapPlanner
│   ├── link/node JSON 정규화
│   ├── graph lookup
│   ├── lane-change closure
│   └── vectorized nearest-segment projection
│
├── LaneChangeCandidateGenerator
│   ├── center link chain
│   ├── left/right destination chain
│   └── cubic Bezier length fitting
│
├── ReverseLinkTargetFinder
│   └── observed route와 graph를 이용한 이전 link 역추적
│
├── VirtualLaneBoundaryGenerator
│   └── 지정 link 중심선의 좌측 offset
│
└── DynamicObstacleLinkProjector
    └── 이동 객체를 주행 가능한 link 선분에 투영
```

`morai_link.py`는 ROS 입출력과 상태를 담당하고, 나머지 다섯 모듈은 지도 기하와 그래프 계산을 분리해 담당한다.

## 3. 초기 데이터 구성

`MoraiMapPublisher`의 지도 초기화 순서는 다음과 같다.

1. `path/MGeo/link_set.json`, `node_set.json`의 위치 결정
2. `MoraiMapPlanner.load(build_nearest_lookup=False)`로 지도 정규화
3. `ReverseLinkTargetFinder`에 link graph와 route 전달
4. 차선변경 목적 link 원본 점 cache 생성
5. route link와 최대 3회 차선변경 closure 계산
6. closure에 포함된 link만 최근접 검색 NumPy 배열로 변환
7. 후보 경로, 가상 경계, 동적 객체 투영 모듈 구성
8. 전체 지도 payload와 위치 기반 결과 생성 준비

지도 전체는 634개 link, 480개 node, 74,782개 선분이다. 실제 현재 link 검색 index에는 route와 주변 차로를 합친 55개 link, 7,560개 선분만 넣어 탐색 범위를 줄였다.

## 4. MoraiMapPlanner

### 4.1 지도 정규화

MGeo point를 다음 세 좌표 표현으로 저장한다.

| 필드 | 의미 |
| --- | --- |
| `morai_x/y/z` | MGeo JSON의 원본 로컬 좌표 |
| `utm_x/y` | 지도 원점 `(302595.0, 4124145.0)`을 더한 localization 좌표 |
| `rviz_x/y` | 원점 offset 설정을 반영한 표시·곡선 계산 좌표 |

JSON의 최상위가 list인 형식과 `idx -> record` dictionary인 형식을 모두 처리한다. 좌표가 유효하지 않거나 point가 2개 미만인 link는 기하 계산에서 제외한다.

### 4.2 MGeo override

주행 구간에 적용할 지도 속성을 정규화 단계에서 설정한다.

- 특정 link의 좌·우 차선변경 허용 여부 설정
- 네 개 link의 좌·우 차선변경 목적 관계 보완
- `A2256W000054`의 `link_type` 보정
- `A2256W000182`의 `related_signal`을 `left_unprotected`로 보정

원본 JSON 자체를 수정하지 않고 override를 코드에 분리해 어떤 보정을 적용했는지 추적할 수 있게 했다.

### 4.3 graph lookup

정규화된 지도에서 다음 lookup을 만든다.

- `link_by_idx`: ID로 link 조회
- `node_by_idx`: ID로 node 조회
- `links_by_from_node`: node에서 출발하는 link
- `links_by_to_node`: node로 들어오는 link
- `lane_neighbor_idxs_by_link_idx`: 좌우 차로 관계를 양방향 인접 집합으로 표현
- `segment_indices_by_link_idx`: link별 최근접 검색 배열 위치

`get_lane_change_link_closure()`는 기준 link에서 좌·우 차선변경 관계를 breadth-first 방식으로 확장한다. 현재 link whitelist에는 최대 3회, 동적 객체 후보에는 최대 2회의 이동 범위를 적용한다.

### 4.4 최근접 선분 투영

link의 각 연속 point 쌍을 하나의 선분으로 만들고 시작점, 방향벡터, 길이 제곱, heading을 NumPy 배열로 보관한다. 차량 위치 `P`를 선분 `A → B`에 투영하는 비율은 다음과 같다.

```text
t = clamp(dot(P - A, B - A) / |B - A|², 0, 1)
Q = A + t(B - A)
distance = |P - Q|
```

yaw가 주어지면 차량과 선분 heading의 차이가 90° 이내인 선분을 우선 사용한다. 조건을 만족하는 선분이 없을 때는 거리만으로 다시 선택한다. 반환값에는 link, 투영점, 선분 index, 선분 내 비율, 거리, heading filter 적용 여부가 포함된다.

## 5. route와 현재 link 상태

route는 40개 항목, 39개 고유 link로 구성된 약 2.185km 순환 경로다. 첫 link와 마지막 link가 `A2256W000748`로 같아 전방·후방 연결을 계속 유지할 수 있다.

### 5.1 계층형 검색 범위

첫 pose에서는 55개 whitelist 전체를 검색한다. 현재 link가 정해진 이후에는 다음 후보만 먼저 검색한다.

- 현재 link
- 현재 link의 시작·종료 node에서 앞으로 나가는 link
- 현재 link 다음에 연결되는 link
- 현재 link와 다음 link에서 최대 3회 차선 이동으로 연결되는 link

지역 검색 결과가 없거나, 최근접 거리가 4m를 넘거나, 주어진 yaw로 heading filter가 적용되지 못하면 whitelist 전체에서 link를 다시 찾는다.

### 5.2 연속 관측 기반 전환

최근접 계산 결과가 현재 link와 다르더라도 즉시 `current_link_idx`를 바꾸지 않는다.

```text
same as current ──> pending 초기화 ──> 현재 link 유지

new candidate ────> pending candidate 기록
                          │
                          ├─ 다른 후보 관측: count를 1로 재시작
                          ├─ 현재 link 재관측: pending 취소
                          └─ 같은 후보 10회 연속: 현재 link 전환
```

전환 확인 중에는 현재 link 하나만 대상으로 차량을 다시 투영해 기존 link의 진행도와 후보 경로의 연속성을 유지한다. 이 상태는 `pending_current_link_idx`와 `pending_current_link_count`로 관리되며 기본 확인 횟수는 10회다.

후보 경로 데이터와 차선변경 목적 link 데이터는 각각 별도의 현재 link/cache 상태를 가진다. 위치 업데이트 시점이 같더라도 두 결과의 갱신 조건과 latched 발행 여부가 다르기 때문이다.

## 6. 도로 문맥 생성

현재 link의 속성과 종료 node를 결합해 `CurrentLink`를 구성한다.

| 정보 | 출처 또는 계산 |
| --- | --- |
| 현재 차로 | 최근접 link의 `idx` |
| 도로 type | MGeo `link_type`과 교차로 preview |
| 제한속도 | link의 `max_speed` |
| 정지선 | 종료 node의 `on_stop_line` |
| 신호등 ID | 종료 node의 `traffic_light_id` |
| 진행 신호 | link의 `related_signal` |
| 좌·우 차선변경 가능 | 해당 방향 후보가 실제 생성되었는지 |
| 구간 label | link ID별 `INTERSECTION_TYPE_BY_LINK_IDX` mapping |

### 6.1 `link_type` preview

현재 link type이 `6`이고 다음 연결 link가 교차로 type `1`이면, 현재 link 종점까지 남은 거리를 중심선 구간별로 누적한다. 기본 10m 이내에 들어왔을 때 미리 type `1`을 발행해 downstream planner가 교차로 상태를 준비할 수 있게 했다.

### 6.2 `intersection_type` 거리 제한

구간 label은 신호등 형태, 회전교차로, 고속도로 진입, 차로 번호, 톨게이트, 가드레일 종점 등을 표현한다. 이 중 긴 신호 link 세 개는 link 전체에서 label을 내보내지 않고 다음 거리 이내에서만 활성화한다.

| Link | Label | 종점 preview |
| --- | --- | ---: |
| `A2256W000213` | `three_light_traffic_signal` | 25m |
| `A2256W000308` | `four_light_traffic_signal` | 20m |
| `A2256W000315` | `four_light_traffic_signal` | 20m |

`is_near_link_end()`는 현재 선분의 남은 비율과 뒤에 이어지는 모든 선분 길이를 합해 종점까지의 실제 중심선 거리를 계산한다.

## 7. LaneChangeCandidateGenerator

### 7.1 현재 차로 유지 후보

차량 투영점에서 시작해 현재 link 중심선을 따라 전방 40m를 수집한다. link 끝에 도달하면 route에 지정된 다음 link를 우선하고, 그 관계가 없으면 같은 node에서 자연스럽게 연결되는 link를 선택한다.

경로 길이가 기본 목표보다 짧더라도 최소 16m를 확보하면 축소된 후보로 유지한다. 연속 link의 같은 끝점은 중복 제거한다.

### 7.2 좌·우 차선변경 후보

MGeo의 `left/right_lane_change_dst_link_idx`가 유효한 방향에 대해 다음 순서로 만든다.

1. source link에서 차량의 진행도와 시작 heading 계산
2. source 위치를 destination link chain에 투영
3. destination link가 짧으면 다음 link까지 기하 연결
4. 시작점 `P0`, 시작 heading, 목적점 `P3`, 목적 heading 구성
5. chord 길이의 절반을 기본으로 2~15m 범위의 control distance 설정
6. 3차 Bezier 곡선 생성
7. 최대 12회 이분 탐색으로 `40.0 ± 0.15m` 길이에 맞춤
8. chord 기준 약 0.5m 간격으로 curve sampling

후보 결과에는 방향, 경로 점, 목적 link ID와 목적 link chain이 함께 들어간다.

### 7.3 두 종류의 후보 출력

- `lane_change_candidate_data`: 현재 위치에서 시작한 center/left/right 실제 기하 경로
- `lane_change_candidate_link`: left/right 차선변경 이후 추종할 목적 link의 원본 중심선과 순서

현재 link와 link 내 진행거리가 같고 이동량이 1m 미만이면 이전 결과를 재사용한다. 두 출력의 계산 결과가 같으면 cache도 공유하되, 각 topic의 발행 signature는 독립적으로 관리한다.

## 8. ReverseLinkTargetFinder

현재 위치와 가장 가까운 원본 link point에서 시작해 point를 역순으로 누적한다. 기본 8m의 목표 거리에 따라 시작 node로 들어오는 이전 link까지 연결해 탐색한다.

분기점의 이전 link 우선순위는 다음과 같다.

1. pose 처리 중 실제로 관측된 직전 link
2. route에 정의된 이전 link
3. 후보가 하나뿐인 이전 link
4. 현재 link 시작 heading과 가장 자연스럽게 연결되는 link

목표 point와 그 다음 전방 point로 yaw를 계산해 후진·주차 경로계획이 사용할 위치와 방향을 함께 제공한다.

## 9. VirtualLaneBoundaryGenerator

가상 중앙선이 필요한 link 그룹은 다음과 같다.

```text
A2256W000304 -> A2256W000148
A2256W000213
```

각 중심선 point의 접선에서 왼쪽 법선 `(-ty, tx)`를 만들고, 설정 차로 폭 2.3m의 절반인 1.15m를 이동한다. 내부 point의 접선은 앞뒤 point를 함께 사용해 꺾이는 구간의 offset 방향을 완화한다. 연속 link 접속점이 1mm 이내로 겹치면 첫 점을 제거해 하나의 경계로 결합한다.

## 10. DynamicObstacleLinkProjector

입력 객체 중 `motion_state == MOVING`, `heading_valid == true`, 자차 거리 50m 이내인 객체만 처리한다.

후보 link는 현재 link에서 전방으로 연결되는 route와 최대 2회 차선 이동으로 도달 가능한 link의 교집합으로 만든다. 객체 위치를 후보 선분에 벡터화해 투영하고 가장 가까운 link를 고른 뒤, link와 객체 heading 차이가 45°를 넘는 결과는 제외한다.

부호 있는 횡거리는 다음 외적으로 계산한다.

```text
signed_distance = cross(link_vector, obstacle - projection) / |link_vector|
```

link 진행 방향 기준 왼쪽은 양수, 오른쪽은 음수다. 반대 방향 객체는 heading 차이를 π와 비교하는 별도 허용 범위를 둬 link 기하와 객체 진행 방향을 구분한다.

## 11. MACARON 8 내부 연결

```text
/lane_change_candidate_data
        │
        ▼
   cubic_spline.cpp
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
                    │
                    ├── occupancy 기반 후보 평가
                    ├── highway mandatory lane change
                    ├── traffic-light state
                    └── reverse / parking planning
```

`visualization.cpp`와 `visualization.py`는 지도, 후보, 가상 경계, 후방 목표를 RViz marker로 변환한다. `traffic.py`와 후방 인지 노드는 `CurrentLink`의 신호 구간 label과 차선변경 가능 방향을 사용한다.

## 12. 저장소 경계

이 저장소의 `path_planning/link`는 MACARON 8에서 구현한 link 계층의 source snapshot이다. `msg/`는 이 계층이 만들거나 입력으로 참조하는 데이터 구조를 설명하고, `path/MGeo/`는 실제 계산에 사용한 지도 데이터다.

MACARON 8의 localization, perception, spline, occupancy planner 및 차량 제어와의 연결 관계를 문서에 정리했다. 링크 데이터를 이용해 생성한 주행 정보와 전체 시스템에 연결한 과정을 프로젝트 단위로 소개한다.
