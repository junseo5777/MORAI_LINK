# morai_link_global_path

MORAI MGeo의 link/node 데이터를 기반으로 지정한 link 체크포인트를 Dijkstra 알고리즘으로 연결해 폐곡선 전역 경로를 생성하고, GPS/IMU 기반 현재 위치에서 current link와 차선 변경 후보 경로를 계산해 ROS 및 RViz로 시각화를 진행했습니다.

## 구성

```text
.
├── docs/
│   └── images/
├── path/
│   ├── link_set.json
│   └── node_set.json
├── path_planning/
│   ├── dijkstra.py
│   ├── lane_change_candidate.py
│   ├── morai_link.py
│   ├── morai_link_planner.py
│   └── visualization.py
└── sensor/
    └── data_hub.py
```

- `path/link_set.json`: MORAI MGeo link 데이터
- `path/node_set.json`: MORAI MGeo node 데이터
- `path_planning/dijkstra.py`: link 방향 그래프 구성 및 Dijkstra 최단 경로 탐색
- `path_planning/morai_link_planner.py`: 지도 로드, 좌표 변환, 최근접 link 탐색, 일반·고정 경로 생성
- `path_planning/lane_change_candidate.py`: 재사용 가능한 차선 변경 후보 곡선 생성 모듈
- `path_planning/morai_link.py`: map, 고정 route, current link, 차선 변경 후보 데이터를 발행하는 ROS node
- `path_planning/visualization.py`: 수신한 map·route·후보 데이터를 RViz marker로 변환하는 ROS node
- `sensor/data_hub.py`: MORAI GPS/IMU UDP 데이터를 받아 `/current_pose`, `/accel`을 발행하는 ROS node

## 동작 구조

```text
MORAI GPS/IMU UDP
        │
        ▼
sensor/data_hub.py
  ├─ /current_pose
  └─ /accel
        │
        ▼
path_planning/morai_link.py
  ├─ MoraiMapPlanner
  │    └─ dijkstra_link()
  ├─ LaneChangeCandidateGenerator
  ├─ /morai_map_data
  ├─ /morai_route_data
  ├─ /current_link_data
  └─ /lane_change_candidate_data
        │
        ▼
path_planning/visualization.py
        │
        ▼
RViz Marker / MarkerArray topics
```

`lane_change_candidate.py`는 별도의 ROS node가 아니라 순수 계산 모듈입니다. `morai_link.py`가 이 모듈을 import해 후보 경로를 계산하고, `visualization.py`는 발행된 결과를 받아 RViz marker로 표현합니다.

## 주요 기능

### 고정 체크포인트 전역 경로

- `FIXED_ROUTE_LINK_IDXS`에 지정된 link를 순서대로 통과
- 인접한 체크포인트 쌍마다 `dijkstra_link()`를 실행한 뒤 하나의 route로 병합
- 첫 번째와 마지막 체크포인트가 같은 폐곡선 route만 허용
- link 길이를 기본 비용으로 사용하고 차선 변경 edge에는 별도 penalty 적용
- 현재 포함된 MGeo 데이터 기준 40개 route link, 4,392개 path point의 폐곡선 경로 생성 확인

### Current link 추정

1. 전체 고정 route에서 차량과 가장 가까운 route link를 찾습니다.
2. 현재 route link와 다음 route link를 기본 후보로 구성합니다.
3. 각 기본 후보에서 연결 가능한 좌·우 차선 변경 목적 link를 추가합니다.
4. 후보 집합에서 차량과 가장 가까운 link를 다시 선택합니다.
5. 선택한 link와 terminal node의 속성을 `macaron_8/CurrentLink`로 발행합니다.

`/current_link_data`는 `/current_pose`가 들어올 때마다 갱신됩니다. `can_change_left`, `can_change_right`는 단순히 지도에 좌·우 link가 존재하는지가 아니라 현재 위치에서 유효한 차선 변경 후보 곡선을 실제로 만들 수 있는지에 따라 결정됩니다.

### 차선 변경 후보 경로

- 차량 진행 위치보다 `10 m` 앞에서 후보 경로 시작
- 좌·우 차선 변경 목적 link를 따라 `20.0 ± 0.05 m` 곡선 생성
- 3차 Bezier curve를 `101개` point로 샘플링
- 시작 heading 대비 각 segment의 진행 방향 편차를 `45°` 이내로 제한
- 짧은 link에서는 다음 forward link까지 연결해 필요한 길이 확보
- 조건을 만족하는 20 m 경로를 만들 수 없으면 해당 후보를 생성하지 않음
- 현재 link가 바뀌거나 같은 link에서 진행 거리가 `1 m` 이상 달라지면 후보 재계산

### GPS/IMU 처리

- WGS84 경위도를 UTM Zone 52N(`EPSG:32652`) 좌표로 변환
- IMU quaternion을 roll, pitch, yaw로 변환
- `/current_pose`의 `x`, `y`에는 UTM 좌표, `z`에는 yaw(rad) 저장
- IMU linear acceleration의 `x`, `y`를 `/accel`로 발행
- 기본 발행 주기 `10 Hz`

## 실행 화면

### RViz 전역 경로 시각화

MORAI link/node map, 전역 경로, 현재 차량 위치를 RViz에 표시한 화면입니다.

![RViz 전역 경로 시각화](docs/images/rviz-global-path-cropped.png)

### MORAI 시뮬레이터 연동

MORAI 차량의 실제 주행 위치와 RViz에 표시되는 전역 경로를 함께 확인할 수 있습니다.
빨간선이 Dijkstra로 계산된 Global Path이고 분홍색 선이 실제 자동차의 위치가 있는 링크를 나타냅니다.

![MORAI 시뮬레이터와 RViz 전역 경로](docs/images/morai-rviz-driving-cropped.png)

### 회전교차로 경로

회전교차로 구간에서 link 기반으로 생성된 경로와 차량 위치를 비교한 화면입니다.

![회전교차로의 MORAI 시뮬레이터와 RViz 경로](docs/images/morai-rviz-roundabout-cropped.png)

### RViz 색상

- 초록색: MGeo map link
- 빨간색: 생성된 global path
- 븐홍색: current link
- 청록색: 좌측 차선 변경 후보
- 주황색: 우측 차선 변경 후보
- 노란색 화살표: ego vehicle 위치와 yaw

## 주요 ROS topic

| Topic | Type | Description |
| --- | --- | --- |
| `/current_pose` | `geometry_msgs/Point` | 현재 차량 위치. `x`, `y`는 UTM, `z`는 yaw(rad) |
| `/accel` | `geometry_msgs/Point` | IMU linear acceleration의 `x`, `y` |
| `/morai_map_data` | `std_msgs/String` | link/node map JSON, latched publish |
| `/morai_route_data` | `std_msgs/String` | 고정 route link와 path point JSON, latched publish |
| `/current_link_data` | `macaron_8/CurrentLink` | 현재 link 속성과 실제 차선 변경 후보 존재 여부 |
| `/lane_change_candidate_data` | `std_msgs/String` | 좌·우 후보 방향, 목적 link, 후보 path JSON, latched publish |
| `/global_path_marker` | `visualization_msgs/Marker` | global path line marker |
| `/morai_link_markers` | `visualization_msgs/MarkerArray` | MGeo map link/node marker |
| `/dijkstra_route_markers` | `visualization_msgs/MarkerArray` | route link 및 시작·종료 point marker |
| `/ego_vehicle_marker` | `visualization_msgs/Marker` | 현재 차량 위치·yaw arrow marker |
| `/current_link_marker` | `visualization_msgs/Marker` | 현재 차량이 위치한 link 강조 marker |
| `/lane_change_candidate_markers` | `visualization_msgs/MarkerArray` | 좌·우 차선 변경 후보 line marker |

## 설정 위치

### 고정 route 및 current link

`path_planning/morai_link.py` 상단에서 다음 값을 변경할 수 있습니다.

```python
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
```

- `MAP_CREATE`: `True`일 때 route의 UTM `x`, `y`를 `path/donggul.npy`로 저장
- `LANE_CHANGE_LOOKAHEAD_M`: Dijkstra graph에 차선 변경 edge를 추가할 때 필요한 source/destination link의 최소 전방 거리
- `CURRENT_LINK_LOOKAHEAD_ROUTE_COUNT`: current link 후보에 포함할 다음 route link 개수
- `LANE_CHANGE_CANDIDATE_START_AHEAD_M`: 차량 위치에서 후보 경로 시작점까지의 전방 거리
- `LANE_CHANGE_CANDIDATE_UPDATE_DISTANCE_M`: 같은 link에서 후보 경로를 다시 계산할 최소 진행 거리
- `FIXED_ROUTE_LINK_IDXS`: 순서대로 통과할 체크포인트 link ID. 폐곡선 route이므로 첫 ID와 마지막 ID가 같아야 함

MORAI local 좌표와 UTM/RViz 변환 원점은 `MoraiMapPublisher.__init__()`에 있습니다.

```python
self.origin_utm_x = 302595.0
self.origin_utm_y = 4124145.0
self.use_utm_offset = True
self.lane_change_penalty = 20.0
```

### 차선 변경 후보

`path_planning/lane_change_candidate.py` 상단에서 후보 곡선 조건을 변경할 수 있습니다.

```python
START_AHEAD_OF_VEHICLE_M = 10.0
CANDIDATE_PATH_LENGTH_M = 20.0
CANDIDATE_LENGTH_TOLERANCE_M = 0.05
CANDIDATE_CURVE_SAMPLES = 101
MAX_CANDIDATE_HEADING_DEVIATION_DEG = 45.0
```

`morai_link.py`에서 generator를 생성할 때 `LANE_CHANGE_CANDIDATE_START_AHEAD_M`를 전달하므로 실제 실행 시 후보 시작 거리는 `morai_link.py` 설정을 따릅니다.

### GPS/IMU UDP

`sensor/data_hub.py`의 UDP 설정은 다음과 같습니다.

```python
GPS_IP = '127.0.0.1'
GPS_PORT = 9090
IMU_IP = '127.0.0.1'
IMU_PORT = 9088
```

## 커스텀 메시지

`morai_link.py`는 현재 차량이 주행 중인 link 정보를 전달하기 위해 `macaron_8/CurrentLink`를 사용합니다.

```text
string link_type
string link_idx
float64 max_speed
bool on_stop_line
string traffic_light_id
string related_signal
bool can_change_left
bool can_change_right
```

| Field | Source / Meaning |
| --- | --- |
| `link_type` | 현재 link의 MGeo `link_type` |
| `link_idx` | 현재 link ID |
| `max_speed` | 현재 link의 제한속도 |
| `on_stop_line` | 현재 link의 `to_node_idx`가 가리키는 terminal node의 정지선 여부 |
| `traffic_light_id` | terminal node에 연결된 신호등 ID |
| `related_signal` | 현재 link의 진행 방향 신호 정보 |
| `can_change_left` | 현재 위치에서 유효한 좌측 후보 곡선이 생성됐는지 여부 |
| `can_change_right` | 현재 위치에서 유효한 우측 후보 곡선이 생성됐는지 여부 |

`on_stop_line`과 `traffic_light_id`는 실시간 신호 상태가 아니라 지도에 저장된 terminal node metadata입니다.
