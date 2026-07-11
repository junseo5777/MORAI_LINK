# morai_link_global_path

MORAI 자율주행 시뮬레이션 환경에서 제공되는 link/node 데이터를 이용해 현재 차량 위치에서 목표 지점까지 전역 경로를 생성하고, RViz로 확인하기 위한 경로 계획 코드입니다.

이 프로젝트는 MORAI 기반 자율주행 대회 및 시뮬레이션 주행 환경에서 사용할 수 있는 전역 경로 생성 모듈을 정리한 것입니다. 차량의 GPS/IMU 기반 현재 위치를 UTM 좌표로 변환하고, 도로 link graph를 기반으로 Dijkstra 최단 경로 탐색을 수행하여 목표 지점까지의 주행 경로를 생성합니다.

## 구성

```text
.
├── path/
│   ├── link_set.json
│   └── node_set.json
├── path_planning/
│   ├── dijkstra.py
│   ├── morai_link.py
│   ├── morai_link_planner.py
│   └── visualization.py
└── sensor/
    └── data_hub.py
```

- `path/link_set.json`: MORAI link 정보
- `path/node_set.json`: MORAI node 정보
- `path_planning/dijkstra.py`: 전방 거리 조건을 포함한 link 기반 Dijkstra 최단 경로 탐색
- `path_planning/morai_link_planner.py`: MORAI map 로드, 좌표 변환, 시작/목표 link 탐색, 차선 변경 경로 생성
- `path_planning/morai_link.py`: ROS node로 map/route/current link 및 좌우 차선 변경 가능 여부 publish
- `path_planning/visualization.py`: map, 전역 경로, 현재 link 및 차량 위치 RViz marker 시각화
- `sensor/data_hub.py`: MORAI GPS/IMU UDP 데이터를 받아 `/current_pose`, `/accel` publish

## 주요 기능

- MORAI link/node JSON 로드
- MORAI local 좌표를 UTM 및 RViz 좌표로 변환
- 현재 위치 `/current_pose` 기준 시작 link 탐색
- GPS/IMU 데이터를 UTM 좌표 및 yaw로 변환해 `/current_pose` publish
- 목표 UTM 좌표까지 link 기반 Dijkstra 경로 생성
- 차선 변경 가능 link를 graph edge로 반영하고 차선 변경 전후 최소 전방 거리 검사
- 생성된 경로와 map link를 RViz marker로 publish
- 현재 link 추정 시 다익스트라 경로의 현재·다음 route link뿐만 아니라, 각 route link에서 실제 차선 변경이 가능한 좌우 link까지 후보에 포함
- 현재 link의 제한속도, 신호 정보와 좌우 차선 변경 가능 여부 publish

## 실행 화면

### RViz 전역 경로 시각화

MORAI link/node map과 Dijkstra로 생성한 전역 경로를 RViz에 표시한 화면입니다.

![RViz 전역 경로 시각화](docs/images/rviz-global-path-cropped.png)

### MORAI 시뮬레이터 연동

MORAI 차량의 실제 주행 위치와 RViz에 표시되는 전역 경로를 함께 확인할 수 있습니다.

![MORAI 시뮬레이터와 RViz 전역 경로](docs/images/morai-rviz-driving-cropped.png)

### 회전교차로 경로

회전교차로 구간에서 link 기반으로 생성된 경로와 차량 위치를 비교한 화면입니다.

![회전교차로의 MORAI 시뮬레이터와 RViz 경로](docs/images/morai-rviz-roundabout-cropped.png)

## 주요 ROS topic

| Topic | Type | Description |
| --- | --- | --- |
| `/current_pose` | `geometry_msgs/Point` | 현재 차량 위치. `x`, `y`는 UTM, `z`는 yaw로 사용 |
| `/accel` | `geometry_msgs/Point` | IMU linear acceleration |
| `/morai_map_data` | `std_msgs/String` | link/node map data JSON |
| `/morai_route_data` | `std_msgs/String` | 생성된 route data JSON |
| `/current_link_data` | `macaron_8/CurrentLink` | 다익스트라 route link와 차선 변경 가능한 좌우 link 후보 중 현재 차량과 가장 가까운 link 정보 |
| `/global_path_marker` | `visualization_msgs/Marker` | 전역 경로 marker |
| `/morai_link_markers` | `visualization_msgs/MarkerArray` | MORAI map link marker |
| `/dijkstra_route_markers` | `visualization_msgs/MarkerArray` | Dijkstra route link marker |
| `/ego_vehicle_marker` | `visualization_msgs/Marker` | 현재 차량 위치 marker |
| `/current_link_marker` | `visualization_msgs/Marker` | 현재 차량이 위치한 link 강조 marker |


## 설정 위치

`path_planning/morai_link.py` 상단의 값을 수정해서 목표 지점과 옵션을 변경할 수 있습니다.

```python
GOAL_UTM_X = 302592.98273088294
GOAL_UTM_Y = 4124264.524556471

MAP_CREATE = False
LANE_CHANGE_LOOKAHEAD_M = 20.0
CURRENT_LINK_LOOKAHEAD_ROUTE_COUNT = 1
```

- `GOAL_UTM_X`, `GOAL_UTM_Y`: 목표 UTM 좌표
- `MAP_CREATE`: `True`일 때 생성 경로를 `.npy`로 저장
- `LANE_CHANGE_LOOKAHEAD_M`: 차선 변경 전후에 확보할 최소 전방 거리와 목표점 거리(m)
- `CURRENT_LINK_LOOKAHEAD_ROUTE_COUNT`: 현재 link 후보에 포함할 다음 route link 개수

그 외 고정 설정은 `MoraiMapPublisher.__init__()`에서 변경할 수 있습니다.

```python
self.origin_utm_x = 302595.0
self.origin_utm_y = 4124145.0
self.use_utm_offset = True
self.use_start_yaw_filter = True
self.heading_filter_deg = 90.0
self.lane_change_penalty = 20.0
```

- `origin_utm_x`, `origin_utm_y`: MORAI local 좌표를 UTM으로 변환할 때 사용하는 원점
- `use_utm_offset`: RViz 표시 시 UTM origin offset 적용 여부
- `use_start_yaw_filter`: 시작 link 탐색 시 차량 yaw 반영 여부
- `heading_filter_deg`: 시작 link 후보 heading 허용 각도
- `lane_change_penalty`: 차선 변경 edge에 추가되는 비용

`sensor/data_hub.py`의 UDP 설정은 다음과 같습니다.

```python
GPS_IP = '127.0.0.1'
GPS_PORT = 9090
IMU_IP = '127.0.0.1'
IMU_PORT = 9088
```

## 커스텀 메시지

morai_link.py는 현재 차량이 주행 중인 link 정보를 전달하기 위해 macaron_8.msg.CurrentLink 커스텀 메시지를 사용합니다.
이 메시지는 link ID, 제한속도, 정지선 여부, 신호등 ID, 진행 방향 신호, 좌우 차선 변경 가능 여부를 /current_link_data topic으로 publish하기 위한 메시지입니다.

```text
string link_idx
float64 max_speed
bool on_stop_line
string traffic_light_id
string related_signal
bool can_change_left
bool can_change_right
```

이를 통해 주행 제어 노드는 현재 차량이 위치한 도로 link의 제한속도, 신호 관련 정보와 좌우 차선 변경 가능 여부를 topic으로 받아 활용할 수 있습니다.
