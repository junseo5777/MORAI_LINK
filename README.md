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
- `path_planning/dijkstra.py`: link 기반 Dijkstra 최단 경로 탐색
- `path_planning/morai_link_planner.py`: MORAI map 로드, 좌표 변환, 시작/목표 link 탐색
- `path_planning/morai_link.py`: ROS node로 map/route/current link 정보 publish
- `path_planning/visualization.py`: RViz marker 시각화
- `sensor/data_hub.py`: MORAI GPS/IMU UDP 데이터를 받아 `/current_pose`, `/accel` publish

## 주요 기능

- MORAI link/node JSON 로드
- MORAI local 좌표를 UTM 및 RViz 좌표로 변환
- 현재 위치 `/current_pose` 기준 시작 link 탐색
- GPS/IMU 데이터를 UTM 좌표 및 yaw로 변환해 `/current_pose` publish
- 목표 UTM 좌표까지 link 기반 Dijkstra 경로 생성
- 차선 변경 가능 link를 graph edge로 반영
- 생성된 경로와 map link를 RViz marker로 publish
- 현재 차량 위치와 가장 가까운 route link 정보 publish

## 주요 ROS topic

| Topic | Type | Description |
| --- | --- | --- |
| `/current_pose` | `geometry_msgs/Point` | 현재 차량 위치. `x`, `y`는 UTM, `z`는 yaw로 사용 |
| `/accel` | `geometry_msgs/Point` | IMU linear acceleration |
| `/morai_map_data` | `std_msgs/String` | link/node map data JSON |
| `/morai_route_data` | `std_msgs/String` | 생성된 route data JSON |
| `/current_link_data` | `macaron_8/CurrentLink` | 현재 차량과 가장 가까운 route link 정보 |
| `/global_path_marker` | `visualization_msgs/Marker` | 전역 경로 marker |
| `/morai_link_markers` | `visualization_msgs/MarkerArray` | MORAI map link marker |
| `/dijkstra_route_markers` | `visualization_msgs/MarkerArray` | Dijkstra route link marker |
| `/ego_vehicle_marker` | `visualization_msgs/Marker` | 현재 차량 위치 marker |


## 설정 위치

`path_planning/morai_link.py` 상단의 값을 수정해서 목표 지점과 옵션을 변경할 수 있습니다.

```python
GOAL_UTM_X = 302395.77395635366
GOAL_UTM_Y = 4124489.0898724506

MAP_CREATE = False
USE_UTM_OFFSET = True
USE_START_YAW_FILTER = True
HEADING_FILTER_DEG = 90.0
LANE_CHANGE_PENALTY = 30.0
```

- `GOAL_UTM_X`, `GOAL_UTM_Y`: 목표 UTM 좌표
- `MAP_CREATE`: `True`일 때 생성 경로를 `.npy`로 저장
- `USE_UTM_OFFSET`: RViz 표시 시 UTM origin offset 적용 여부
- `USE_START_YAW_FILTER`: 시작 link 탐색 시 차량 yaw 반영 여부
- `HEADING_FILTER_DEG`: 시작 link 후보 heading 허용 각도
- `LANE_CHANGE_PENALTY`: 차선 변경 edge에 추가되는 비용

## 커스텀 메시지

morai_link.py는 현재 차량이 주행 중인 link 정보를 전달하기 위해 macaron_8.msg.CurrentLink 커스텀 메시지를 사용합니다.
이 메시지는 link ID, 제한속도, 정지선 여부, 신호등 ID, 진행 방향 신호 정보를 /current_link_data topic으로 publish하기 위한 메시지입니다.

이를 통해 주행 제어 노드는 현재 차량이 위치한 도로 link의 제한속도와 신호 관련 정보를 topic으로 받아 활용할 수 있습니다.

