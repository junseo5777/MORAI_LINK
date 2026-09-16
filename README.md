## 한눈에 보기

| 항목 | 결과 |
| --- | ---: |
| MGeo link / node | 634개 / 480개 |
| 전체 지도 선분 | 74,782개 |
| 순환 route | 40개 항목, 39개 고유 link, 약 2.185km |
| 현재 link 검색 범위 | route와 주변 차로 55개 link, 7,560개 선분 |
| 고속도로 차선변경 후보 범위 | 기준 5개 link에서 확장한 14개 link |
| 현재 link 전환 조건 | 같은 새 link가 10회 연속 관측될 때 확정 |
| 후보 경로 | center/left/right, 기본 40m, 약 0.5m 간격 |
| 후진 기준점 | link를 역추적한 기본 8m 후방 위치와 yaw |

제가 담당한 핵심은 다음과 같습니다.

- MGeo 좌표와 속성을 주행용 데이터로 정규화
- 현재 차량이 속한 link를 거리·yaw·graph 관계로 추정
- 평행 차로와 교차로에서 발생하는 link 오인식을 상태 기반으로 안정화
- 현재 차로 유지 및 고속도로 좌·우 차선변경 후보 생성
- 정지선·신호·교차로·톨게이트 등 도로 문맥 발행
- 후진 기준점, 가상 차선 경계, 동적 객체의 link 상대정보 생성
- 반복 선분 투영 계산을 NumPy로 벡터화하고 불필요한 캐시 제거

## 해결하려 한 문제

MGeo에는 도로 중심선과 연결 관계가 있지만, 차량의 현재 위치와 가장 가까운 중심선 하나를 고르는 것만으로는 안정적인 주행 정보를 만들기 어렵습니다.

- 평행한 여러 차로가 가까이 있어 옆 차로로 link가 순간 전환될 수 있습니다.
- 교차로와 분기점에서는 여러 link가 같은 node에 연결됩니다.
- 일부 MGeo 차선변경 속성과 목적 link가 실제 주행 경로와 맞지 않았습니다.
- 짧은 link에서는 40m 후보가 다음 link까지 이어져야 합니다.
- GPS와 yaw가 흔들릴 때 현재 link와 후보 경로가 프레임마다 바뀔 수 있습니다.
- 장애물의 절대좌표만으로는 어느 차로의 어느 쪽에 있는지 판단하기 어렵습니다.

따라서 단순 최근접 검색이 아니라 `지도 정규화 → 제한된 후보 검색 → 방향 검증 → 전환 안정화 → 도로 문맥 및 경로 생성` 계층으로 구성했습니다.

## 전체 데이터 흐름

```text
MGeo link/node ──> 좌표·속성 정규화 ──> graph / NumPy 선분 index
                                              │
차량 UTM pose + yaw ──> 현재 link 추정 ───────┼─> CurrentLink 도로 문맥
                         │                    ├─> center/left/right 후보
                         │                    ├─> 차선변경 목적 link chain
                         │                    ├─> 8m 후방 기준점
                         │                    └─> 가상 차선 경계
                         │
동적 객체 ───────────────┴──────────────────────> link 소속·횡거리·방향 차이
                                              │
                                              ▼
                         spline / occupancy / mission planner
```

[`morai_link.py`](link/morai_link.py)는 ROS 입출력과 상태를 관리하고, 나머지 모듈은 지도 graph와 기하 계산을 분리해 담당합니다.

## 1. MGeo 정규화와 검색 index

[`morai_link_planner.py`](link/morai_link_planner.py)는 [`link_set.json`](path/MGeo/link_set.json)과 [`node_set.json`](path/MGeo/node_set.json)을 읽어 모든 링크 계산의 기준을 만듭니다.

### 좌표계 통합

MGeo 로컬 좌표에 지도 원점을 더해 localization과 같은 UTM 좌표로 변환했습니다.

```text
origin_utm_x = 302595.0
origin_utm_y = 4124145.0

utm_x = morai_x + origin_utm_x
utm_y = morai_y + origin_utm_y
```

각 point에는 원본 MORAI 좌표, UTM 좌표, RViz/기하 계산 좌표를 함께 보관합니다. 형식이 잘못된 point와 point가 2개 미만인 link는 기하 계산에서 제외하고, 유효하지 않은 폭은 기본값으로 보정합니다.

### graph와 선분 index

지도는 다음 lookup으로 변환했습니다.

- link ID → link 데이터
- node ID → node 데이터
- 시작/종료 node → 연결 link 목록
- link → 좌우 인접 차로 집합
- link → 최근접 검색용 선분 index
- 전체 선분의 시작점·방향벡터·길이·heading NumPy 배열

차량 위치가 들어올 때마다 전체 JSON을 순회하지 않고, route에서 도달 가능한 link만 모은 배열에 차량점을 일괄 투영합니다.

### 원본 지도 보정

원본 JSON을 직접 수정하지 않고 보정 규칙을 코드 상단에 분리했습니다. 어떤 지도 예외가 주행 판단에 개입하는지 추적하기 위한 선택입니다.

| 예외 | 처리 |
| --- | --- |
| 왼쪽 차선변경 금지 | `A2256W000418`, `431`, `435`, `423` |
| 오른쪽 차선변경 금지 | `A2256W000409`, `423`, `422`, `408` |
| 왼쪽 목적 link 보완 | `319 → 318`, `849 → 054` |
| 오른쪽 목적 link 보완 | `318 → 319`, `054 → 849` |
| link type 보정 | `054 → 4`, `318/319 → 6` |
| 진행 신호 보정 | `182 → left_unprotected` |

이 보정은 현재 link, 차선변경 가능 방향, 후보 목적 차로가 서로 다른 기준을 사용하지 않도록 지도 로딩 단계에서 한 번만 적용합니다.

## 2. 현재 link 추정과 전환 안정화

route는 40개 항목, 39개 고유 link로 구성된 약 2.185km 순환 경로입니다. 실제 주행에서는 route 옆 차로로 이동하므로 route만 검색하지 않고, 최대 3회의 좌우 차선 이동으로 도달 가능한 범위까지 확장했습니다.

현재 link는 다음 순서로 찾습니다.

1. 첫 pose에서는 55개 whitelist 전체 검색
2. 이후에는 현재 link, 연결 link, 다음 link와 그 주변 차로를 우선 검색
3. 차량점을 모든 후보 선분에 직교 투영해 최단거리 계산
4. yaw가 있으면 진행 방향 차이가 90° 이내인 선분 우선
5. 지역 검색 결과가 없거나 4m보다 멀거나 heading filter가 실패하면 전체 whitelist 재탐색
6. 같은 새 link가 기본 10회 연속 관측될 때 현재 link 전환

### 순간 오인식 방지

```text
현재 link 재관측 ──> pending 초기화 ──> 현재 link 유지

새 link 관측 ──────> pending link/count 기록
                           │
                           ├─ 다른 후보: count를 1로 재시작
                           ├─ 기존 link: pending 취소
                           └─ 같은 후보 10회: link 전환 확정
```

전환 확인 중에는 새 후보를 즉시 사용하지 않고 기존 link 하나에 차량을 다시 투영합니다. 이 처리로 현재 link의 진행도와 후보 경로가 옆 차로로 순간 이동하는 것을 막았습니다.

방향 조건을 만족하는 선분이 하나도 없을 때는 위치 검색 자체를 실패시키지 않고 거리 기반 결과로 fallback한 뒤, 상위 계층이 전체 whitelist 재탐색 여부를 결정합니다.

## 3. 현재 도로 문맥 생성

현재 link와 종료 node를 결합해 [`CurrentLink.msg`](msg/CurrentLink.msg)를 구성합니다.

| 필드 | 생성 기준 | 사용 목적 |
| --- | --- | --- |
| `link_idx` | 확정된 현재 link | 현재 차로 및 mission 구간 식별 |
| `link_type` | 보정된 MGeo type과 교차로 preview | 일반도로·교차로 구분 |
| `max_speed` | link 제한속도 | 도로 속도 정보 |
| `on_stop_line` | 종료 node 속성 | 정지선 판단 |
| `traffic_light_id` | 종료 node 속성 | 지도 신호등과 perception 연결 |
| `related_signal` | 직진·좌회전 등 진행 신호 | 방향별 신호 판단 |
| `can_change_left/right` | 보정된 MGeo 차선변경 속성 | 차선변경 허용 방향 |
| `link_information` | link ID별 도로·mission label | 신호 형태와 구간 상태 전환 |
| `not_avoid` | 지정된 회피 비허용 link 여부 | 회피를 제한해야 하는 구간 표시 |

`can_change_left/right`는 후보 경로 생성 성공 여부와 분리했습니다. 후보 발행 구간이 제한되더라도 지도에서 보정한 차선 관계 자체는 일관되게 전달합니다.

### 거리 기반 preview

다음 link가 교차로 type `1`이면 현재 link 종점 전 기본 13m부터 `link_type=1`을 미리 발행합니다. 긴 신호 link는 전체 구간에서 신호 label을 내보내지 않고 중심선 잔여거리를 계산해 종점 부근에서만 활성화합니다.

| Link | `link_information` | 활성 범위 |
| --- | --- | ---: |
| `A2256W000213` | `three_light_traffic_signal` | 종점 전 25m |
| `A2256W000308` | `four_light_traffic_signal` | 종점 전 20m |

그 밖에 회전교차로 진입, 고속도로 진입, 차로 번호, 톨게이트, 가드레일 종점도 link별 label로 표현합니다.

## 4. 주행 후보 경로

[`lane_change_candidate.py`](link/lane_change_candidate.py)는 차량의 현재 진행도에서 최대 세 후보를 생성합니다.

| Candidate ID | 종류 | 생성 방식 |
| ---: | --- | --- |
| 0 | center | 현재 투영점부터 link 중심선을 따라 전방 연결 |
| 1 | left | 시작점과 왼쪽 목적 link의 목표점을 직선으로 연결 |
| 2 | right | 시작점과 오른쪽 목적 link의 목표점을 직선으로 연결 |

center 후보는 전체 route에서 만들고, 좌·우 후보는 실제 차선변경 판단이 필요한 고속도로 구간으로 제한했습니다. 기준 route 5개 link에서 주변 차로까지 확장한 14개 link가 대상입니다.

### 목적점과 길이 결정

기본 후보 길이는 40m입니다. 목적 link가 짧으면 연결 가능한 다음 link까지 chain을 확장하고, 시작점을 중심으로 반지름 40m인 원과 목적 link 선분의 첫 전방 교점을 해석적으로 구합니다. 찾은 목적점까지 직선을 약 0.5m 간격으로 표본화합니다.

반복적인 곡선 길이 탐색 대신 선분-원 교점을 직접 계산해 계산량과 분기 수를 줄였습니다. 목적 link chain 전체가 40m보다 짧은 경우에는 최소 16m를 확보했을 때만 축소 후보를 허용합니다.

### 구간별 예외처리

- `A2256W000445`의 좌·우 후보는 기본 40m 대신 20m로 생성합니다.
- `A2256W000445`, `A2256W000422`에서 시작한 후보는 직접 인접한 목적 link 안에서 끝나야 합니다.
- 위 구간에서 후보 종점이 다음 구간으로 넘어가면 짧게 잘라 대체하지 않고 후보 자체를 제외합니다.
- graph 분기에서 route 우선순위가 없고 후보가 여러 개면 임의의 link를 고르지 않습니다.
- link chain 순환은 visited 집합과 최대 link 수로 차단합니다.
- 연속 link의 중복 접속점과 길이가 0인 선분은 기하 계산에서 제외합니다.

### 두 종류의 출력

- `/lane_change_candidate_data`: 현재 위치에서 시작하는 center/left/right 실제 경로
- `/lane_change_candidate_link`: 차선변경 후 유지할 목적 link의 원본 중심선과 link 순서

실제 후보는 10Hz에서 현재 위치를 기준으로 매번 다시 계산합니다. 과거에는 항상 강제 갱신하면서 결과를 캐시에 쓰고 있어 재사용되지 않던 캐시와 관련 함수를 제거했습니다. 목적 link 원본 중심선은 차량 진행도와 무관하므로 link 구성 signature가 바뀔 때만 latched topic을 갱신합니다.

## 5. 계산 최적화와 동일성 검증

차선변경 후보 계산에서 반복 호출되는 최근접 선분 투영을 Python 선분별 반복문에서 NumPy 배열 연산으로 바꿨습니다.

```text
모든 선분의 투영 비율 계산
→ [0, 1] 범위 제한
→ 모든 투영점 계산
→ 거리 제곱 계산
→ 최소 거리 선분 선택
```

링크 geometry를 만들 때 X/Y 좌표, 선분 벡터, 선분 길이를 `float64` 배열로 저장하고 이후 계산에서 재사용합니다. 최소 선분을 선택한 뒤 최종 진행거리는 기존 스칼라 연산 순서로 다시 계산해 출력 변화를 최소화했습니다.

로컬 환경에서 실제 MGeo를 사용해 측정한 결과입니다.

| 항목 | 기존 | 변경 후 |
| --- | ---: | ---: |
| 최근접 진행도 계산 | 약 143μs | 약 65μs |
| 후보 1회 계산 | 약 1.52ms | 약 1.08ms |

- 최근접 계산 53,848건 비교: 불일치 0건
- 후보 생성 91개 조건 비교: 불일치 0건
- 대표 후보 결과 JSON hash: 기존과 동일

후보 선택 조건, link 연결 순서, 길이 제한, 출력 형식은 변경하지 않고 거리 계산 방식만 벡터화했습니다.

## 6. 후진 기준점

[`reverse_link.py`](link/reverse_link.py)는 현재 위치에서 link point를 역순으로 누적해 기본 8m 뒤의 위치와 진행 yaw를 계산합니다. 현재 link 안에서 거리가 부족하면 시작 node로 들어오는 이전 link까지 역추적합니다.

분기점에서는 다음 우선순위로 이전 link를 선택합니다.

1. 실제 pose 처리 중 관측한 직전 link
2. route에 정의된 이전 link
3. 후보가 하나뿐인 이전 link
4. 현재 link 시작 heading과 가장 자연스럽게 이어지는 link

순환 graph는 visited 집합과 최대 hop 수로 종료하고, 같은 좌표가 연속될 때는 yaw 계산에 사용할 다음 유효 point를 탐색합니다.

## 7. 가상 차선 경계

[`virtual_lane_boundary.py`](link/virtual_lane_boundary.py)는 지정 link 중심선을 왼쪽으로 1.15m 이동해 점유격자가 사용할 가상 경계를 만듭니다.

```text
가상 좌측 경계  ─────────────────────
                  1.15m
link 중심선      ─────────────────────>
```

내부 point는 앞뒤 point를 함께 사용해 접선을 구하므로 꺾이는 구간에서 offset 방향이 급변하지 않습니다. 접선 길이가 0이면 앞뒤의 유효 선분을 사용하고, 연결된 link의 접속점이 1mm 이내로 겹치면 첫 point를 제거해 하나의 연속 경계로 만듭니다.

적용 구간은 `A2256W000304 → A2256W000148`과 `A2256W000213`입니다.

## 8. 동적 객체의 link 투영

[`dynamic_obstacle_link_projector.py`](link/dynamic_obstacle_link_projector.py)는 perception 객체의 절대좌표를 도로 기준 정보로 변환합니다.

처리 대상은 다음 조건을 모두 만족하는 객체입니다.

- 이동 상태가 `MOVING`
- 진행 방향이 유효함
- 자차에서 50m 이내
- 선택된 link와 heading 오차가 45° 이내
- 반대 방향 heading 허용 범위에 포함되지 않음

현재 link에서 전방으로 연결되고 최대 2회 차선 이동으로 도달 가능한 link만 후보로 사용합니다. 먼저 기하학적으로 가장 가까운 link를 확정한 뒤 방향이 맞지 않으면 객체를 제외합니다. 방향만 맞는 더 먼 평행 link로 객체를 강제로 옮기지 않기 위한 처리입니다.

결과에는 다음 정보가 포함됩니다.

- 객체 track ID와 최근접 link ID
- link 중심선 위 UTM 투영점
- 자차와 객체 사이 평면거리
- link 진행 방향 기준 부호 있는 횡거리
- link와 객체의 부호 있는 heading 차이

왼쪽 횡거리는 양수, 오른쪽은 음수입니다. 결과는 자차 거리와 track ID 순으로 정렬해 동일 입력에서 순서를 결정적으로 유지합니다.

## 생성 데이터와 시스템 연결

| 생성 데이터 | 내용 | 연결 대상 |
| --- | --- | --- |
| `/morai_map_data` | 정규화된 전체 link/node | RViz 지도 시각화 |
| `/current_link_data` | 현재 link와 도로 문맥 | tracking, highway, traffic, rear vision |
| `/lane_change_candidate_data` | center/left/right 기하 경로 | spline 보간 및 후보 평가 |
| `/lane_change_candidate_link` | 좌·우 목적 link 중심선 | 차선변경 이후 목적 차로 추종 |
| `/reverse_link_point` | link 기준 후방 위치와 yaw | 후진·주차 경로계획 |
| `/current_link_virtual_lane_boundaries` | 가상 좌측 경계 | 점유격자 기반 경로 제약 |
| `/dynamic_obstacle_link_data` | 동적 객체의 link 소속과 상대 위치 | 객체-차로 관계 표현 인터페이스 |

동적 객체 link 투영은 계산과 메시지 발행까지 구현한 독립 계층이며, 최종 planner 소비 연결은 이 저장소의 범위에 포함하지 않았습니다.

## 검증 결과

실제 MGeo와 route link의 원본 point 중간 위치를 사용해 기하 결과를 확인했습니다.

| 검증 항목 | 결과 |
| --- | ---: |
| 지도 로드 | 634 link, 480 node |
| 전체 선분 수 | 74,782 |
| route 검색 closure | 55 link, 7,560 선분 |
| route 중간점 center 후보 | 39 / 39 |
| 고속도로 구간 left 후보 | 4개 |
| 기본 8m 후방 목표 | 39 / 39 |
| 연속 가상 경계 (`304 → 148`) | 502 point |

예외 상황에서는 잘못된 결과를 임의로 만드는 대신 빈 후보 또는 `None`을 반환하도록 했습니다. 대표적으로 유효하지 않은 좌표, 2개 미만의 point, 길이 0인 geometry, 연결이 모호한 분기, 순환 link, 목표 길이가 부족한 차선변경 경로를 방어합니다.

## 결과 시각화

MGeo 도로망과 생성 경로를 RViz에 표시하고 MORAI 차량 위치와 비교했습니다.

### 전역 route와 link graph

![RViz 전역 경로](docs/images/rviz-global-path-cropped.png)

### 실제 주행 위치와 RViz 경로 정합

![MORAI와 RViz 주행 위치 비교](docs/images/morai-rviz-driving-cropped.png)

### 회전교차로의 link 연결

![회전교차로 link 경로](docs/images/morai-rviz-roundabout-cropped.png)

## 코드 구성

```text
link/
├── morai_link.py
│   └── ROS 데이터 흐름, 현재 link 상태, 하위 계산 모듈 조합
├── morai_link_planner.py
│   └── MGeo 로드, 좌표 변환, graph/index, 최근접 선분 검색
├── lane_change_candidate.py
│   └── center/left/right 후보와 목적 link chain 생성
├── reverse_link.py
│   └── 이전 link 선택과 후방 목표점 계산
├── virtual_lane_boundary.py
│   └── 중심선 offset 기반 가상 경계 생성
└── dynamic_obstacle_link_projector.py
    └── 이동 객체를 주행 가능한 link에 투영
```

`msg/`에는 이 계층이 생성하거나 입력으로 참조하는 ROS 메시지 구조를, `path/MGeo/`에는 실제 계산에 사용한 지도 데이터를 담았습니다.
