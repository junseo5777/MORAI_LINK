# Implementation review

이 문서는 현재 source extract를 정리하며 확인한 동작, 수정 사항과 남은 위험을 기록한다.

## 확인 완료

- Python 6개 모듈 AST 구문 검사 통과
- ROS Noetic 환경에서 모든 모듈 import 성공
- MGeo link 634개, node 480개 로드 성공
- route 40개 항목 전부 지도에 존재
- 연속 route 링크의 node 연결 단절 없음
- 좌우 차선변경 reference에 존재하지 않는 목적 링크 없음
- route 중간점 기준 39개 모든 고유 링크에서 center 후보 생성
- 같은 검사에서 left 후보 4개, right 후보 6개 생성
- route 중간점 기준 모든 링크에서 8m reverse target 생성
- 가상 경계 그룹 `A2256W000304 -> A2256W000148`의 접속점 일치

## 정리본에서 수정한 사항

### 동적 객체 토픽

MACARON 8의 원본 `morai_link.py`는 `/yolo_lidar_fusion/dynamic_objects`를 구독하지만 fusion node의 실제 출력은 `/yolo_lidar_fusion/dynamic_points`다.

이 source extract에서는 기본 입력을 실제 출력에 맞게 `dynamic_points`로 수정했다.

## 남은 주의사항

### 1. 현재 링크 검색에 최종 거리 제한이 없다

`CURRENT_LINK_REACQUIRE_DISTANCE_M=4.0`은 전체 검색으로 전환하는 기준이지 결과를 거부하는 기준이 아니다. GPS가 초기화되지 않았거나 좌표계가 틀려도 whitelist 중 하나가 선택된다.

확인용으로 UTM `(0, 0)`을 입력했을 때 약 4,134km 떨어진 링크가 반환됐다.

권장 개선:

- `current_link_acceptance_distance_m`을 별도로 추가
- 최종 결과가 한계를 넘으면 current link와 latched 후보를 명시적으로 clear
- pose freshness와 finite/range 검증 추가

### 2. heading filter가 강한 선택 편향을 만든다

허용 각도 안의 선분이 하나라도 있으면 다른 모든 선분은 거리와 무관하게 제외된다. 잘못된 yaw가 들어오면 가까운 링크보다 먼 방향 일치 링크가 선택될 수 있다.

거리와 heading을 hard filter가 아닌 가중 score로 결합하거나, 거리 gate를 먼저 적용하는 방식이 안전하다.

### 3. intersection type 발행 범위 확인이 필요하다

`intersection_type`은 지정된 링크의 전체 구간에서 발행된다.

영향 구간 예:

| Link | 길이 |
| --- | ---: |
| `A2256W000213` | 78.5m |
| `A2256W000315` | 67.8m |
| `A2256W000308` | 29.0m |

`traffic.py`가 이 값을 직접 사용하므로 의도한 동작인지 실차 시나리오로 확인해야 한다.

### 4. 동적 객체의 링크 횡거리 gate가 없다

객체는 자차에서 50m 이내여야 하지만, 객체에서 선택된 링크 중심선까지의 최대 거리는 제한하지 않는다. 교차로나 평행 도로 부근에서 잘못된 소속 링크가 나올 수 있다.

`max_link_lateral_distance_m` 추가를 권장한다.

### 5. 순환 route가 동적 후보 범위를 크게 만든다

route가 폐회로라서 어느 현재 링크에서 시작해도 동적 투영 후보가 거의 같은 54개 링크로 확장된다. 자차 거리 50m gate가 실제 범위를 줄이지만 그래프의 “전방 도달 가능” 의미는 약해진다.

거리 누적 또는 최대 forward link hop 제한이 필요하다.

### 6. reverse target은 원본 point 단위로 양자화된다

차량을 link segment에 투영하지 않고 가장 가까운 원본 point에서 시작한다. 지도 point 간격만큼 목표 거리 오차가 발생할 수 있다.

또한 `Point.z=yaw` 관례를 사용해 timestamp와 frame 정보가 없다. `PointStamped` 또는 별도 message가 더 명확하다.

### 7. source extract는 독립 Catkin 패키지가 아니다

`morai_link.py`는 `macaron_8.msg`를 import한다. 이 저장소만 clone해서는 메시지가 생성되지 않으며 MACARON 8 workspace 또는 별도 package scaffold가 필요하다.

원본 MACARON 8도 `catkin_make install` 시 helper module과 MGeo JSON을 모두 설치하지 않아 install-space 배포에는 추가 CMake/package 구성이 필요하다.

### 8. localization 메시지 타입 통일이 필요하다

현행 link node는 `geometry_msgs/PointStamped`를 요구한다. MACARON 8의 `datahub_imu.py`는 이 타입을 발행하지만 구형 `data_hub.py`는 같은 `/current_pose`에 `geometry_msgs/Point`를 사용한다.

두 localization node를 동시에 사용하거나 구형 node만 실행하면 ROS 연결 타입이 맞지 않는다.

### 9. 자동 테스트가 없다

현재 계산 모듈은 ROS I/O와 어느 정도 분리돼 있으므로 다음 테스트를 추가하기 쉽다.

- route 연결성 및 모든 ID 존재 검사
- 최근접 link의 거리/heading 경계값
- 짧은 링크를 넘는 40m 후보 생성
- 분기점에서 route 우선순위
- reverse target의 링크 경계 통과
- 동적 객체의 motion/heading/distance filter
- virtual boundary 접속과 offset 방향

## 현재 실행 환경 주의

MACARON 8 workspace에서 source 파일 이동 전 생성된 Catkin devel relay가 남아 있으면 존재하지 않는 `src/path_planning/morai_link.py`를 가리킬 수 있다. 이 경우 workspace 루트에서 다시 빌드해야 한다.

```bash
catkin_make
source devel/setup.bash
```
