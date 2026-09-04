int32 track_id
int32 class_id
string class_name
float32 confidence
uint8 camera_mask

# 위치/형상이 어떤 측정 경로에서 나왔는지. FAR_SPARSE는 좌표용 희소점이며
# motion_state/heading은 충분한 정상 클러스터로 전환되기 전까지 신뢰하지 않는다.
uint8 SOURCE_YOLO_ONLY=0
uint8 SOURCE_LIDAR_CLUSTER=1
uint8 SOURCE_FAR_SPARSE=2
uint8 SOURCE_REACQUIRED=3
uint8 SOURCE_PREDICTED=4
uint8 measurement_source

geometry_msgs/Point position_lidar
geometry_msgs/Point position_utm
float32 heading_utm

# heading_utm 이 무엇에서 나왔는지. 박스 축 추정기(PCA / L-shape)는 물체의
# "축"만 줄 뿐 앞뒤 방향을 결정하지 못하므로, BOX_AXIS 로 나온 heading_utm 은
# pi 만큼 뒤집혀 있을 수 있다. 방향까지 신뢰할 수 있는 것은 속도 벡터에서
# 나온 VELOCITY 뿐이다.
uint8 HEADING_NONE=0      # 헤딩을 관측하지 못함 (heading_utm 무의미)
uint8 HEADING_BOX_AXIS=1  # 박스 축 기반. 축만 유효하고 방향은 미확정
uint8 HEADING_VELOCITY=2  # 이번 프레임 속도 벡터 기반. 방향까지 유효
uint8 HEADING_FROZEN=3    # 과거 속도로 확정한 헤딩 유지 중 (정차/재획득/예측)
uint8 heading_source

# 진행 방향까지 신뢰할 수 있을 때만 true (VELOCITY 또는 FROZEN).
# 하류에서 heading_utm 을 진행 방향으로 쓰려면 이 값을 먼저 확인해야 한다.
bool heading_valid

geometry_msgs/Vector3 dimensions
geometry_msgs/Vector3 relative_velocity
geometry_msgs/Vector3 world_velocity
uint8 motion_state
uint32 point_count
