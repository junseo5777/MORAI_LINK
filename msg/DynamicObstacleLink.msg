int32 track_id

string nearest_link_idx
geometry_msgs/Point projected_position_utm

# 현재 자차 위치에서 장애물 중심까지의 UTM 평면 직선거리.
float64 ego_distance_m

# 장애물 중심에서 링크 중심선까지의 부호 있는 횡방향 거리.
# 링크 진행 방향을 기준으로 왼쪽은 양수, 오른쪽은 음수다.
float64 signed_link_distance_m

# 링크의 국소 진행 방향에서 장애물 진행 방향을 뺀 값. 범위는 [-pi, pi].
float64 signed_heading_error_rad
