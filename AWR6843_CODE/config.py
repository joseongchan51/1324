# =========================
# 전역 상수 (설정값)
# =========================
MAGIC_WORD = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER_LEN = 40
MAX_PACKET_LEN = 65535

# [DBSCAN] 클러스터링 파라미터
DBSCAN_EPS = 0.3
DBSCAN_MIN_SAMPLES = 3
MIN_RANGE = 0.5
MAX_RANGE = 10.0

# EKF 예측 단계용 프레임 시간 간격 
DT_DEFAULT = 0.05
DT_MIN = 0.001
DT_MAX = 0.2

#프레임간 최근접 연관 거리 게이트(m)
TRACK_ASSOCIATION_MAX_DISTANCE = 0.8

# 속도 필터링 임계값 (m/s)
VELOCITY_THRESHOLD = 0.2
Y_DISTANCE_THRESHOLD = 0.2
X_RANGE = 0.5

# [그래프 고정] 화면 좌표 범위
VIEW_X_MIN, VIEW_X_MAX = -5.0, 5.0
VIEW_Y_MIN, VIEW_Y_MAX = 0.0, 10.0
# [Lane visualization] 50 cm lanes, 3 lanes, ego vehicle centered in lane 2.
LANE_WIDTH_M = 0.5
LANE_COUNT = 3
EGO_LANE_INDEX = 2


#================================================#
# [Tracking] Track 관리 파라미터

# 예측 위치와 현재 측정 위치가
# 이 거리 안이면 같은 객체로 판단
TRACK_ASSOCIATION_MAX_DISTANCE = 0.35

# 몇 프레임 동안 안 보여도
# track 유지할지
TRACK_MAX_MISSES = 5

# 몇 번 이상 연속 검출되어야
# 안정적인 객체로 인정할지
TRACK_MIN_HITS_TO_CONFIRM = 2


# [Kalman Filter] 튜닝 파라미터

# 시스템 노이즈(Q)
# 값이 커질수록 모델을 덜 믿고
# 센서를 더 따라감
KF_SIGMA_A = 1.5

# 측정 노이즈(R)
# 값이 커질수록 센서를 덜 믿음
KF_MEASUREMENT_STD = 0.25

# 초기 위치 불확실성
KF_INITIAL_POSITION_STD = 0.5

# 초기 속도 불확실성
KF_INITIAL_VELOCITY_STD = 2.0


def build_config():
    """실행 설정을 한 곳에서 관리."""
    return {
        "cli_port": "COM7",
        "data_port": "COM6",
        "baud_cli": 115200,
        "baud_data": 921600,
        "cfg_file": r"C:\ti\mmwave_sdk_03_06_02_00-LTS\packages\ti\demo\xwr68xx\mmw\profiles\profile_2d.cfg",
        "read_timeout": 0.01,
        "cli_timeout": 0.5,
    }