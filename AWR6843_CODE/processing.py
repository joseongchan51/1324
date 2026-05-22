import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
from config import ( DBSCAN_EPS,
    DBSCAN_MIN_SAMPLES,
    MIN_RANGE,
    MAX_RANGE,
    VELOCITY_THRESHOLD,
    Y_DISTANCE_THRESHOLD,
    X_RANGE,

    TRACK_ASSOCIATION_MAX_DISTANCE,
    TRACK_MAX_MISSES,
    TRACK_MIN_HITS_TO_CONFIRM,)
from kalman_filter import KalmanFilter


# DBSCAN을 사용하여 점들을 클러스터링하고, 유효한 점들만 필터링하는 함수입니다.
def dbscan_scattering(points):
    points = np.array(points, dtype=float)

    distance = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
    
    valid = (distance >= MIN_RANGE) & (distance <= MAX_RANGE)  # 거리 설정 #부울함수를 사용하면 true fail로 저장됨
    points = points[valid] # 여기서 거리에 벗어나는 점들은 여기서 다 걸러진다. 필터링된 포인트가 시작되는 시작점 

    if len(points) == 0:
        return None, None, None, None, None

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    v = points[:, 3]

    distance = np.sqrt(x**2 + y**2 + z**2)
    #코드를 보면 알겠지만 DBSCAN을 하면 xy좌표에 따라 라벨링만 한다는 것을 알 수 있다. 
   ## 벽이나 물체의 너비를 보려면 eps, min_samples를 모두 작게해야 보이려나?..
   ## 목표1: 벽이 길게 point가 짝히는 eps, minsample 값을 오늘 찾아보자!
   
    xy = points[:, :2]
    labels = DBSCAN(
        eps=DBSCAN_EPS,
        min_samples=DBSCAN_MIN_SAMPLES
    ).fit_predict(xy)

    ## 노이즈를 제거하고 클러스터링된 포인트만 남긴다.
    filtered_points = points[labels != -1]
    filtered_labels = labels[labels != -1]
    filtered_distance = np.sqrt(filtered_points[:, 0]**2 + filtered_points[:, 1]**2 + filtered_points[:, 2]**2)
    ## df는 시각화와 분석을 위해 포인트 데이터를 구조화한 형태입니다.
    df = pd.DataFrame({
        "X_m": filtered_points[:, 0],
        "Y_m": filtered_points[:, 1],
        "Z_m": filtered_points[:, 2],
        "Distance_m": filtered_distance,
        "Velocity_mps": filtered_points[:, 3],
        "ClusterID": filtered_labels,
    })

    return df, filtered_labels, filtered_points[:,0], filtered_points[:,1], filtered_points

# extract_clusters에 들어가는 points는 dbscan_scattering에서 노이즈 제거, 제한된 거리로 필터링된 포인트들이다.
# 따라서 df도 노이즈제거, 제한된 거리로 필터링된 라이브러리다. 
def extract_clusters(points, labels): #extract : 추출하다 
    points = np.array(points, dtype=float)
    labels = np.array(labels)

    cluster_centroid_objects = [] #centroid 된 좌표 

    for cluster_id in set(labels):
        
        if cluster_id == -1:
            continue

        cluster_points = points[labels == cluster_id]   # 부울함수로서 true fail 로 출력된다는걸 명심 (마스킹) 


        centroid_x = np.mean(cluster_points[:, 0])
        centroid_y = np.mean(cluster_points[:, 1])
        centroid_z = np.mean(cluster_points[:, 2])
        centroid_v = np.mean(cluster_points[:, 3])

        centroid_distance = np.sqrt(
            centroid_x**2 + centroid_y**2 + centroid_z**2
        )
        #centroid 된 좌표 
        cluster_centroid_objects.append({
            "track_id": cluster_id,
            "x": centroid_x,
            "y": centroid_y,
            "z": centroid_z,
            "v": centroid_v,
            "distance": centroid_distance,
        })

    return cluster_centroid_objects

def assign_track_ids(cluster_objects, prev_tracks, next_track_id, dt):

    # 최종 tracking 객체 저장 리스트
    tracked_objects = []

    # 이전 프레임 track들의 예측 결과 저장
    predicted_tracks = []

    # 이전 프레임 track들을 현재 시점으로 예측
    for track in prev_tracks:

        # 칼만필터 predict 실행
        state = track["kf"].predict(dt)

        predicted_tracks.append({
            "track_id": track["track_id"],
            "kf": track["kf"],

            # 예측 위치
            "pred_x": state["x"],
            "pred_y": state["y"],

            # track 유지 정보
            "miss_count": track.get("miss_count", 0),
            "hit_count": track.get("hit_count", 1),
        })

    # 이미 사용된 track 저장
    used_track_indices = set()

    # 현재 프레임 객체 반복
    for obj in cluster_objects:

        best_idx = -1
        best_dist = float("inf")

        # 가장 가까운 예측 track 찾기
        for i, track in enumerate(predicted_tracks):

            # 이미 사용된 track은 건너뜀
            if i in used_track_indices:
                continue

            # 현재 객체와 예측 위치 거리 계산
            dist = np.hypot(
                obj["x"] - track["pred_x"],
                obj["y"] - track["pred_y"]
            )

            # 가장 가까운 track 저장
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        # 원본 객체 복사
        tracked_obj = dict(obj)

        # 기존 track과 매칭 성공
        if (
            best_idx != -1
            and best_dist <= TRACK_ASSOCIATION_MAX_DISTANCE
        ):

            track = predicted_tracks[best_idx]

            # 해당 track 사용 처리
            used_track_indices.add(best_idx)

            # 칼만필터 update
            state = track["kf"].update(
                obj["x"],
                obj["y"]
            )

            tracked_obj["track_id"] = track["track_id"]

            # 칼만필터가 보정한 위치 사용
            tracked_obj["x"] = state["x"]
            tracked_obj["y"] = state["y"]

            # 추정 속도
            tracked_obj["vx"] = state["vx"]
            tracked_obj["vy"] = state["vy"]

            tracked_obj["speed"] = state["speed"]
            tracked_obj["distance"] = state["distance"]

            # 정상 검출되었으므로 miss 초기화
            tracked_obj["miss_count"] = 0

            # 검출 횟수 증가
            tracked_obj["hit_count"] = (
                track["hit_count"] + 1
            )

            # 다음 프레임에서도 같은 KF 사용
            tracked_obj["kf"] = track["kf"]

            # 현재 정상 검출 상태
            tracked_obj["lost"] = False

        # 기존 track과 매칭 실패
        else:

            # 새 칼만필터 생성
            kf = KalmanFilter(
                dt,
                obj["x"],
                obj["y"]
            )

            state = kf.state_dict()

            tracked_obj["track_id"] = next_track_id

            tracked_obj["x"] = state["x"]
            tracked_obj["y"] = state["y"]

            tracked_obj["vx"] = state["vx"]
            tracked_obj["vy"] = state["vy"]

            tracked_obj["speed"] = state["speed"]
            tracked_obj["distance"] = state["distance"]

            tracked_obj["miss_count"] = 0
            tracked_obj["hit_count"] = 1

            tracked_obj["kf"] = kf

            tracked_obj["lost"] = False

            # 다음 ID 증가
            next_track_id += 1

        tracked_objects.append(tracked_obj)

    # 이번 프레임에서 검출 안 된 track 처리
    for i, track in enumerate(predicted_tracks):

        # 이미 매칭된 track은 제외
        if i in used_track_indices:
            continue

        # miss 증가
        miss_count = track["miss_count"] + 1

        # 너무 오래 안 보이면 삭제
        if miss_count > TRACK_MAX_MISSES:
            continue

        # 마지막 예측 상태 유지
        state = track["kf"].state_dict()

        lost_obj = {
            "id": -1,

            "track_id": track["track_id"],

            "x": state["x"],
            "y": state["y"],

            "vx": state["vx"],
            "vy": state["vy"],

            "v": 0.0,

            "speed": state["speed"],
            "distance": state["distance"],

            "miss_count": miss_count,
            "hit_count": track["hit_count"],

            "kf": track["kf"],

            # 현재 프레임에서 검출 실패 상태
            "lost": True,
        }

        tracked_objects.append(lost_obj)

    # 다음 프레임으로 넘길 track 정보 저장
    next_prev_tracks = []

    for obj in tracked_objects:

        next_prev_tracks.append({
            "track_id": obj["track_id"],
            "kf": obj["kf"],

            "miss_count": obj.get(
                "miss_count",
                0
            ),

            "hit_count": obj.get(
                "hit_count",
                1
            ),
        })

    # 시각화 및 제어용 객체 선택
    visible_tracks = [

        obj for obj in tracked_objects

        # 현재 검출된 객체만 사용
        if not obj.get("lost", False)

        # 충분히 안정적인 track만 사용
        and obj.get(
            "hit_count",
            0
        ) >= TRACK_MIN_HITS_TO_CONFIRM
    ]

    # 가장 가까운 객체 선택
    if len(visible_tracks) == 0:

        nearest_obj = None

    else:

        nearest_obj = min(
            visible_tracks,
            key=lambda obj: obj["distance"]
        )

    return (
        tracked_objects,
        nearest_obj,
        next_prev_tracks,
        next_track_id
    )

def velocity_filter(obj):

    if not obj:
        return np.empty((0, 6), dtype = float)  # 빈 배열 반환 (5는 객체의 속성 수)
    
    if isinstance(obj[0], dict): # 이 변수의 자료형이 맞는지 검사하는 코드
        obj = [item for item in obj if not item.get("lost", False)]
        if not obj:
            return np.empty((0, 6), dtype=float)

        obj = np.array([
            [
                float(item["track_id"]),
                float(item["x"]),
                float(item["y"]),
                float(item.get("z", 0.0)),
                float(item.get("v", 0.0)),
                float(item["distance"])
            ]
            for item in obj
        ], dtype=float)
    else:
        obj = np.array(obj, dtype=float)
    
    velocity = obj[:, 4] 
    Y_distance = obj[:, 2]
    X_distance = obj[:, 1]
    
    valid =(
    (np.abs(velocity) > VELOCITY_THRESHOLD) 
    &(Y_distance > Y_DISTANCE_THRESHOLD)  
    &(Y_distance < MAX_RANGE) 
    &(X_distance < X_RANGE) 
    &(X_distance > -X_RANGE))  

    velocity_obj = obj[valid]  

    return velocity_obj
