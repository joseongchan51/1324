from config import (  KF_SIGMA_A, KF_MEASUREMENT_STD,KF_INITIAL_POSITION_STD,KF_INITIAL_VELOCITY_STD,)
import numpy as np


class KalmanFilter:

    # dt : 프레임 간 시간
    # x, y : 객체 초기 위치
    def __init__(
        self,
        dt,
        x,
        y,
    ):

        # 상태벡터(State Vector)
        # [x위치, y위치, x속도, y속도]
        self.X = np.array([
            [x],
            [y],
            [0.0],
            [0.0]
        ], dtype=float)

        # 상태 전이 행렬(F)
        # 현재 상태를 기반으로 다음 상태 예측
        self.F = np.eye(4, dtype=float)

        # 측정 행렬(H)
        # 레이더는 x,y 위치만 측정 가능
        # 속도는 직접 측정 못함
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # 초기 오차 공분산(P)
        # 현재 상태를 얼마나 못 믿는지
        self.P = np.diag([
    KF_INITIAL_POSITION_STD ** 2,
    KF_INITIAL_POSITION_STD ** 2,
    KF_INITIAL_VELOCITY_STD ** 2,
    KF_INITIAL_VELOCITY_STD ** 2,
    ])

        # 시스템 노이즈 크기 저장
        self.sigma_a = KF_SIGMA_A

        # 측정 노이즈 공분산(R)
        # 레이더 센서 오차
        r = KF_MEASUREMENT_STD ** 2

        self.R = np.array([
            [r, 0],
            [0, r],
        ], dtype=float)

        # 초기 Q 생성
        self.Q = np.eye(4, dtype=float)

        # dt 기반 F,Q 초기화
        self._update_model(dt)

    # dt가 바뀔 때마다
    # F(상태전이행렬), Q(시스템노이즈) 갱신
    def _update_model(self, dt):

        dt = float(max(dt, 1e-3))

        # Constant Velocity Model
        # x = x + vx*dt
        # y = y + vy*dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)

        # Q 계산용 dt 항들
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        # 가속도 노이즈 분산
        q = self.sigma_a ** 2

        # Constant Velocity용 Process Noise(Q)
        # 갑작스런 가속/감속 반영
        self.Q = q * np.array([
            [dt4 / 4, 0,         dt3 / 2, 0],
            [0,         dt4 / 4, 0,         dt3 / 2],
            [dt3 / 2, 0,         dt2,     0],
            [0,         dt3 / 2, 0,         dt2],
        ], dtype=float)

    # Predict 단계
    # 다음 상태 예측
    def predict(self, dt):

        # dt에 따라 모델 갱신
        self._update_model(dt)

        # 상태 예측
        # X = F * X
        self.X = self.F @ self.X

        # 오차 공분산 예측
        # P = FPF^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.state_dict()

    # Update 단계
    # 센서값으로 보정
    def update(self, meas_x, meas_y):

        # 측정 벡터(Z)
        Z = np.array([
            [meas_x],
            [meas_y]
        ], dtype=float)

        # Innovation(잔차)
        # 실제 측정값 - 예측 측정값
        Y = Z - self.H @ self.X

        # Innovation Covariance
        S = self.H @ self.P @ self.H.T + self.R

        # 칼만 이득(K)
        # 센서와 예측 중 누구를 더 믿을지 결정
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 상태 업데이트
        # X = X + K * Y
        self.X = self.X + K @ Y

        # Joseph Form
        # 수치 안정성이 더 좋음
        I = np.eye(4)

        KH = K @ self.H

        self.P = (
            (I - KH) @ self.P @ (I - KH).T
            + K @ self.R @ K.T
        )

        return self.state_dict()

    # 현재 상태를 dict 형태로 반환
    def state_dict(self):

        x = float(self.X[0, 0])
        y = float(self.X[1, 0])

        vx = float(self.X[2, 0])
        vy = float(self.X[3, 0])

        return {
            "x": x,
            "y": y,

            "vx": vx,
            "vy": vy,

            # 전체 속도 크기
            "speed": float(np.hypot(vx, vy)),

            # 원점 기준 거리
            "distance": float(np.hypot(x, y)),
        }