from collections import deque #deque: 자동으로 오래된 값을 버리는 리스트
import numpy as np

class MovingAvergeFilter:
    def __init__(self, window_size = 5): # 최근5개의값
        self.x_buffer = deque(maxlen = window_size) # deque(덱)은 "Double-Ended Queue"의 약자로, 양쪽 끝에서 자유롭게 데이터를 넣고 뺄 수 있는 자료구조
        self.y_buffer = deque(maxlen = window_size) # x, y, v, d 다 최근5개의값 사용
        self.v_buffer = deque(maxlen = window_size)
        self.distance_buffer = deque(maxlen = window_size)
    
    def update(self,obj): #새로운 데이터()가 딕셔너리 형태로 들어올 때마다 이 함수를 실행
        if obj is None:
            return None
        
        self.x_buffer.append(obj["x"])      #최근값 들어오면 오래된거 내보냄 x,y,v,d 다 할당
        self.y_buffer.append(obj["y"])
        self.v_buffer.append(obj["v"])
        self.distance_buffer.append(obj["distance"])

        return {
            "id": obj["id"],
            "x": np.mean(self.x_buffer),
            "y": np.mean(self.y_buffer),
            "z": obj["z"],
            "v": np.mean(self.v_buffer),
            "distance": np.mean(self.distance_buffer),
        }