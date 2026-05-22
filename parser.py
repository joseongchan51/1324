import struct
import time
from config import MAGIC_WORD, HEADER_LEN, MAX_PACKET_LEN


def read_packet_buffer(data, buffer):
    if data.in_waiting > 0:        #읽을 데이터 확인
        buffer.extend(data.read(data.in_waiting)) # append 쓸시 큰 상자 그대로 넣기 extned시 알맹이만 빼서 넣기

    if len(buffer) < HEADER_LEN:
        time.sleep(0.005)
        return None

    idx = buffer.find(MAGIC_WORD) #매직워드

    if idx == -1:       #magic word 못 찾음. 아직 packet 시작 못 찾음
        if len(buffer) > 5000:
            del buffer[:-1000]  #오래된 데이터 삭제
        return None

    del buffer[:idx]    #매직워드앞에 쓰레기데이터 삭제

    if len(buffer) < HEADER_LEN:
        return None

    header = buffer[:HEADER_LEN]
    total_packet_len = struct.unpack_from("<I", header, 12)[0] #전체 packet 길이를 해석

    if total_packet_len < HEADER_LEN or total_packet_len > MAX_PACKET_LEN:
        del buffer[:8]
        return None #앞 8바이트 버리고 다시 탐색 왜 8바이트? 보통 MAGIC_WORD 길이가 8바이트.

    if len(buffer) < total_packet_len:  #헤더에서 읽은 총 길이
        time.sleep(0.005)
        return None 

    packet = bytes(buffer[:total_packet_len])
    del buffer[:total_packet_len]

    return packet


def parse_tlv_points(packet):
    num_detected_obj = struct.unpack_from("<I", packet, 28)[0]
    num_tlvs = struct.unpack_from("<I", packet, 32)[0]

    offset = HEADER_LEN
    points = []

    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            break

        tlv_type = struct.unpack_from("<I", packet, offset)[0]
        tlv_len = struct.unpack_from("<I", packet, offset + 4)[0]

        payload_start = offset + 8
        payload_end = offset + tlv_len

        if tlv_len < 8 or payload_end > len(packet):
            break

        payload = packet[payload_start:payload_end]

        if tlv_type == 1:
            point_size = 16
            n_points = len(payload) // point_size

            for i in range(n_points):
                base = i * point_size
                x, y, z, v = struct.unpack_from("<ffff", payload, base) #바이트를 플롯형태로 변환하는 struct.unpack-from 
                points.append([x, y, z, v])

        offset += tlv_len

    return points, num_detected_obj