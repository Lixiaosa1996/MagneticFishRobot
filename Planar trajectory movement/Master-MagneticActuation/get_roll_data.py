import numpy as np
from pyquaternion import Quaternion
from transforms3d import quaternions, euler
import struct
import math


def hex_to_bytes(hex_file_path):
    with open(hex_file_path, 'r') as hex_file:
        hex_string = hex_file.read().replace('\n', '')  # Read the file and remove newline characters
        bytes_data = bytes.fromhex(hex_string)  # Convert the hex string to bytes
    return bytes_data


# 使用函数
bytes_data = hex_to_bytes('roll_data\T4-0Hz.TXT')
# print(bytes_data)

# 读取四元数，并转换为欧拉角
euler_datas = []
flagOfByte = 0
for i in range(len(bytes_data)):
    byte = bytes_data[i]
    if flagOfByte == 0 and byte == 0xAA:
        flagOfByte = 1
    elif flagOfByte == 1 and byte == 0xFF:
        flagOfByte = 2
    elif flagOfByte == 2 and byte == 0x04:
        flagOfByte = 3
    elif flagOfByte == 3 and byte == 0x09:
        flagOfByte = 0
        receiveBuffer = bytes_data[i + 1:i + 12]

        i = i + 12

        q0 = struct.unpack('h' * 1, receiveBuffer[0:2])
        q1 = struct.unpack('h' * 1, receiveBuffer[2:4])
        q2 = struct.unpack('h' * 1, receiveBuffer[4:6])
        q3 = struct.unpack('h' * 1, receiveBuffer[6:8])

        # q0 = hex(receiveBuffer[1] * 256 + receiveBuffer[0])
        # q1 = hex(receiveBuffer[3] * 256 + receiveBuffer[2])
        # q2 = hex(receiveBuffer[5] * 256 + receiveBuffer[4])
        # q3 = hex(receiveBuffer[7] * 256 + receiveBuffer[6])

        qf0 = float(q0[0]) / 10000
        qf1 = float(q1[0]) / 10000
        qf2 = float(q2[0]) / 10000
        qf3 = float(q3[0]) / 10000

        quaternion = np.array([qf0, qf1, qf2, qf3])  # 示例四元数
        euler_angle = euler.quat2euler(quaternion, 'sxyz')
        roll = math.degrees(euler_angle[0])
        pitch = math.degrees(euler_angle[1])
        yaw = math.degrees(euler_angle[2])

        euler_datas.append([yaw, pitch, roll])

# 将欧拉角数据存为txt
with open('roll_data\euler_T4-0Hz.TXT', 'w') as f:
    # 遍历数据
    for row in euler_datas:
        # 把每行数据转换为字符串，并用逗号连接
        line = '  '.join(format(x, '.6f') for x in row)
        # 写入文件，并添加换行符
        f.write(line + '\n')