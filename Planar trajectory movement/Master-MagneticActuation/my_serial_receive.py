import time

import serial
import struct
import math
import numpy as np
from transforms3d import euler

import threading

# 指示接收串口的开始与停止接收
receive_flag = False


# 用于接收数据的线程函数
def receive_data(port, baud_rate, filename):
    receive_serial = serial.Serial(port, baud_rate)

    with open(filename, 'wb') as f:

        print('receive data begin...')
        flag_of_byte = 0

        # 循环从串口读取四元数，并转化为欧拉角保存
        while receive_flag:
            # 从串口获取1byte数据
            byte_data = receive_serial.read(1)

            # 解码
            if flag_of_byte == 0 and byte_data == b'\xaa':
                flag_of_byte = 1
            elif flag_of_byte == 1 and byte_data == b'\xff':
                flag_of_byte = 2
            elif flag_of_byte == 2 and byte_data == b'\x04':
                flag_of_byte = 3
            elif flag_of_byte == 3 and byte_data == b'\x09':
                flag_of_byte = 0
                receive_buffer = receive_serial.read(11)

                q0 = struct.unpack('h' * 1, receive_buffer[0:2])
                q1 = struct.unpack('h' * 1, receive_buffer[2:4])
                q2 = struct.unpack('h' * 1, receive_buffer[4:6])
                q3 = struct.unpack('h' * 1, receive_buffer[6:8])

                qf0 = float(q0[0]) / 10000
                qf1 = float(q1[0]) / 10000
                qf2 = float(q2[0]) / 10000
                qf3 = float(q3[0]) / 10000

                quaternion = np.array([qf0, qf1, qf2, qf3])  # 示例四元数
                euler_angle = euler.quat2euler(quaternion, 'sxyz')
                roll = math.degrees(euler_angle[0])
                pitch = math.degrees(euler_angle[1])
                yaw = math.degrees(euler_angle[2])

                # 将姿态角逐行写入文件
                euler_data = [yaw, pitch, roll]
                #print(euler_data)
                line = '  '.join(map(str, euler_data)).encode('utf-8')
                f.write(line + b'\n')

            else:
                flag_of_byte = 0

    print('receive data finish.')

    receive_serial.close()


if __name__ == '__main__':

    thread_receive = threading.Thread(target=receive_data, args=('COM9', 115200, 'roll_data_sync\euler_T1-30Hz.TXT'))

    receive_flag = True
    thread_receive.start()
    time.sleep(5)
    receive_flag = False

    #等待子线程结束
    thread_receive.join()

