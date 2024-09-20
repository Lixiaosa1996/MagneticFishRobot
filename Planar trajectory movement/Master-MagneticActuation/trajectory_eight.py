import serial
import time
import _thread
import struct


class MSerialPort:
    message = ''

    def __init__(self, port, buand, timex=5):
        self.port = serial.Serial(port, buand, timeout=timex)
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self, data):
        number = self.port.write(data)
        return number

    def read_data(self):
        while True:
            data = self.port.readline()
            try:
                print(data.decode('ASCII'), end='')
            except:
                print(data)

def print_bytes_hex(data):
    lin = ['%02X' % i for i in data]
    print(" ".join(lin))


if __name__ == '__main__':
    mSerial = MSerialPort('COM3', 115200)
    _thread.start_new_thread(mSerial.read_data, ())

    time.sleep(1)

    # 磁力参数设定
    attract_force = -150
    repell_force = 70
    vibrate_force = 200

    time_guide = 0.5
    time_stable = 1.5
    time_release = 0.1

    vibration_freq = 30
    guide_steps = 20
    release_steps = 20

    # 轨迹设定：引导线圈+稳定线圈+振动线圈
    coil_path = [[[4, 3], [5, 2], [3, 2]],  # 9
                 [[1, 7], [4, 3], [3, 2]],  # 16
                 [[4, 4], [1, 7], [3, 5]],  # 23
                 [[5, 5], [4, 4], [3, 5]],  # 24
                 [[2, 5], [5, 5], [3, 5]],  # 18
                 [[4, 5], [2, 5], [3, 5]],  # 11
                 [[5, 4], [4, 5], [3, 5]],  # 10
                 [[1, 7], [5, 4], [3, 5]],  # 16
                 [[5, 3], [1, 7], [3, 2]],  # 22
                 [[4, 2], [5, 3], [3, 2]],  # 21
                 [[2, 2], [4, 2], [3, 2]],  # 14
                 [[5, 2], [2, 2], [3, 2]]   # 8
                 ]

    # 8字运动
    for i in range(1000):
        coil_no = i % 12
        c1_sel = coil_path[coil_no][0][0]
        c1_no = coil_path[coil_no][0][1]
        c2_sel = coil_path[coil_no][1][0]
        c2_no = coil_path[coil_no][1][1]
        c3_sel = coil_path[coil_no][2][0]
        c3_no = coil_path[coil_no][2][1]

        # guide
        for j in range(guide_steps):
            data = struct.pack('BBllBBllBBllHcc',
                               c1_sel, c1_no, int(1.5 * attract_force * (guide_steps - j /1.5) / guide_steps), vibrate_force,
                               c2_sel, c2_no, int(attract_force*0.1* (guide_steps - j * 2.0) / guide_steps), vibrate_force,
                               c3_sel, c3_no, 0, vibrate_force,
                               vibration_freq, b'\r', b'\n')
            mSerial.send_data(data)
            time.sleep(time_guide / guide_steps)

        # stable
        data = struct.pack('BBllBBllBBllHcc',
                               c1_sel, c1_no, attract_force, vibrate_force,
                               c2_sel, c2_no, repell_force, vibrate_force,
                               c3_sel, c3_no, 0, vibrate_force,
                               vibration_freq, b'\r', b'\n')
        mSerial.send_data(data)
        time.sleep(time_stable)

        # release
        for k in range(release_steps):
            data = struct.pack('BBllBBllBBllHcc',
                                c1_sel, c1_no, int(attract_force * (release_steps - j) / release_steps), vibrate_force,
                                c2_sel, c2_no, int(repell_force * (release_steps - j) / release_steps), vibrate_force,
                                c3_sel, c3_no, 0, vibrate_force,
                                vibration_freq, b'\r', b'\n')
            mSerial.send_data(data)
            time.sleep(time_release / release_steps)
        print(c1_sel, c1_no)

    # 关闭串口
    mSerial.port_close()
