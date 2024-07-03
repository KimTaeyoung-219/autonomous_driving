import Function_Library as fl
import time

# window: "COM5", mac: "/dev/cu.xxxxx"
arduino_port = "COM8"
lidar_port = "/dev/cu.usbserial-0001"
img_num = 200

if __name__ == "__main__":
    # Exercise Environment Setting
    # camera
    env = fl.libCAMERA(wait_value=10, max_speed = 130)
    time_check = False
    # arduino
    ser = fl.libARDUINO()
    comm = ser.init(arduino_port, 9600)

    # 처음 직진
    t1 = time.time()
    env.send_signal_to_arduino(comm, 70, 14)
    while True:
        t2 = time.time()
        if t2 - t1 > 18:
            break

    # 정지
    env.send_signal_to_arduino(comm, 0, 14)
    while True:
        t3 = time.time()
        if t3 - t2 > 2:
            break

    # 좌측으로 전방 방향
    env.send_signal_to_arduino(comm, 70, 40)
    while True:
        t4 = time.time()
        if t4 - t3 > 7:
            break

    # 정지
    env.send_signal_to_arduino(comm, 0, 0)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    # 뒤로
    env.send_signal_to_arduino(comm, 140, 0)
    while True:
        t6 = time.time()
        if t6 - t5 > 12:
            break

    # 정지
    env.send_signal_to_arduino(comm, 0, 0)
    while True:
        t7 = time.time()
        if t7 - t6 > 4:
            break

    # 와리가리
    env.send_signal_to_arduino(comm, 70, 0)
    while True:
        t8 = time.time()
        if t8 - t7 > 1:
            break

    env.send_signal_to_arduino(comm, 0, 0)
    while True:
        t9 = time.time()
        if t9 - t8 > 0.5:
            break

    env.send_signal_to_arduino(comm, 70, 40)

    while True:
        t10 = time.time()
        if t10 - t9 > 1:
            break

    env.send_signal_to_arduino(comm, 0, 0)

    while True:
        t11 = time.time()
        if t11 - t10 > 1:
            break

    env.send_signal_to_arduino(comm, 70, 40)

    while True:
        t12 = time.time()
        if t12 - t11 > 16:
            break

    env.send_signal_to_arduino(comm, 0, 0)

    while True:
        t13 = time.time()
        if t13 - t12 > 2:
            break

    env.send_signal_to_arduino(comm, 70, 14)

    while True:
        t14 = time.time()
        if t14 - t13 > 15:
            break

    env.send_signal_to_arduino(comm, 0, 0)

    while True:
        t15 = time.time()
        if t15 - t14 > 1:
            break