import Function_Library as fl
import time

# window: "COM5", mac: "/dev/cu.xxxxx"
arduino_port = "/dev/cu.usbmodem1301"
lidar_port = "/dev/cu.usbserial-0001"
img_num = 200


def print_stage(stage, flag):
    if flag is False:
        return
    print(f"-------------------")
    print(f"------{stage}------")
    print(f"-------------------")


def go_backward(env, image):
    resize_image = env.resize_image(image)
    valid_image = env.extract_valid_image(resize_image)
    edges = env.convert_image_to_1d(valid_image)
    result, ans = env.edge_detection(edges, lines=[450, 400, 350, 300])
    return result, ans, edges


if __name__ == "__main__":
    # Exercise Environment Setting
    # camera
    env = fl.libCAMERA(wait_value=10, max_speed=70)
    time_check = False
    # arduino
    ser = fl.libARDUINO()
    ch0, ch1 = env.initial_setting(capnum=1)
    comm = ser.init(arduino_port, 9600)
    lidar = fl.libLIDAR(lidar_port)
    time.sleep(3)
    straight = 144
    backward = 140
    left = 200
    right = 100
    T3 = 11.3
    T4 = 11
    env.send_signal_to_arduino(comm, 0, straight)

    input("Parking Start!!")

    # go straight
    t1 = time.time()
    print_stage("STAGE1", True)
    env.send_signal_to_arduino(comm, 70, straight)

    t2 = time.time()
    while True:
        t3 = time.time()
        if t3 - t2 > 3.5:
            print("getting LiDAR data")
            break
    # lidar.fetch_scanning()
    # while True:
    #     if lidar.check_scanning():
    #         lidar_data = lidar.read_scanning()
    #         print("LiDAR data received")
    #         flag = lidar.getAngleDistanceRange(lidar_data, 260, 280, 100, 1000)
    #         flag2 = lidar.getAngleDistanceRange(lidar_data, 260, 280, 1300, 2000)
    #         if flag:
    #             # lidar.stop()
    #             print("found!!")
    #             T = 3
    #             break
    #         if flag2:
    #             # lidar.stop()
    #             print("found2!!")
    #             T = 10
    #             break
    for lidar_data in lidar.scanning():
        print("LiDAR data received")
        flag = lidar.getAngleDistanceRange(lidar_data, 265, 275, 100, 900)
        flag2 = lidar.getAngleDistanceRange(lidar_data, 267, 273, 1100, 2000)
        if flag:
            print("found!!")
            T2 = 2.5
            T = 14
            break
        if flag2:
            print("found2!!")
            T2 = 2.7
            T = 16
            break
    # stop
    print_stage("STAGE2", True)
    t2 = time.time()
    env.send_signal_to_arduino(comm, 0, straight)
    while True:
        t3 = time.time()
        if t3 - t2 > 2:
            break
    # move foward
    print_stage("STAGE2.5", True)
    t2 = time.time()
    env.send_signal_to_arduino(comm, 70, straight)
    while True:
        t3 = time.time()
        if t3 - t2 > T2:
            print(f"time: {t3 - t2}")
            break

    # move foward left
    print_stage("STAGE3", True)
    t3 = time.time()
    env.send_signal_to_arduino(comm, 70, left)
    while True:
        t4 = time.time()
        if t4 - t3 > T3:
            break

    # stop
    print_stage("STAGE4", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 0, straight)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    # move backward
    t4 = time.time()
    env.send_signal_to_arduino(comm, 1000, backward)
    while True:
        t5 = time.time()
        if t5 - t4 > T:
            break

    env.send_signal_to_arduino(comm, 0, straight)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    # move backward, parking, stop
    print_stage("STAGE5", True)
    t2 = time.time()
    while True:
        _, image = env.camera_read(ch0)
        crosswalk_image = env.convert_crosswalk_image(image)
        result, ans, edges = go_backward(env, image)
        speed, angle = env.get_speed_angle(ans)
        # speed = 1000
        # angle = 280 - angle
        # if angle > 168:
        #     angle = 168
        # if angle < 112:
        #     angle = 112
            # env.send_signal_to_arduino(comm, speed, angle)
        env.image_show(result, edges)
        t3 = time.time()
        if t3 - t2 > 2:
            break
        # if env.find_crosswalk(crosswalk_image):
        #     # env.send_signal_to_arduino(comm, 0, 14)
        #     time.sleep(2)
        #     break
        key = env.wait_key()

    # stop
    print_stage("STAGE6", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 0, straight)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    # move forward
    print_stage("STAGE7", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 70, straight)
    while True:
        t5 = time.time()
        if t5 - t4 > 4:
            break

    # move right
    print_stage("STAGE8", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 70, right)
    while True:
        t5 = time.time()
        if t5 - t4 > T4:
            break

    # move forward
    print_stage("STAGE9", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 170, straight)
    while True:
        t5 = time.time()
        if t5 - t4 > 10:
            break

    # stop
    env.send_signal_to_arduino(comm, 0, straight)
