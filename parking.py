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
    env.send_signal_to_arduino(comm, 0, 144)

    input("Start!!")

    # go straight
    t1 = time.time()
    print_stage("STAGE1", True)
    env.send_signal_to_arduino(comm, 70, 144)
    lidar.fetch_scanning()
    while True:
        if lidar.check_scanning():
            lidar_data = lidar.read_scanning()
            print("LiDAR data received")
            flag = lidar.getAngleDistanceRange(lidar_data, 260, 280, 100, 800)
            flag2 = lidar.getAngleDistanceRange(lidar_data, 260, 280, 1000, 2000)
            if flag:
                # lidar.stop()
                print("found!!")
                T = 3
                break
            if flag2:
                # lidar.stop()
                print("found2!!")
                T = 10
                break
    # stop
    print_stage("STAGE2", True)
    t2 = time.time()
    env.send_signal_to_arduino(comm, 0, 140)
    while True:
        t3 = time.time()
        if t3 - t2 > 2:
            break

    # move foward left
    print_stage("STAGE3", True)
    t3 = time.time()
    env.send_signal_to_arduino(comm, 70, 168)
    while True:
        t4 = time.time()
        if t4 - t3 > 12:
            break

    # stop
    print_stage("STAGE4", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 0, 144)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    t4 = time.time()
    env.send_signal_to_arduino(comm, 1000, 144)
    while True:
        t5 = time.time()
        if t5 - t4 > T:
            break

    env.send_signal_to_arduino(comm, 0, 144)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    # move backward, parking, stop
    print_stage("STAGE5", True)
    while True:
        _, image = env.camera_read(ch0)
        crosswalk_image = env.convert_crosswalk_image(image)
        result, ans, edges = go_backward(env, image)
        speed, angle = env.get_speed_angle(ans)
        speed = 1000
        angle = 280 - angle
        if angle > 168:
            angle = 168
        if angle < 112:
            angle = 112
            # env.send_signal_to_arduino(comm, speed, angle)
        env.image_show(result, edges)
        # if env.find_crosswalk(crosswalk_image):
        #     env.send_signal_to_arduino(comm, 0, 14)
        #     time.sleep(2)
        #     break
        key = env.wait_key()

    # stop
    print_stage("STAGE6", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 0, 0)
    while True:
        t5 = time.time()
        if t5 - t4 > 2:
            break

    # move forward
    print_stage("STAGE7", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 70, 0)
    while True:
        t5 = time.time()
        if t5 - t4 > 10:
            break

    # move left
    print_stage("STAGE8", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 70, 28)
    while True:
        t5 = time.time()
        if t5 - t4 > 10:
            break

    # move forward
    print_stage("STAGE9", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 70, 14)
    while True:
        t5 = time.time()
        if t5 - t4 > 30:
            break

    # stop
    env.send_signal_to_arduino(comm, 0, 14)
