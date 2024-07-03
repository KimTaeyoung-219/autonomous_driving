import Function_Library as fl
import time

# window: "COM5", mac: "/dev/cu.xxxxx"
arduino_port = "/dev/cu.usbmodem1201"
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
    lidar.fetch_scanning()

    # go straight
    t1 = time.time()
    print_stage("STAGE1", True)
    env.send_signal_to_arduino(comm, 70, 14)
    while True:
        t2 = time.time()
        if lidar.check_scanning():
            lidar_data = lidar.read_scanning()
            flag = lidar.getAngleDistanceRange(lidar_data, 170, 190, 1600, 2000)
            if flag:
                lidar.stop()
                break

    # stop
    print_stage("STAGE2", True)
    t2 = time.time()
    env.send_signal_to_arduino(comm, 0, 14)
    while True:
        t3 = time.time()
        if t3 - t2 > 2:
            break

    # move foward left
    print_stage("STAGE3", True)
    t3 = time.time()
    env.send_signal_to_arduino(comm, 70, 40)
    while True:
        t4 = time.time()
        if t4 - t3 > 7:
            break

    # stop
    print_stage("STAGE4", True)
    t4 = time.time()
    env.send_signal_to_arduino(comm, 0, 0)
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
        angle = 28 - angle
        if angle > 28:
            angle = 28
        if angle < 0:
            angle = 0
            env.send_signal_to_arduino(comm, speed, angle)
        env.image_show(result, ans, edges)
        if env.find_crosswalk(crosswalk_image):
            env.send_signal_to_arduino(comm, 0, 14)
            time.sleep(2)
            break
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
