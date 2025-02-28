import Function_Library as fl
import time

# window: "COM5", mac: "/dev/cu.xxxxx"
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
arduino_port = "/dev/cu.usbmodem1301"
lidar_port = "/dev/cu.usbserial-0001"
img_num = 0
save_num = 80000
save_image = True

def print_stage(stage, flag):
    if flag is False:
        return
    print(f"-------------------")
    print(f"------{stage}------")
    print(f"-------------------")
    return


def go_forward(env, image):
    resize_image = env.resize_image(image)
    valid_image = env.extract_valid_image(resize_image)
    edges = env.convert_image_to_1d(valid_image)
    result, ans = env.edge_detection(edges)
    return result, ans, edges


if __name__ == "__main__":
    # Exercise Environment Setting
    # camera
    env = fl.libCAMERA(wait_value=10, max_speed=240)
    time_check = False
    stage_check = True
    # change lane to ${change_lane}
    change_lane = "left"
    state = None
    pred = 16 # front: 18, 11 # left 35 right 5
    pred2 = 11 # front 12, 9
    stage8 = 25
    a = pred
    stage = 1
    lidar_num = 0
    # arduino
    ser = fl.libARDUINO()
    lidar = None
    comm = ser.init(arduino_port, 9600)
    lidar = fl.libLIDAR(lidar_port)

    # Camera Initial Setting
    ch0, ch1 = env.initial_setting(capnum=1)
    # Camera using Thread
    # env.fetch_image_camera(channel=ch0)
    # LiDAR using Thread
    env.send_signal_to_arduino(comm, 0, 140)
    input("Obstacle start!!")
    lidar.fetch_scanning()

    # Camera Reading..
    print_stage("STAGE 1", stage_check)
    while True:
        # reading source
        if time_check:
            t1 = time.time()
        # _, image = env.camera_read(ch0)
        # _, image2, _, image = env.camera_read(ch0, ch1)
        _, image = env.camera_read(ch0)
        # Camera using Thread
        # image = env.read_image_thread()
        # env.fetch_image_camera(channel=ch0)
        # env.image_show(image, image2)
        # env.wait_key()

        # LiDAR using Thread
        # env.fetch_image_camera(channel=ch0)
        if lidar.check_scanning() is True:
            lidar_data = lidar.read_scanning()

        # image = env.file_read("data/image"+str(img_num)+".png")

        # set target point with image
        if time_check:
            t2 = time.time()
        result, ans, edges = go_forward(env, image)
        crosswalk_image = env.convert_crosswalk_image(image)

        # custom the target point by obstacle detection, lidar data, crosswalk
        if time_check:
            t3 = time.time()
        if stage == 1:  # when starts, change lane to left immediately
            if lidar.check_scanning():
                lidar_data = lidar.read_scanning()
                print(f"First LiDAR data received: {lidar_num}")
                lidar_num += 1
                flag = lidar.getAngleDistanceRange(lidar_data, 165, 185, 100, 3300)
                if flag:
                    env.stage = "LEFT"
                    change_lane = "left"
                    print_stage("STAGE 8", stage_check)
                    stage = 8
        elif stage == 8:
            if stage8 <= 0:
                stage = 2
                print_stage("STAGE 2", stage_check)
            else:
                stage8 -= 1
        elif stage == 2:  # stay in line when the car move to the left side of obstacle1
            edges = env.find_car_lane(edges, ans)
            if a > 0 or env.cur_lane != change_lane:
                ans = env.change_car_lane(ans, change_lane)
                state = 171
                print(f"Go left: {a}")
                a -= 1
            else:
                env.stage = "NONE"
                state = None
                print_stage("STAGE 3", stage_check)
                a = pred2
                stage = 3
        elif stage == 3:  # when find obstacle2 in front, change lane to right
            if lidar.check_scanning():
                print(f"Second LiDAR data received: {lidar_num}")
                lidar_num += 1
                lidar_data = lidar.read_scanning()
                flag = lidar.getAngleDistanceRange(lidar_data, 165, 195, 300, 2000)
                if flag:
                    env.stage = "RIGHT"
                    change_lane = "right"
                    print_stage("STAGE 4", stage_check)
                    stage = 4
        elif stage == 4:  # change lane to right
            edges = env.find_car_lane(edges, ans)
            if a > 0 or env.cur_lane != change_lane:
                ans = env.change_car_lane(ans, change_lane)
                state = 115
                print(f"Go right: {a}")
                a -= 1
            else:
                env.stage = "NONE"
                state = None
                print_stage("STAGE 5", stage_check)
                a = 60
                stage = 5
        elif stage == 5:  # when find crosswalk_image, stop for 2 sec and move straight
            a -= 1
            if a <= 0:
                env.max_speed = 160
            if env.find_crosswalk(crosswalk_image) and a <= 0:
                env.send_signal_to_arduino(comm, 0, 140)
                stage = 6
                print_stage("STAGE 6", stage_check)
                gp = env.find_green_traffic_light(image)
                env.green_pixel = gp
                env.max_speed = 0
                # env.send_signal_to_arduino(comm, 60, 14)
                # # lidar.stop()
                # print(f"obstacle terminated")
                # time.sleep(2)
                # stage = 6
        elif stage == 6:
            gp = env.find_green_traffic_light(image)
            if (gp - env.green_pixel) > 4500:
                env.send_signal_to_arduino(comm, 60, 140)
                time.sleep(2)
                env.max_speed = 250
                stage = 7
            else:
                continue

        # clear lidar buffer when stage is not using lidar data
        if stage == 2 or stage == 4 or stage == 5 or stage == 6 or stage == 7 or stage == 8:
            if lidar.check_scanning() is True:
                print(f"Dump LiDAR data received: {lidar_num}")
                lidar_num += 1
                lidar_data = lidar.read_scanning()

        # get speed and angle of the car
        if time_check:
            t4 = time.time()
        speed, angle = env.get_speed_angle(ans)

        if state is not None and a > 0:
            angle = state

        # send data to arduino
        if time_check:
            t5 = time.time()
        env.send_signal_to_arduino(comm, speed, angle)

        # print image of final results
        if time_check:
            t6 = time.time()
        env.image_show(result, edges, crosswalk_image, image)
        # env.image_show(image, edges)

        if time_check:
            print(f"total time: {t6 - t1}")
            print(f"data read: {t2 - t1}")
            print(f"find target point: {t3 - t2}")
            print(f"modify target point: {t4 - t3}")
            print(f"send to arduino: {t6 - t4}\n")

        if save_image:
            # env.save_file(image, "output/image" + str(save_num))
            save_num += 1

        # Process Termination (If you input the 'q', camera scanning is ended.)
        key = env.wait_key()
        if key == "quit":
            break
        elif key == "next":
            if img_num == 46:
                print("Final image!!")
            else:
                img_num += 1
                print("image number: " + str(img_num))
        elif key == "prev":
            if img_num == 0:
                print("First image!!")
            else:
                img_num -= 1
                print("image number: " + str(img_num))
        elif key == "save":
            env.save_file(image, "data/image")

        # env.save_file(frame0, "data/image")
