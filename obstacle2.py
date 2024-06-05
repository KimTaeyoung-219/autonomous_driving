import Function_Library as fl
import time

# window: "COM5", mac: "/dev/cu.xxxxx"
arduino_port = "/dev/cu.usbmodem1201"
lidar_port = "/dev/cu.usbserial-0001"
img_num = 0

def print_stage(stage, flag):
    if flag is False:
        return
    print(f"-------------------")
    print(f"------{stage}------")
    print(f"-------------------")

def go_forward(env, image):
    resize_image = env.resize_image(image)
    valid_image = env.extract_valid_image(resize_image)
    edges = env.convert_image_to_1d(valid_image)
    result, ans = env.edge_detection(edges)
    return result, ans, edges

if __name__ == "__main__":
    # Exercise Environment Setting
    # camera
    env = fl.libCAMERA(wait_value=10, max_speed=70)
    time_check = False
    stage_check = True
    # change lane to ${change_lane}
    change_lane = "left"
    pred = 100
    a = pred
    stage = 1
    # arduino
    ser = fl.libARDUINO()
    lidar = None
    comm = ser.init(arduino_port, 9600)
    lidar = fl.libLIDAR(lidar_port)

    # Camera Initial Setting
    ch0, ch1 = env.initial_setting(capnum=1)
    # Camera using Thread
    env.fetch_image_camera(channel=ch0)
    # LiDAR using Thread
    lidar.fetch_scanning()

    # input("if start, press ENTER!!")

    # Camera Reading..
    print_stage("STAGE 1", stage_check)
    while True:
        # reading source
        if time_check:
            t1 = time.time()
        # _, image = env.camera_read(ch0)
        # Camera using Thread
        image = env.read_image_thread()
        env.fetch_image_camera(channel=ch0)

        # LiDAR using Thread
        # env.fetch_image_camera(channel=ch0)
        if lidar.check_scanning() is True:
            lidar_data = lidar.read_scanning()

        # image = env.file_read("data/image"+str(img_num)+".png")

        # set target point with image
        if time_check:
            t2 = time.time()
        result, ans, edges = go_forward(env, image)

        # custom the target point by obstacle detection, lidar data, crosswalk
        if time_check:
            t3 = time.time()
        if stage == 1:  # when starts, change lane to left immediately
            if lidar.check_scanning() == True:
                lidar_data = lidar.read_scanning()
                flag = lidar.getAngleDistanceRange(lidar_data, 170, 190, 1600, 2000)
                if flag == True:
                    change_lane = "left"
                    print_stage("STAGE 2", stage_check)
                    stage = 2
        elif stage == 2: # stay in line when the car move to the left side of obstacle1
            edges = env.find_car_lane(edges, ans)
            if env.cur_lane == "right" or a != 0:
                ans = env.change_car_lane(ans, change_lane)
                a -= 1
            else:
                print_stage("STAGE 3", stage_check)
                a = pred
                stage = 3
        elif stage == 3:  # when find obstacle2 in front, change lane to right
            if lidar.check_scanning() == True:
                lidar_data = lidar.read_scanning()
                flag = lidar.getAngleDistanceRange(lidar_data, 170, 190, 1600, 2000)
                if flag == True:
                    change_lane = "right"
                    print_stage("STAGE 4", stage_check)
                    stage = 4
        elif stage == 4:  # change lane to right
            edges = env.find_car_lane(edges, ans)
            if env.cur_lane == "left" or a != 0:
                ans = env.change_car_lane(ans, change_lane)
                a -= 1
            else:
                print_stage("STAGE 5", stage_check)
                a = pred
                stage = 5
        elif stage == 5:  # when find crosswalk_image, stop for 2 sec and move straight
            crosswalk_image = env.convert_crosswalk_image(image)
            if env.find_crosswalk(crosswalk_image):
                env.send_signal_to_arduino(comm, 0, 14)
                time.sleep(2)
                env.send_signal_to_arduino(comm, 60, 14)
                lidar.stop()
                print(f"obstacle terminated")
                break

        # clear lidar buffer when stage is not using lidar data
        if stage == 2 or stage == 4 or stage == 5:
            if lidar.check_scanning() is True:
                lidar_data = lidar.read_scanning()

        # get speed and angle of the car
        if time_check:
            t4 = time.time()
        speed, angle = env.get_speed_angle(ans)

        # send data to arduino
        if time_check:
            t5 = time.time()
        env.send_signal_to_arduino(comm, speed, angle)

        # print image of final results
        if time_check:
            t6 = time.time()
        env.image_show(result, edges)
        # env.image_show(image, edges)

        if time_check:
            print(f"total time: {t6-t1}")
            print(f"data read: {t2-t1}")
            print(f"find target point: {t3-t2}")
            print(f"modify target point: {t4-t3}")
            print(f"send to arduino: {t6-t4}\n")

        # Process Termination (If you input the 'q', camera scanning is ended.)
        key = env.wait_key()
        if key == "quit":
            break
        elif key == "next":
            if (img_num == 46):
                print("Final image!!")
            else:
                img_num += 1
                print("image number: " + str(img_num))
        elif key == "prev":
            if (img_num == 0):
                print("First image!!")
            else:
                img_num -= 1
                print("image number: " + str(img_num))
        elif key == "save":
            env.save_file(image, "data/image")

        # env.save_file(frame0, "data/image")


