import Function_Library as fl
import Lib_LiDAR as LiDAR
import time

# window: "COM5", mac: "/dev/cu.xxxxx"
arduino_port = "/dev/cu.usbmodem214201"
lidar_port = "/dev/cu.usbserial-0001"
img_num = 100

if __name__ == "__main__":
    # Exercise Environment Setting
    # camera
    env = fl.libCAMERA(wait_value=0)
    time_check = False
    crosswalk = True
    # arduino
    ser = fl.libARDUINO()
    # comm = ser.init(arduino_port, 9600)

    # Camera Initial Setting
    # ch0, ch1 = env.initial_setting(capnum=1)
    # Camera using Thread
    # env.fetch_image_camera(channel=ch0)

    # input("if start, press ENTER!!")

    # Camera Reading..
    while True:
        # reading source
        if time_check:
            t1 = time.time()
        # _, image = env.camera_read(ch0)
        # Camera using Thread
        # image = env.read_image_thread()
        # env.fetch_image_camera(channel=ch0)

        image = env.file_read("data/image"+str(img_num)+".png")

        # extracting valid region from image
        if time_check:
            t2 = time.time()
        resize_image = env.resize_image(image)
        valid_image = env.extract_valid_image(resize_image)

        # extract edges to 1d
        if time_check:
            t3 = time.time()
        edges = env.convert_image_to_1d(valid_image)

        # find crosswalk, if crosswalk, stop for 2 sec
        if crosswalk:
            crosswalk_image = env.convert_crosswalk_image(image)
            if env.find_crosswalk(crosswalk_image):
                crosswalk = False
                # print("find crosswalk!!")
                # env.send_signal_to_arduino(comm, 0, 0)
                time.sleep(2)
                # continue

        # get coordinate of the target: ans
        if time_check:
            t4 = time.time()
        result, ans = env.edge_detection(edges)

        # get speed and angle of the car
        if time_check:
            t5 = time.time()
        speed, angle = env.get_speed_angle(ans)

        # send data to arduino
        if time_check:
            t6 = time.time()
        # env.send_signal_to_arduino(comm, speed, angle)

        # print image of final results
        if time_check:
            t7 = time.time()
        env.image_show(result, edges, image)
        # env.image_show(image)

        if time_check:
            t8 = time.time()
            print(f"file read: {t2-t1}")
            print(f"extract region: {t3-t2}")
            print(f"extract edges: {t4-t3}")
            print(f"get target: {t5-t4}")
            print(f"get speed, angle: {t6-t5}")
            print(f"send to arduino: {t8-t6}")
            print(f"time without getting source: {t8-t2}")
            print(f"total time: {t8-t1}")

        # Process Termination (If you input the 'q', camera scanning is ended.)
        key = env.wait_key()
        if key == "quit":
            break
        elif key == "next":
            if (img_num == 50):
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


