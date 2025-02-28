import Function_Library as fl
import time
import cv2

EPOCH = 500000
num=0

# ser = fl.libARDUINO()
arduino_port = "/dev/cu.usbmodem1301"
lidar_port = "/dev/cu.usbserial-0001"
video_file = "data/testVideo.mp4"

env = fl.libCAMERA(wait_value=10)
img_num = 250
ser = fl.libARDUINO()
comm = ser.init(arduino_port, 9600)
ch0, ch1 = env.initial_setting(capnum=2)

lidar = fl.libLIDAR(lidar_port)

frameNum = 0

print(f"fetching first data from lidar")
t1 = time.time()
# cap = cv2.VideoCapture(video_file)
# env.fetch_image_video(cap)
# lidar.fetch_scanning()
env.send_signal_to_arduino(comm, 60, 144)

t = 0.08
print(f"main function starts")
for i in range(EPOCH):
    _, image, _, image2 = env.camera_read(ch0, ch1)
    env.image_show(image, image2)
    # frameNum += 1
    # if frameNum % 2 == 1:
    #     image = env.file_read("output/crosswalk3.jpg")
    # else:
    #     image = env.file_read("output/crosswalk4.jpg")
    # env.traffic_light_detection(image, print_enable=True)
    # env.image_show(image)
    # image = env.file_read("output/image" + str(img_num) + ".png")

    # Find red color in the image
    # red_detected_image = find_red_color(image, print_enable=True)
    # green_detected_image = env.find_red_traffic_light(image, print_enable=True)
    # crosswalk_image = env.convert_crosswalk_image(image)
    # flag = env.find_crosswalk(crosswalk_image)
    # env.image_show(crosswalk_image)

    # image = env.read_image_thread()
    # env.fetch_image_video(cap)
    # env.send_signal_to_arduino(comm, 0, 14)
    # time.sleep(100)

    # image = env.file_read("data/image"+str(img_num)+".png")
    # resize_image = env.resize_image(image)
    # valid_image = env.extract_valid_image(resize_image)
    #
    # edges = env.convert_image_to_1d(valid_image)
    # crosswalk_image = env.convert_crosswalk_image(image)
    #
    # flag = env.find_crosswalk(crosswalk_image)
    # print(crosswalk_image.shape)
    # print(flag)
    #
    # env.image_show(image, edges, crosswalk_image)

    # for i in range(0, 28, 1):
    #     coord = i
    #     # coord = i % 28
    #     # if coord <= 0:
    #     #     coord = 0
    #     # elif coord > 0 and coord <= 4:
    #     #     coord = 4
    #     # elif coord > 4 and coord <= 10:
    #     #     coord = 10
    #     # elif coord > 10 and coord < 18:
    #     #     coord = 14
    #     # elif coord >= 18 and coord < 24:
    #     #     coord = 18
    #     # elif coord >= 24 and coord < 28:
    #     #     coord = 24
    #     # elif coord >= 28:
    #     #     coord = 28
    #     print(coord)
    #     if coord==0:
    #         coord +=1
    #     env.send_signal_to_arduino(comm, 0, coord -1)
    #     time.sleep(t)
    # time.sleep(0.2)
    # for i in range(28, 0, -1):
    #     coord = i
    #     # coord = i % 28
    #     # if coord <= 0:
    #     #     coord = 0
    #     # elif coord > 0 and coord <= 4:
    #     #     coord = 4
    #     # elif coord > 4 and coord <= 10:
    #     #     coord = 10
    #     # elif coord > 10 and coord < 18:
    #     #     coord = 14
    #     # elif coord >= 18 and coord < 24:
    #     #     coord = 18
    #     # elif coord >= 24 and coord < 28:
    #     #     coord = 24
    #     # elif coord >= 28:
    #     #     coord = 28
    #     print(coord)
    #     if coord==0:
    #         coord +=1
    #     env.send_signal_to_arduino(comm, 0, coord - 1)
    #     time.sleep(t)

    # image = env.read_image_thread()
    # env.fetch_image_video(cap)
    # _, image = cap.read()
    # print(f"frameNum: {frameNum}")
    # image = env.file_read("data/image"+str(img_num)+".png")
    # # _, image = env.camera_read(ch0)
    for scan in lidar.scanning():
        # t2 = time.time()
        # print(f"liDAR data get in frame")
        # t1 = t2
        # scan = lidar.read_scanning()
        # scan = lidar.getAngleDistanceRange(scan, 280, 300, 100, 2000)
        flag = lidar.getAngleDistanceRange(scan, 268, 273, 100, 1000)
        flag2 = lidar.getAngleDistanceRange(scan, 268, 273, 1600, 2000)
        # if scan:
        #     print(f"stop!!!!!: {frameNum}")
        #     frameNum += 1
        if flag:
            print(f"1 :{frameNum}")
            frameNum += 1
        if flag2:
            print(f"2 :{frameNum}")
            frameNum += 1
    # print("Start!")
    # t3 = time.time()
    # env.send_signal_to_arduino(comm, 70, 168)
    # while True:
    #     t4 = time.time()
    #     if t4 - t3 > 10:
    #         break
    # print("Stop!")
    # env.send_signal_to_arduino(comm, 0, 140)

    # # 이미지 보여주기
    # print((image.shape))
    # resize_image = env.resize_image(image, new_shape=(720, 2200, 3))

    # coord = i % 28
    # if coord <= 0:
    #     coord = 0
    # elif coord > 0 and coord <= 4:
    #     coord = 4
    # elif coord > 4 and coord <= 10:
    #     coord = 10
    # elif coord > 10 and coord < 18:
    #     coord = 14
    # elif coord >= 18 and coord < 24:
    #     coord = 18
    # elif coord >= 24 and coord < 28:
    #     coord = 24
    # elif coord >= 28:
    #     coord = 28

    # print(f"coord: {coord}")

    # print(f"total time: {t2-t1}")
    # env.send_signal_to_arduino(comm, 100+i, 8+i)

    # print(f"sending coord: {coord}")
    # env.send_signal_to_arduino(comm, 0, coord)

    # frameNum += 1
    # time.sleep(0.05)

    # n 누르면 다음 이미지
    # q 누르면 종료
    # p 누르면 이전 이미지
    key = env.wait_key()
    if key == "quit":
        # lidar.stop()
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



