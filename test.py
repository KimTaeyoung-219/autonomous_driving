import Function_Library as fl
import time
import cv2

EPOCH = 500000
num=0

# ser = fl.libARDUINO()
arduino_port = "/dev/cu.usbmodem1201"
lidar_port = "/dev/cu.usbserial-0001"
video_file = "data/testVideo.mp4"

env = fl.libCAMERA(wait_value=0)
img_num = 0
ser = fl.libARDUINO()
# comm = ser.init(arduino_port, 9600)
# ch0, ch1 = env.initial_setting(capnum=1)

# lidar = fl.libLIDAR(lidar_port)

frameNum = 0

print(f"fetching first data from lidar")
t1 = time.time()
cap = cv2.VideoCapture(video_file)
env.fetch_image_video(cap)
# lidar.fetch_scanning()

t = 0.03
print(f"main function starts")
for i in range(EPOCH):
    image = env.read_image_thread()
    env.fetch_image_video(cap)

    # image = env.read_image_thread()
    # env.fetch_image_video(cap)
    # _, image = cap.read()
    # print(f"frameNum: {frameNum}")
    # image = env.file_read("data/image"+str(img_num)+".png")
    # # _, image = env.camera_read(ch0)
    # if lidar.check_scanning() is True:
    #     t2 = time.time()
    #     print(f"liadr data get in frame {frameNum}, time: {t2-t1}")
    #     t1 = t2
    #     scan = lidar.read_scanning()
    #     scan = lidar.getAngleDistanceRange(scan, 170, 190, 200, 250)
    #     if len(scan) > 0:
    #         print("stop!!!!!")
    #
    # # 이미지 보여주기
    print((image.shape))
    # resize_image = env.resize_image(image, new_shape=(720, 2200, 3))
    env.X_length = 1280
    env.Y_length = 720
    env.belowX = 100
    env.upperX = 600
    env.belowY = 50
    env.upperY = 150
    valid_image = env.extract_valid_image(image)
    edges = env.convert_image_to_1d(valid_image)
    result, ans = env.edge_detection(edges)
    #
    env.image_show(edges, valid_image, image)
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

    frameNum += 1
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



