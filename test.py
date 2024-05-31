import Function_Library as fl
import time

EPOCH = 500000
num=0

# ser = fl.libARDUINO()
arduino_port = "/dev/cu.usbmodem2101"
lidar_port = "/dev/cu.usbserial-0001"

env = fl.libCAMERA(wait_value=0)
img_num = 0
ser = fl.libARDUINO()
comm = ser.init(arduino_port, 9600)
# ch0, ch1 = env.initial_setting(capnum=1)

# lidar = fl.libLIDAR(lidar_port)

frameNum = 0

print(f"fetching first data from lidar")
t1 = time.time()
# lidar.fetch_scanning()

print(f"main function starts")
for i in range(EPOCH):

    # print(f"frameNum: {frameNum}")
    image = env.file_read("data/image"+str(img_num)+".png")
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
    env.image_show(image)

    # print(f"total time: {t2-t1}")
    # env.send_signal_to_arduino(comm, 100+i, 8+i)

    env.send_signal_to_arduino(comm, 100+i, 200+i, 10)

    frameNum += 1

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
    # elif key == "save":
        # env.save_file(image, "data/image")



