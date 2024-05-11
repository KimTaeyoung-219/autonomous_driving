import cv2
import Function_Library as fl
import threading
import time

EPOCH = 500000
num=0

# ser = fl.libARDUINO()
arduino_port = "/dev/cu.usbmodem214201"

env = fl.libCAMERA(wait_value=10)
img_num = 0
# ser = fl.libARDUINO()
# comm = ser.init(arduino_port, 9600)
ch0, ch1 = env.initial_setting(capnum=1)

for i in range(EPOCH):

    t1 = time.time()
    # 이미지 읽기
    # image = env.file_read("data/image"+str(img_num)+".png")
    _, image = env.camera_read(ch0)

    # 이미지 보여주기
    env.image_show(image)

    t2 = time.time()

    print(f"total time: {t2-t1}")

    # env.send_signal_to_arduino(comm, 100, 10)

    # n 누르면 다음 이미지
    # q 누르면 종료
    # p 누르면 이전 이미지
    key = cv2.waitKey(10) & 0xFF
    if key == ord('n'):
        print("next image")
        img_num+=1
    if key == ord('q'):
        break



