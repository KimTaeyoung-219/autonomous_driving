"""
-------------------------------------------------------------------
  FILE NAME: Function_Library.py
  Copyright: Sungkyunkwan University, Automation Lab.
-------------------------------------------------------------------
  This file is included library class for below subject.
  1) Arduino
  2) LiDAR
  3) Camera
-------------------------------------------------------------------
  Authors: Jonghun Kim, YoungSoo Do, SungBhin Oh, HyeongKeun Hong

  Generated: 2022-11-10
  Revised: 2022-11-18
-------------------------------------------------------------------
  If you find some wrong code, plz contact me(Main Author: Jonghun Kim).
-------------------------------------------------------------------
  You should never modify this file during workshop exercise.
-------------------------------------------------------------------
"""

import sys
import cv2                           # pip install opencv
import time
import serial                        # pip install serial
import numpy as np                   # pip install numpy
import matplotlib.pyplot as plt      # pip install matplotlib
from rplidar import RPLidar          # pip install rplidar-roboticia
import struct
import threading
import queue


np.set_printoptions(threshold=sys.maxsize, linewidth=150)

"""------------------Arduino Variable------------------"""
WAIT_TIME = 2
"""----------------------------------------------------"""


"""-------------------LIDAR Variable-------------------"""
SCAN_TYPE = "normal"
SAMPLE_RATE = 10
MAX_BUFFER_SIZE = 3000
MIN_DISTANCE = 0
"""----------------------------------------------------"""


"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30
SATURATION = 150
FORWARD_THRESHOLD = 0.3
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)
FORWARD, LEFT, RIGHT = (0, 1, 2)
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
DIRECTION = ("FORWARD", "LEFT", "RIGHT")
HUE_THRESHOLD = ([4, 176], [40, 80], [110, 130], [20, 40])
"""-----------------------------------------------------"""


"""
-------------------------------------------------------------------
  CLASS PURPOSE: Arduino Exercise Library
  Author: SungBhin Oh
  Revised: 2022-11-14
-------------------------------------------------------------------
"""
# noinspection PyMethodMayBeStatic
class libARDUINO(object):
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.wait_time = WAIT_TIME  # second unit

    # Arduino Serial USB Port Setting
    def init(self, port, baudrate):
        ser = serial.Serial()
        ser.port, self.port = port, port
        ser.baudrate, self.baudrate = baudrate, baudrate
        ser.open()
        time.sleep(self.wait_time)
        return ser


"""
-------------------------------------------------------------------
  CLASS PURPOSE: LiDAR Sensor Exercise Library
  Author: YoungSoo Do
  Revised: 2022-11-18
-------------------------------------------------------------------
"""
class libLIDAR(object):
    def __init__(self, port):
        self.rpm = 0
        self.lidar = RPLidar(port)
        self.scan = []
        self.q = queue.Queue()
        self.t = None
        self.thread_event = None

    def init(self):
        info = self.lidar.get_info()
        print(info)

    def getState(self):
        health = self.lidar.get_health()
        print(health)

    def scanning(self):
        scan_list = []
        iterator = self.lidar.iter_measures(SCAN_TYPE, MAX_BUFFER_SIZE)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan_list) > SAMPLE_RATE:
                    np_data = np.array(list(scan_list))
                    yield np_data[:, 1:]
                scan_list = []
            if distance > MIN_DISTANCE:
                scan_list.append((quality, angle, distance))

    def fetch_scanning(self):
        self.thread_event = threading.Event()
        self.t = threading.Thread(target=self.fetch_scanning_threads, args=(self.thread_event,))
        self.t.start()
        return

    def check_scanning(self):
        # if self.t is not None and self.t.is_alive():
        #     return False
        if self.q.empty():
            return False
        return True

    def read_scanning(self):
        data = self.q.get()
        return data

    def fetch_scanning_threads(self, event):
        scan_list = []
        iterator = self.lidar.iter_measures(SCAN_TYPE, MAX_BUFFER_SIZE)
        for new_scan, quality, angle, distance in iterator:
            if event.is_set():
                return
            if new_scan:
                if len(scan_list) > SAMPLE_RATE:
                    np_data = np.array(list(scan_list))
                    self.q.put(np_data[:, 1:])
                scan_list = []
            if distance > MIN_DISTANCE:
                scan_list.append((quality, angle, distance))

    def stop(self):
        self.thread_event.set()
        if self.t is not None:
            self.t.join()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def setRPM(self, rpm):
        self.lidar.motor_speed = rpm

    def getRPM(self):
        return self.lidar.motor_speed

    def getAngleRange(self, scan, minAngle, maxAngle):
        data = np.array(scan)
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle))
        # return data[condition]
        result = True if len(data[condition]) > 0 else False
        return result

    def getDistanceRange(self, scan, minDist, maxDist):
        data = np.array(scan)
        condition = np.where((data[:, 1] < maxDist) & (data[:, 1] > minDist))
        # return data[condition]
        result = True if len(data[condition]) > 0 else False
        return result

    def getAngleDistanceRange(self, scan, minAngle, maxAngle, minDist, maxDist):
        data = np.array(scan)
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle) & (data[:, 1] < maxDist) & (data[:, 1] > minDist))
        # return data[condition]
        result = True if len(data[condition]) > 0 else False
        return result

    def get_far_distance(self, scan, minAngle, maxAngle):
        datas = self.getAngleRange(scan, minAngle, maxAngle)
        max_idx = datas[:, 1].argmax()
        return datas[max_idx]

    def get_near_distance(self, scan, minAngle, maxAngle):
        datas = self.getAngleRange(scan, minAngle, maxAngle)
        min_idx = datas[:, 1].argmin()
        return datas[min_idx]


"""
-------------------------------------------------------------------
  CLASS PURPOSE: Camera Sensor Exercise Library
  Author: Jonghun Kim
  Revised: 2022-11-12
-------------------------------------------------------------------
"""
# noinspection PyMethodMayBeStatic
class libCAMERA(object):
    def __init__(self, wait_value = 0, max_speed = 120):
        self.capnum = 0
        self.wait_value = wait_value
        self.image_num = 200
        self.row, self.col, self.dim = (0, 0, 0)
        self.belowX = 200
        self.upperX = 420
        self.belowY = 50
        self.upperY = 900
        self.X_length = 1920
        self.Y_length = 1080
        self.valid_Y = 480
        self.valid_X = 640
        self.valid_image = None
        self.edges = None
        self.result = np.zeros((self.valid_Y, self.valid_X))
        self.center_point = (470, 320)
        self.max_speed = max_speed
        self.max_angle = 45
        self.max_voltage = 16
        self.max_lane_width = 25
        self.min_lane_width = 10
        self.t = None
        self.direction = None
        self.queue = queue.Queue()
        self.left_lane = None
        self.right_lane = None
        self.cur_lane = "right"
        self.stage="NONE"

    def wait_key(self):
        key = cv2.waitKey(self.wait_value) & 0xFF
        if key == ord('q'):
            return "quit"
        elif key == ord('n'):
            return "next"
        elif key == ord('p'):
            return "prev"
        elif key == ord('s'):
            return "save"

    def loop_break(self):
        if cv2.waitKey(10) & 0xFF == ord('q'):
            print("Camera Reading is ended.")
            return True
        else:
            return False

    def next_step(self):
        if cv2.waitKey(100) & 0xFF == ord('n'):
            print("Next image printed.")
            return True
        else:
            return False

    def prev_step(self):
        if cv2.waitKey(0) & 0xFF == ord('p'):
            print("prev image printed.")
            return True
        else:
            return False

    def save_file(self, img, img_path):
        # if cv2.waitKey(10) & 0xFF == ord('s'):
        img_path = img_path + str(self.image_num) + ".png"
        print("Saving image: " + img_path)
        self.image_num += 1
        cv2.imwrite(img_path, img)

    def file_read(self, img_path):
        return np.array(cv2.imread(img_path))

    def read_image_thread(self):
        self.t.join()
        image = self.queue.get()
        return image

    def fetch_image_camera(self, channel):
        self.t = threading.Thread(target=self.fetch_image_camera_thread, args=(channel, self.queue))
        self.t.start()
        return

    def fetch_image_camera_thread(self, channel, q):
        _, image = self.camera_read(channel)
        q.put(image)

    def fetch_image_video(self, cap):
        self.t = threading.Thread(target=self.fetch_image_video_thread, args=(cap, self.queue))
        self.t.start()
        return

    def fetch_image_video_thread(self, cap, q):
        _, image = cap.read()
        q.put(image)
        return

    def initial_setting(self, cam0port=0, cam1port=1, capnum=1):
        # OpenCV Initial Setting
        print("OpenCV Version:", cv2.__version__)
        channel0 = None
        channel1 = None
        self.capnum = capnum

        if capnum == 1:
            channel0 = cv2.VideoCapture(cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")
            else:
                print("Camera Channel0 is not opened!")
        elif capnum == 2:
            channel0 = cv2.VideoCapture(cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")
            else:
                print("Camera Channel0 is not opened!")

            channel1 = cv2.VideoCapture(cam1port)
            if channel1.isOpened():
                print("Camera Channel1 is enabled!")
            else:
                print("Camera Channel1 is not opened!")

        return channel0, channel1

    def camera_read(self, cap1, cap2=None):
        result, capset = [], [cap1, cap2]

        for idx in range(0, self.capnum):
            ret, frame = capset[idx].read()
            result.extend([ret, frame])

        return result

    def image_show(self, frame0, frame1=None, frame2=None, frame3=None):
        if frame1 is None:
            cv2.imshow('frame0', frame0)
        else:
            cv2.imshow('frame0', frame0)
            cv2.imshow('frame1', frame1)
        if frame2 is not None:
            cv2.imshow('frame2', frame2)
        if frame3 is not None:
            cv2.imshow('frame3', frame3)

    def draw_dot(self, img, x, y):
        # print(f"x,y: {x},{y}")
        for i in range(-2, 3):
            for j in range(-2, 3):
                img[x + i][y + j] = 155
        return img

    def draw_car_line(self, img, result, x, y, y2):
        flag = True
        mid = (int)((y + y2) / 2)
        for i in range(1, 30):
            if not flag:
                break
            right = mid
            left = mid
            img[x - i][mid] = 155

            for j in range(1, 20):
                if mid + j == 640:
                    right = mid + j - 1
                    break
                elif img[x - i][mid + j] == 255:
                    right = mid + j
                    break
            for j in range(1, 20):
                if mid - j == 0:
                    left = mid - j
                    break
                elif img[x - i][mid - j] == 255:
                    left = mid - j
                    break

            if right - left < 3:
                flag = False
                continue
            for y3 in range(left, right + 1):
                result[x - i][y3] = 155
            mid = (int)((left + right) / 2)
        return

    def set_valid_image_parameter(self, belowX=50, upperX=600):
        self.belowX = belowX
        self.upperX = upperX
        return

    def resize_image(self, pre_image, new_shape=(1080, 2920, 3)):
        self.X_length = 2920
        image = np.zeros(new_shape, dtype=np.uint8)
        start_index = (new_shape[1] - pre_image.shape[1]) // 2
        end_index = start_index + pre_image.shape[1]

        image[:, start_index:end_index, :] = pre_image
        self.set_valid_image_parameter(belowX=50, upperX=980)
        return image

    def extract_valid_image(self, image):
        # self.belowX = 280
        # self.upperX = 560
        # self.belowY = 50
        # self.upperY = 600
        # self.X_length = 1920
        # self.Y_length = 1080
        self.result = np.zeros((self.valid_Y, self.valid_X))
        pts1 = np.float32([[self.belowX, self.Y_length - self.belowY],
                           [self.X_length - self.belowX, self.Y_length - self.belowY],
                           [self.upperX, self.Y_length - self.upperY],
                           [self.X_length - self.upperX, self.Y_length - self.upperY]])
        pts2 = np.float32([[0, self.valid_Y], [self.valid_X, self.valid_Y], [0, 0], [self.valid_X, 0]])

        red = (0, 0, 225)

        cv2.line(image, (self.belowX, self.Y_length - self.belowY),
                 (self.X_length - self.belowX, self.Y_length - self.belowY), red, 5)
        cv2.line(image, (self.X_length - self.belowX, self.Y_length - self.belowY),
                 (self.X_length - self.upperX, self.Y_length - self.upperY), red, 5)
        cv2.line(image, (self.upperX, self.Y_length - self.upperY),
                 (self.X_length - self.upperX, self.Y_length - self.upperY), red, 5)
        cv2.line(image, (self.upperX, self.Y_length - self.upperY),
                 (self.belowX, self.Y_length - self.belowY), red, 5)

        mtrx = cv2.getPerspectiveTransform(pts1, pts2)
        self.valid_image = cv2.warpPerspective(image, mtrx, (self.valid_X, self.valid_Y))

        return self.valid_image

    def convert_image_to_1d(self, image):
        self.edges = cv2.Canny(image, 60, 300)
        return self.edges

    def convert_crosswalk_image(self, image):
        lower_white = np.array([160, 160, 160])
        upper_white = np.array([255, 255, 255])
        crosswalk = cv2.inRange(image, lower_white, upper_white)
        return crosswalk

    def check_upper(self, img, result, x, y):
        # 점 위로 선이 연결 되어 있는지 확인
        # 10칸 이상 연결 되어 있으면 차 선으로 판단 후 True 반환
        uy = y
        for i in range(20):
            if img[x - i][uy] == 255:
                # img[x-i][uy]=155
                # result[x - i][uy] = 155
                uy = uy
            elif img[x - i][uy - 1] == 255:
                # img[x - i][uy-1] = 155
                # result[x - i][uy-1] = 155
                uy = uy - 1
            elif img[x - i][uy + 1] == 255:
                # img[x - i][uy+1] = 155
                # result[x - i][uy+1] = 155
                uy = uy + 1
            elif img[x - i][uy - 2] == 255:
                # img[x - i][uy-1] = 155
                # result[x - i][uy-1] = 155
                uy = uy - 2
            elif img[x - i][uy + 2] == 255:
                # img[x - i][uy-1] = 155
                # result[x - i][uy-1] = 155
                uy = uy + 2
            else:
                return False
        return True

    def find_inclination(self, img, result, x, y):
        uy = y
        cx = 0
        ux = x
        for i in range(20):
            ux = x - i
            if (x - i) == 0:
                # print("a")
                return None
            if uy == 0 or uy == self.valid_X:
                # print("b")
                return None
            if img[x - i][uy] == 255:
                uy = uy
                cx += 1
            elif img[x - i][uy - 1] == 255:
                uy = uy - 1
                cx += 1
            elif img[x - i][uy + 1] == 255:
                uy = uy + 1
                cx += 1
            elif img[x - i][uy - 2] == 255:
                uy = uy - 2
                cx += 1
            elif img[x - i][uy + 2] == 255:
                uy = uy + 2
                cx += 1
            else:
                return None
        ans = (ux, uy)
        return ans

    def count_upper(self, img, x, y):
        uy = y
        count = 0
        for i in range(100):
            # print("asdjlkf")
            if (x - i) == 0:
                # print("a")
                return count
            if uy == 0 or uy == self.valid_X:
                # print("b")
                return count
            if img[x - i][uy] == 255:
                uy = uy
                count += 1
            elif img[x - i][uy - 1] == 255:
                uy = uy - 1
                count += 1
            elif img[x - i][uy + 1] == 255:
                uy = uy + 1
                count += 1
            elif img[x - i][uy - 2] == 255:
                uy = uy - 2
                count += 1
            elif img[x - i][uy + 2] == 255:
                uy = uy + 2
                count += 1
            else:
                return count
        return count

    def get_line_Canny_with_X_left(self, img, result, x, Y):
        ans = None
        flag = False
        for y in range(Y, 40, -1):
            pre = []
            if flag is True:
                break
            if img[x][y] == 255:
                if not self.check_upper(img, result, x, y):
                    continue
                for inc in range(self.min_lane_width, self.max_lane_width):
                    if img[x][y - inc] == 255:
                        pre.append(y - inc)
                for y2 in pre:
                    if self.check_upper(img, result, x, y2):
                        # res2 = get_num_line(img, result, x, y, y2)
                        # if res2 > min_num:
                        #     min_num = res2
                        ans = (y, y2)
                        flag = True
                        self.draw_car_line(img, result, x, y2, y)
                        # for y3 in range(y, y2):
                        #     img[x][y3] = 155
                        #     result[x][y3]=155
                        return result, ans
        # if ans is not None:
        #     draw_car_line(img, result, x, ans[0], ans[1])
        return result, ans

    def get_line_Canny_with_X_right(self, img, result, x, Y):
        ans = None
        flag = False
        for y in range(Y, 599):
            pre = []
            if flag is True:
                break
            if img[x][y] == 255:
                if not self.check_upper(img, result, x, y):
                    continue
                for inc in range(self.min_lane_width, self.max_lane_width):
                    if img[x][y + inc] == 255:
                        pre.append(y + inc)
                for y2 in pre:
                    if self.check_upper(img, result, x, y2):
                        # res2 = get_num_line(img, result, x, y, y2)
                        # if res2 > min_num:
                        #     min_num = res2
                        ans = (y, y2)
                        flag = True
                        self.draw_car_line(img, result, x, y, y2)
                        # for y3 in range(y, y2):
                        #     img[x][y3] = 155
                        #     result[x][y3]=155
                        return result, ans
        # if ans is not None:
        #     self.draw_car_line(img, result, x, ans[0], ans[1])
        return result, ans

    def edge_detection(self, image, lines = [450, 400]):
        ans = []
        X = lines
        Y = self.center_point[1]
        for i in range(self.valid_X):
            for x in X:
                self.result[x][i] = 155
                image[x][i] = 255
        diff, before, flag = None, None, None
        jump = 10
        # find target point using 2 car lanes
        # if we find both car lane, we set the target point in middle of car lanes
        for x in X:
            for inc in range(5):
                self.result, left = self.get_line_Canny_with_X_left(image, self.result, x - (inc * jump), Y)
                self.result, right = self.get_line_Canny_with_X_right(image, self.result, x - (inc * jump), Y)
                # print(f"{x-(inc*10)}, {Y}")
                # self.draw_dot(self.result, x - (inc * jump), Y)

                if (left is not None) and (right is not None):
                    width = int(right[0]-left[0])
                    # width between two car line should be in range 150~250
                    if width > 350 or width < 150:
                        continue
                    # self.left_lane = int((left[0]+left[1])/2)
                    # self.right_lane = int((right[0]+right[1])/2)
                    self.left_lane = left[0]
                    self.right_lane = right[0]
                    Y = int((left[0] + right[0]) / 2)
                    ans.append((x - (inc * jump), Y))
                    self.draw_dot(self.result, x - (inc * jump), Y)
                    self.draw_dot(image, x - (inc * jump), Y)
                    return self.result, ans
                if diff is None:
                    if self.cur_lane == "left" and left is not None:
                        diff = self.find_inclination(image, self.result, x - (inc * jump), left[0])
                        if diff is not None:
                            before = (x - (inc * jump), left[0])
                            # self.draw_dot(image, before[0], before[1])
                            # self.draw_dot(image, diff[0], diff[1])
                            self.left_lane = left[0]
                            self.right_lane = 2 * self.center_point[1] - left[0]
                            diff = (diff[0] - before[0], diff[1] - before[1])
                    elif self.cur_lane == "right" and right is not None:
                        diff = self.find_inclination(image, self.result, x - (inc * jump), right[0])
                        if diff is not None:
                            before = (x - (inc * jump), right[0])
                            # self.draw_dot(image, before[0], before[1])
                            # self.draw_dot(image, diff[0], diff[1])
                            self.left_lane = 2 * self.center_point[1]-before[1]
                            self.right_lane = before[1]
                            diff = (diff[0] - before[0], diff[1] - before[1])
            if flag is True:
                break
        # if vision cannot find both car lane and cannot specify the target point
        # calculate the target point by inclination of single car lane
        if diff is not None:
            ans.append((self.center_point[0]+diff[0], self.center_point[1]+diff[1]))
            # self.draw_dot(image, self.center_point[0], self.center_point[1])
            self.draw_dot(image, self.center_point[0]+diff[0], self.center_point[1]+diff[1])
            self.draw_dot(self.result, self.center_point[0] + diff[0], self.center_point[1] + diff[1])
            # self.draw_dot(self.result, ans[0][0], ans[0][1])
            # self.draw_dot(self.result, before[0], self.left_lane)
            # self.draw_dot(self.result, before[0], self.right_lane)
            return self.result, ans
        return self.result, ans

    def get_speed_angle(self, ans):
        if ans is None:
            return None, None
        self.draw_dot(self.result, self.center_point[0], self.center_point[1])
        if len(ans) != 0:
            x = ans[0][0]
            y = ans[0][1]
            angle = y - self.center_point[1]
            tangent = angle / (self.center_point[0] - x + 130)
            inverse_tan = np.arctan(tangent)
            angle_in_degrees = inverse_tan * (180 / np.pi)

            # 1.42 = 20 / 14 = (max angle) / (max voltage)
            # voltage: 28 -> max left, 14 -> middle, 0 -> max right
            coord = angle_in_degrees / 1.42
            coord = int(coord)
            # print(f"coord: {coord}")
            coord = -coord + 14
            if coord < 0:
                coord = 0
            elif coord >= 28:
                coord = 27
            speed = self.max_speed
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
            coord -= 1
            if coord == -1:
                coord = 0
            # print(f"speed: {speed}, angle: {angle}, tangent: {tangent}, angle: {angle_in_degrees}")
            # print(f"coordinate: {coord}")
            return speed, coord
        return None, None

    def send_signal_to_arduino(self, comm, speed, angle):
        if speed is None:
            return
        if self.stage == "RIGHT":
            angle = 0
        if self.stage == "LEFT":
            angle = 27
        # print(f"send arduino: {speed}, {angle}")

        data = struct.pack('<HHH', speed, speed, angle)
        comm.write(data)

        return

    def find_crosswalk(self, image):
        # self.valid_Y = 480
        # self.valid_X = 640
        # self.center_point = (470, 320)
        x_start = 600
        x_end = 1320
        y_start = 740
        y_end = 750
        total = (x_end - x_start) * (y_end - y_start)
        num = 0
        for x in range(x_start, x_end):
            for y in range(y_start, y_end):
                # self.result[y][x]=255
                if image[y][x] == 255:
                    num += 1
        percentage = num / total
        print(f"precentage of crosswalk: {percentage}")
        if percentage > 0.30:
            return True
        else:
            return False

    def find_obstacle(self):
        return False

    def find_car_lane(self, image, ans):
        if len(ans) == 0:
            return image
        # _, left = self.get_line_Canny_with_X_left(image, self.result, ans[0][0], self.center_point[1])
        # _, right = self.get_line_Canny_with_X_right(image, self.result, ans[0][0], self.center_point[1])
        # left = int((left[0]+left[1])/2)
        # right = int((right[0]+right[1])/2)
        X = ans[0][0]

        left_count = self.count_upper(image, X, self.left_lane)
        right_count = self.count_upper(image, X, self.right_lane)

        self.draw_dot(self.edges, X, self.left_lane)
        self.draw_dot(self.edges, X, self.right_lane)
        # print(f"left count: {left_count}, right count: {right_count}")

        if left_count > right_count:
            self.cur_lane = "left"
        else:
            self.cur_lane = "right"
        return image

    def change_car_lane(self, ans, lane):
        if len(ans) == 0:
            return ans
        # print(f"current lane: {self.cur_lane}, change it to {lane}")
        if lane == "left" and self.cur_lane == "right":
            Y = 2 * self.left_lane - ans[0][1]
            if Y <= 0:
                Y = 0
            ans = [(ans[0][0], Y)]
            # self.draw_dot(self.edges, ans[0][0], ans[0][1])
            # self.draw_dot(self.result, ans[0][0], ans[0][1])
            return ans
        if lane == "right" and self.cur_lane == "left":
            Y = 2 * self.right_lane - ans[0][1]
            if Y >= self.Y_length:
                Y = self.Y_length - 1
            ans = [(ans[0][0], Y)]
            # self.draw_dot(self.edges, ans[0][0], ans[0][1])
            # self.draw_dot(self.result, ans[0][0], ans[0][1])
            return ans
        return ans















