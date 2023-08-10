import math
from le_mind_controller.MindData import MindData, HubPortName
from le_mind_controller.MindComm import MindComm
from le_mind_controller.Helpers import Helpers
import cv2
import numpy as np
import time

global first_orient
center_point = [0, 0]
size_from_pygame_chart = (416, 417)

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent / 100)
    height = int(frame.shape[0] * percent / 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

def color_detection(frame):
    global center_point #define a global value
    kernel = np.ones((2, 2), np.uint8)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    a_channel = lab[:, :, 1]

    ret, thresh = cv2.threshold(a_channel, 105, 255, cv2.THRESH_BINARY_INV)  # to binary
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)  # to get outer boundery only

    thresh = cv2.dilate(thresh, kernel, iterations=5)  # to strength week pixels
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        # find the biggest countour (c) by the area
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        x1 = x + w
        y1 = y + h
        # draw the biggest contour (c) in green
        cv2.rectangle(frame, (x, y), (x1, y1), (255, 0, 0), 2)
        center_point = [int((x + x1) / 2), int((y + y1) / 2)] #still updating the global value
        print(center_point)
        # coordinates of the rectangle

        #print("Center point funct: ", center_point)


def read_coordinates(file_name):
    with open(file_name) as file:
        coordinates = []
        for line in file:
            row = line.split()
            x = row[0]
            y = row[1]
            coordinates.append((float(x), float(y)))
    return coordinates

def convert(k, x):
    x = x - k
    if x > 180:
        x = x - 360
    if x < -180:
        x = 360 + x
    return x

def reverse_convert(x):
    if x < 0:
        return 360 + x
    else:
        return x

def update(old_direction, new_direction):
    result = 0
    if (old_direction - new_direction) >= 0:
        return old_direction - new_direction
    else:
        for i in range(new_direction):
            if old_direction - i <= 0:
                new_i = new_direction - i
                result = 360 - new_i
                break
        return result

def steer(left_wheel, right_wheel, goal_direction, temp_direction, steering_range, ksi, beta):

    cte = convert(goal_direction, temp_direction)
    if cte <= 0:
        if abs(cte) > steering_range:
            right_wheel = left_wheel*(-1)
        else:
            right_wheel = left_wheel + (ksi * cte)
    if cte > 0:
        if cte > steering_range:
            left_wheel = right_wheel*(-1)
        else:
            left_wheel = right_wheel - (ksi * cte)
    new_direction = goal_direction + beta * (right_wheel - left_wheel)
    return new_direction, left_wheel, right_wheel

print("Available COM ports:")
for prt in Helpers.get_available_ports():
    print(prt)

port = input("Selected COM port: ")
ser = Helpers.create_serial(port)
md = MindData()
mc = MindComm(ser, md)

while not mc.data_received:
    pass
mc.start_command_streaming()

print("Number of devices connected to the hub: {}".format(md.determine_type_of_connected_devices()))

ksi = 0.6
beta = 0.1
left = 50
right = 50
precision = 60

goal_ = read_coordinates('../old_paths/v1/goal_coords.txt')
start_ = read_coordinates('../old_paths/v1/start_coords.txt')
path_ = read_coordinates('../old_paths/v1/path_coords.txt')

x_start, y_start = zip(*start_)
x_goal, y_goal = zip(*goal_)

x_stop = x_goal[0]
y_stop = y_goal[0]
path_points_x, path_points_y = zip(*path_)

normalizing_ = read_coordinates('../old_paths/v1/normalizing_coord.txt')
x_min, y_min = zip(*normalizing_)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, size_from_pygame_chart[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size_from_pygame_chart[1])

first_orient = reverse_convert(md.tilt_angle[0])
while True:

    for i in range(0, len(path_)):
        _, frame = cap.read()
        frame75 = rescale_frame(frame, percent=75)
        color_detection(frame75)
        # print(first_orient)
        actual_x = center_point[0] - x_min[0]
        actual_y = center_point[1] - y_min[0]
        print("x : ", actual_x)
        print("y: ", actual_y)

        goal_from_path_direction = math.atan2(path_points_y[i] - actual_y, path_points_x[i] - actual_x) * 180 / 3.14

        print('Actual point: ', actual_x, actual_y, "Path point: ", path_points_x[i], path_points_y[i], "Direction: ",
              goal_from_path_direction)

        temp_direction = update(reverse_convert(md.tilt_angle[0]), first_orient)
        act_dir, left_speed, right_speed = steer(left, right, goal_from_path_direction, temp_direction, precision, ksi, beta)
        mc.motor_double_turn_on_deg(HubPortName.A, HubPortName.B, left_speed, right_speed, degrees=act_dir)
        time.sleep(0.1)

        print("First_orient:", first_orient)
        print("Actual_orient:", temp_direction)
        print("Counted direction: ", act_dir)
        # print("path point i : ", path_points_x[i],path_points_y[i],i)
        cv2.imshow("Frame75", frame75)
        if cv2.waitKey(1) & 0xFF == ord('d'):
            break
        if (path_points_x[i] - 5) < actual_x < (path_points_x[i] + 5) and (path_points_y[i] - 5) < actual_y < (
                path_points_y[i] + 5):
            print("#############################################################")
            i = i + 1
            mc.stop_program_execution()
            continue
            if i == len(path_)-1:
                mc.stop_program_execution()
                break

# 300.0 82.0
# 292.3918577108188 164.7303917873207
# 281.0584943215823 209.930190443873
# 248.38127942326042 256.45556414782874
# 216.25032815333682 285.05693406380817
# 201.62171689236357 305.16504130259415
# 187.88319756893998 329.7918992983735
# 168.26522466038097 348.4050458643151
# 149.0273802964277 363.3096629228987
# 141.25792010794154 375.7987247484982
# 141.0 378.0
