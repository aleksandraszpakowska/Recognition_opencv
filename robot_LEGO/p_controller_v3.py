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
        print("center point from function ", center_point)
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
    # if -20 <= new_direction <= 0:
    #     right_wheel = right_wheel + 10
    # if 0 <= new_direction <= 20:
    #     left_wheel = left_wheel + 10
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

ksi = 0.6 #0.6
beta = 0.3 #0.3
left = 50
right = 50
precision = 70

goal_ = read_coordinates('goal_coords.txt')
start_ = read_coordinates('start_coords.txt')
# path_ = read_coordinates('basic_path.txt')
path_ = read_coordinates('path_coords.txt')

x_start, y_start = zip(*start_)
x_goal, y_goal = zip(*goal_)

x_stop = x_goal[0]
y_stop = y_goal[0]
path_points_x, path_points_y = zip(*path_)

normalizing_ = read_coordinates('normalizing_coord.txt')
x_min, y_min = zip(*normalizing_)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, size_from_pygame_chart[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size_from_pygame_chart[1])

first_orient = reverse_convert(md.tilt_angle[0])
counter = 0
while True:

    for i in path_:
        _, frame = cap.read()
        #frame75 = rescale_frame(frame, percent=75)
        color_detection(frame)
        actual_x = center_point[0] - x_min[0]
        actual_y = center_point[1] - y_min[0]

        while ((i[0] - 7) <= actual_x or actual_x <= (i[0] + 7)) and ((i[1] - 7) <= actual_y or actual_y <= (i[1] + 7)):
            # #print("*********************************************************************")
            _, frame = cap.read()
            #frame75 = rescale_frame(frame, percent=75)
            color_detection(frame)

            cv2.imshow('AR capture', frame)
            if cv2.waitKey(1) & 0xFF == ord('d'):
                break

            actual_x = center_point[0] - x_min[0]
            actual_y = center_point[1] - y_min[0]

            goal_from_path_direction = math.atan2(i[1] - actual_y, i[0] - actual_x) * 180 / 3.14

            temp_direction = update(reverse_convert(md.tilt_angle[0]), first_orient)
            act_dir, left_speed, right_speed = steer(left, right, goal_from_path_direction, temp_direction, precision, ksi, beta)
            print("act dir: ", act_dir)
            # if -10 <= act_dir <= 10:
            #     print("****************************************")
            #     counter = counter + 1
            #     break
            print(left_speed,right_speed)
            mc.motor_double_turn_on_deg(HubPortName.A, HubPortName.B, left_speed, right_speed, degrees=act_dir)
            # if act_dir < 15:
            #     print("*******************************")
            #     mc.motor_double_turn_on(HubPortName.A, HubPortName.B, abs(left_speed), abs(right_speed), distance_cm=10)
            #     time.sleep(0.01)
            #     mc.motor_double_turn_on(HubPortName.A, HubPortName.B, abs(left_speed), abs(right_speed), distance_cm=1)
            #     time.sleep(0.01)
            time.sleep(0.01)
            print(i, actual_x, actual_y)
            if ((i[0] - 5) <= actual_x <= (i[0] +5)) and ((i[1] - 5) <= actual_y <= (i[1] + 5)):
                counter = counter+1
                break

    if len(path_) == counter:
        mc.stop_program_execution()
        break
