import math
from le_mind_controller.MindData import MindData, HubPortName
from le_mind_controller.MindComm import MindComm
from le_mind_controller.Helpers import Helpers

def convert(k,x):
    x = x - k
    if x > 180:
        x = x - 360
    if x < -180:
        x = 360 + x
    return x

def steer(left_wheel, right_wheel, kierunek_cel, temp_direction, steering_range, ksi, beta):

    cte = convert(kierunek_cel, temp_direction)
    print('cte=', cte)
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
    print('left_wheel=', left_wheel, 'right_wheel=', right_wheel)
    nowy_kierunek = kierunek_cel + beta * (right_wheel - left_wheel)
    return nowy_kierunek, left_wheel, right_wheel

ksi = 0.6
beta = 0.2
left = 30
right = 30
precision = 50

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



import time
while True:
    temp_direction = md.tilt_angle[0]
    #kierunek_cel = math.atan2(path_points_y[i] - actual_x, path_points_x[i] - actual_y) * 180 / 3.14
    kierunek_cel = 120
    act_dir, left_speed, right_speed = steer(left, right, kierunek_cel, temp_direction, precision, ksi, beta)

    print("Tilt angle: ", temp_direction)
    print("Goal direction: ", kierunek_cel)
    print("New direction: ", act_dir)

    mc.motor_double_turn_on_deg(HubPortName.A, HubPortName.B, left_speed, right_speed, degrees=act_dir)
    print("left= ", left_speed, "right = ", right_speed)
    time.sleep(0.05)  # waiting

    # function for moving forward with given distance
    #mc.motor_double_turn_on(HubPortName.A, HubPortName.B, abs(k2), abs(k1), distance_cm=15)
    #time.sleep(5)  # waiting

