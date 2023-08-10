
from pynput.keyboard import Key, Listener

from le_mind_controller.MindData import MindData, HubPortName
from le_mind_controller.MindComm import MindComm
from le_mind_controller.Helpers import Helpers


def on_press(key):
    # print('{0} pressed'.format(key))
    match key:
        case Key.up:
            mc.motor_double_turn_on(HubPortName.A, HubPortName.B, 20, -20,distance_cm= 20)
        case Key.left:
            mc.motor_turn_on(HubPortName.A, 50)
        case Key.right:
            mc.motor_turn_on(HubPortName.A, -50)
        case Key.down:
            mc.motor_turn_on(HubPortName.B, 70)
        case Key.shift_l:
            mc.motor_turn_on(HubPortName.C, -35)
        case Key.shift_r:
            mc.motor_turn_on(HubPortName.C, 35)
        case Key.ctrl_l:
            mc.motor_turn_on(HubPortName.D, 35)
        case Key.ctrl_r:
            mc.motor_turn_on(HubPortName.D, -35)
        case Key.f12:
            print("Distance: {}".format(md.get_distance()[1]))
            print("Color: {}".format(md.get_color()[1].name))


def on_release(key):
    # print('{0} release'.format(key))
    match key:
        case Key.esc:
            print("Exiting...")  # Stop listener
            return False
        case _:
            mc.stop_program_execution()


print("Available COM ports:")
for prt in Helpers.get_available_ports():
    print(prt)
port = input("Selected COM port: ")
ser = Helpers.create_serial(port)

md = MindData()
mc = MindComm(ser, md)
with Listener(on_press=on_press, on_release=on_release) as listener: #ignore

    while not mc.data_received:
        pass

    mc.start_command_streaming()

    print("Number of devices connected to the hub: {}".format(md.determine_type_of_connected_devices()))
    mc.motor_double_turn_on(HubPortName.A, HubPortName.B, 20, -20, distance_cm=20)
    listener.join() #ignore
