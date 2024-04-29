from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time
import numpy as np



import pygame
from pygame.locals import *

pygame.init()

joystick = pygame.joystick.Joystick(0)



bus = make_can_bus()
print("Bus running")

wheel1 = FxFDrive(bus, 10)
wheel2 =  FxFDrive(bus, 11)
time.sleep(0.1)

print("Setting parameters")
wheel1.set_parameter_int(wheel1.parameters.PARAM_CURRENT_LIMIT, 10_000, 2)
wheel2.set_parameter_int(wheel2.parameters.PARAM_CURRENT_LIMIT, 10_000, 2)

input("Idle drivers?")
wheel1.action.request_state_change(DriveState.drive_state_idle.value)
wheel2.action.request_state_change(DriveState.drive_state_idle.value)

input("Enable and move?")
wheel1.action.request_state_change(DriveState.drive_state_position_control.value)
wheel2.action.request_state_change(DriveState.drive_state_position_control.value)

wheel1.action.send_position_target(0)
wheel2.action.send_position_target(0)

def apply_deadzone(value, deadzone):
    if(abs(value) < deadzone):
        value = 0.0
    elif(value > deadzone):
        value = value - deadzone
    elif(value < deadzone):
            value = value + deadzone

    return value

try:
    target = 0
    t = 0
    while(True):
        for event in pygame.event.get(): # get the events (update the joystick)
            pass

        if joystick.get_button(1):
            print("stopped")
            break

        # speed = np.sin(t*0.01) * 50
        # print(target)

        # target += speed

        wheel1.action.send_position_target(apply_deadzone(joystick.get_axis(1), 0.15) * 30_000)
        wheel2.action.send_position_target(apply_deadzone(joystick.get_axis(1), 0.15) * 30_000)

        time.sleep(0.01)
        t +=1

except(KeyboardInterrupt):
    pass

wheel1.action.request_state_change(DriveState.drive_state_disabled.value)
wheel2.action.request_state_change(DriveState.drive_state_disabled.value)