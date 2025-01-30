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

time.sleep(0.1)

print("Setting parameters")
wheel1.set_parameter_int(wheel1.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, 1_000, 2)


input("Idle drivers?")
wheel1.action.request_state_change(DriveState.drive_state_idle.value)


input("Enable and move?")
wheel1.action.request_state_change(DriveState.drive_state_position_control.value)

wheel1.action.send_position_target(0)

wheel1.action.send_position_target(500)

wheel1.action.request_state_change(DriveState.drive_state_disabled.value)
