from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time
import pygame
from pygame.locals import *

pygame.init()

# Initialize CAN bus and motor driver
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


# Apply a deadzone to the input
def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        value = 0.0
    elif value > deadzone:
        value = value - deadzone
    elif value < -deadzone:
        value = value + deadzone
    return value


try:
    target = 0
    t = 0
    pygame.mouse.set_visible(False)  # Hide the mouse cursor
    screen = pygame.display.set_mode((800, 600))  # Create a dummy window for mouse capture
    pygame.event.set_grab(True)  # Lock the mouse within the window

    while True:
        for event in pygame.event.get():  # Process events
            if event.type == QUIT:
                raise KeyboardInterrupt  # Quit when the window is closed

        # Check for mouse button click to exit
        mouse_buttons = pygame.mouse.get_pressed()
        if mouse_buttons[2]:  # Right mouse button to stop
            print("stopped")
            break

        # Get mouse movement
        mouse_dx, mouse_dy = pygame.mouse.get_rel()  # Get relative mouse movement

        # Apply deadzone and scaling
        mouse_dy = apply_deadzone(mouse_dy, 5)  # Deadzone for vertical movement
        target += mouse_dy * 100  # Scale mouse movement to position

        # Send position target to the motor
        wheel1.action.send_position_target(target)
        print(target)

        time.sleep(0.01)
        t += 1

except (KeyboardInterrupt):
    pass

# Disable the motor safely on exit
wheel1.action.request_state_change(DriveState.drive_state_disabled.value)
