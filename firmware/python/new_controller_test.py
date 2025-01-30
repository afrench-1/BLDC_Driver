from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time
import numpy as np

bus = make_can_bus()
motor = FxFDrive(bus, 10)
print("Started...")

# drive_state_num, drive_error_num = motor.telemetry.get_drive_state()
# print("Drive state", DriveState(drive_state_num).name)

motor.action.request_state_change(DriveState.drive_state_idle.value)

input("Enable control?")
motor.action.request_state_change(DriveState.drive_state_velocity_control.value)

motor.action.send_position_target(0)

motor.set_parameter_int(motor.parameters.PARAM_KP, 3_000, 3)
print(motor.get_parameter_int(motor.parameters.PARAM_KP))
# motor.set_parameter_int(motor.pa)
motor.set_parameter_int(motor.parameters.PARAM_KV, 90, 3)
print(motor.get_parameter_int(motor.parameters.PARAM_KV))
motor.set_parameter_int(motor.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, 15_000, 2)
print(motor.get_parameter_int(motor.parameters.PARAM_MAXIMUM_MOTOR_CURRENT))

input("Run")
motor.action.send_velocity_target(2)

try:
    while(True):
        print(motor.telemetry.get_velocity_radsps())
        time.sleep(0.03)

except KeyboardInterrupt:
    pass

input("Stop")
motor.action.send_velocity_target(0.0)

for i in range(5):
    input("move to first target")
    motor.action.send_position_target(int(2 * 3.1415 * 1000 * 2))

    input("move to second target")
    motor.action.send_position_target(0)


input("shutdown")
motor.action.request_state_change(DriveState.drive_state_idle.value)
bus.shutdown()