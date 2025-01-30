from FxFDrive import FxFDrive, make_can_bus, list_available_drives, bytes_to_int, int_to_bytes
from FxFDrive import DriveError, DriveState
import time
import numpy as np

# Initialize the CAN bus and motor
bus = make_can_bus()
motor = FxFDrive(bus, 10)
print("Started...")

# Set motor to idle state initially
motor.action.request_state_change(DriveState.drive_state_idle.value)

# Enable control
input("Enable control?")
motor.action.request_state_change(DriveState.drive_state_velocity_control.value)

# Initialize position and parameters
motor.action.send_position_target(0)
motor.set_parameter_int(motor.parameters.PARAM_KP, 0.1, 3)
print("KP:", motor.get_parameter_int(motor.parameters.PARAM_KP))
motor.set_parameter_int(motor.parameters.PARAM_KV, 360, 3)
print("KV:", motor.get_parameter_int(motor.parameters.PARAM_KV))
motor.set_parameter_int(motor.parameters.PARAM_MAXIMUM_MOTOR_CURRENT, 50_000, 2)
print("Maximum Current:", motor.get_parameter_int(motor.parameters.PARAM_MAXIMUM_MOTOR_CURRENT))

print("Maximum Motor Voltage:", motor.get_parameter_int(motor.parameters.anti_cogging), "mV")


# Start varying speed
input("Run motor with varying speed?")

try:
    t = 0  # Time counter
    while True:
        # Generate a sinusoidal velocity target
        velocity_target = 5.0 * np.sin(t)  # Amplitude of 2 rad/s, adjust as needed
        motor.action.send_velocity_target(velocity_target)
        
        # Print current telemetry for debugging
        current_velocity = motor.telemetry.get_velocity_radsps()
        print(f"Target: {velocity_target:.2f}, Current: {current_velocity:.2f}")
        
        # Increment time counter
        t += 0.02  # Increment time step (adjust for smoother or faster oscillations)
        time.sleep(0.06)

except KeyboardInterrupt:
    # Stop motor on keyboard interrupt
    motor.action.send_velocity_target(0)
    print("Motor stopped.")
