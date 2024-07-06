#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"
#include "sensors.h"
#include "math.h"

#include "trig_luts.h"

#include "stdbool.h"

#include "cmsis_os.h"

//////////// DEFINES
#define MIN_MOTOR_RESISTANCE_OHM 0.1f
#define MAX_MOTOR_RESISTANCE_OHM 10.0f

#define MIN_SUPPLY_VOLTAGE_V 10

//////////// ENUMS
enum DriveError {
    drive_error_none,
    drive_error_high_resistance,
    drive_error_low_resistance,
    drive_error_low_voltage
};

extern enum DriveError drive_error;

// == Startup
// On startup, the drive immediately tries to init itself
// If no-voltage, or other faults are present, it will fall back into an error state

// == Calibration
// From disabled, it can be pushed through the calibration flow (either one at a time or full)

// == Running Control Modes
// From disabled, the drive can be pushed into idle, and then again into one of the control modes
// Theoretically the drive can be pushed straight from disabled into a control mode, but the
// extra step potentially avoids some dumb mistakes on the software side

// == Disabling
// The drive can either be returned to idle, or all the way to disabled again
// Generally idle is if you intend to keep motor power on
// Note: the drive will fall into an error state (due to low voltage) if motor power is removed
// And you will have to do init-(auto)>disabled-(manual)>idle-(manual)>control
// TODO: Potentially init automagically when voltage returns?

// The main drive state enum values
enum DriveState {
    // "Off" modes
    drive_state_error, // Error is present, check drive_error
    drive_state_init, // Init DRV chip, state runs on startup and moves to drive_state_disabled
    drive_state_disabled, // All phases disabled, but logic active with no faults

    // Calibration modes
    drive_state_resistance_estimation,
    drive_state_encoder_calibration,
    drive_state_full_calibration, // minus anti-cogging ? 
    drive_state_anti_cogging_calibration,

    // Control Modes
    drive_state_idle, // FOC current control at 0, drive must be in idle before other control modes are run
    drive_state_torque_control,
    drive_state_velocity_control,
    drive_state_position_control
};

extern enum DriveState drive_state;

// If true, FOC is allowed to control motor phases
extern bool foc_active;
// If true FOC runs with anti-cogging table torque offsets
extern bool anti_cogging_enabled;


//////////// VARIABLES

/**
 * @brief Maximum motor current in milliAmps that the FOC loop is allowed to apply
 * 
 * @warning This is not an error-generating limit, and only applies to the FOC loop. 
 *          If other parts of the program command more current (e.g. through a voltage based PWM function)
 *          they are not be limited through this.
 * 
 */
extern int max_motor_current_mAmps;

/**
 * @brief Estimated phase resistance in milliOhms
 * 
 * @see estimate_phase_resistance
 * 
 */
extern int estimated_resistance_mOhms;

extern uint8_t electrical_angle_offset;
extern int8_t electrical_mechanical_ratio;

extern int current_Q_setpoint_mA;

extern int position_setpoint;

//////////// EXTERNS
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;


//////////// FUNCTIONS
void start_drive_timers();

// Enable DRV chip by setting ENABLE pin
void enable_DRV();
// Disable DRV chip by setting ENABLE pin
void disable_DRV();


// Main drive state machine
void drive_state_machine();

/**
 * @brief Enable FOC loop, allowing it to apply phase voltages
 * 
 */
void enable_foc_loop();

/**
 * @brief Disable FOC loop from applying phase voltages 
 * and zero out phases.
 * 
 */
void disable_foc_loop();

/**
 * @brief Estimate phase resistance by applying
 * a known voltage and measuring resulting phase current.
 * 
 * Throws a fault if any resistance is out of line with expected.
 * (0.1 to 10 ohms)
 * 
 * @param voltage voltage (in volts) to apply during estimation
 * 
 * @note
 * This will disable FOC during the duration of the test as it requires
 * direct access to phases voltages.
 */
void estimate_phase_resistance(float voltage);

/**
 * @brief Estimate encoder mechanical offset by rotating using D current
 * 
 * @param voltage voltage (in volts) to apply during estimation
 */
void calibrate_encoder(float voltage);

uint8_t convert_to_electrical_angle(int mechanical_angle, int ratio, int offset);

enum DriveError check_supply_voltage();

/**
 * @brief Apply PWM duty cycle at a specific electrical angle,
 * 
 * @param angle Electrical angle as a uint8 (0 = 0, 256 = 2 * pi)
 * @param magnitude PWM magnitude as a uint8 (out of 256)
 */
void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude);

/**
 * @brief Set phase A PWM duty cycle
 * 
 * @param value PWM magnitude as a uint8 (out of 256)
 */
void set_duty_phase_A(uint8_t value);

/**
 * @brief Set phase B PWM duty cycle
 * 
 * @param value PWM magnitude as a uint8 (out of 256)
 */
void set_duty_phase_B(uint8_t value);

/**
 * @brief Set phase C PWM duty cycle
 * 
 * @param value PWM magnitude as a uint8 (out of 256)
 */
void set_duty_phase_C(uint8_t value);

/**
 * @brief Set all duty PWM phases together
 * 
 * @param A_value PWM magnitude as a uint8 (out of 256)
 * @param B_value PWM magnitude as a uint8 (out of 256)
 * @param C_value PWM magnitude as a uint8 (out of 256)
 */
void set_duty_phases(uint8_t A_value, uint8_t B_value, uint8_t C_value);

#endif