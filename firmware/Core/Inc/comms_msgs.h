#define DEVICE_TYPE_ID 50

////////////////////////////////// DIAGNOSTIC (0, 19)

// MESSAGE Device UID
// LAYOUT [type, uid, uid, uid, uid]
// UNIT [device type, unique chip id]
#define DIAGNOSTIC_UID_ID 5

#define DIAGNOSTIC_HARD_RESET_ID 10

//////////// ACTION [20 - 99]

#define ACTION_REQUEST_STATE_CHANGE 20 // [new state]

#define ACTION_TORQUE_SETPOINT 50 // [torque torque torque] Nm / 1000
#define ACTION_POSITION_SETPOINT 51 // [pos pos pos] rads / 1000 # signed
#define ACTION_VELOCITY_SETPOINT 52 // [vel vel vel] rads / 1000


//////////// TELEMETRY [100 - 149]
#define TELEM_DRIVE_STATE 100 // [drive state, drive error]

#define TELEM_MOTOR_VOLTAGE 101
#define TELEM_MOTOR_CURRENT 102
#define TELEM_MOTOR_TORQUE 103
#define TELEM_MOTOR_POSITION 104
#define TELEM_MOTOR_VELOCITY 105

#define TELEM_FET_TEMP 110
#define TELEM_MOTOR_TEMP 111

//////////// PARAMETERS 150 - 255

// --- FOC PARAMETERS
#define PARAM_ENCODER_OFFSET 150// [offset]
#define PARAM_RATIO 151 // [ratio] SIGNED

#define PARAM_CURRENT_P_GAIN 155 // [p_gain p_gain p_gain p_gain] gain / 1000
#define PARAM_CURRENT_I_GAIN 156 // [i_gain i_gain i_gain i_gain] gain / 1000

#define PARAM_ANTI_COGGING 160
#define PARAM_ANTI_COGGING_TABLE 161

#define PARAM_MAXIMUM_MOTOR_VOLTAGE 170 // [V V] mv
#define PARAM_MAXIMUM_MOTOR_CURRENT 171 // [A A] mA

// --- Motor parameters
#define PARAM_PHASE_RESISTANCE 180 // [resistance, resistance] mOhms
#define PARAM_MOTOR_KV 181 // [kv kv kv] rpm/V / 1000
#define PARAM_MOTOR_KT 182 // [kt kt kt] Nm / 1000

// --- Control parameters
#define PARAM_KP 191 // [kp kp] Nm / rad error / 1000
#define PARAM_KPV 192 // [kpv kpv] Nm / radps / 1000 
#define PARAM_KV 193 // [kv kv] Nm / radps error / 1000

#define PARAM_P_MAX 194 // [p_max p_max p_max p_max] rads/1000 SIGNED
#define PARAM_P_MIN 195 // [p_max p_max p_max p_max] rads/1000 SIGNED

#define PARAM_V_MAX 196 // [v_max v_max v_max v_max] radsps/1000 SIGNED



////////////////////////////////// SYSTEM (msgs sent on ID 0)
#define SYS_STOP_ALL_ID 0
#define SYS_HARD_RESET_ALL_ID 1

#define SYS_LIST_ALL_ID 5

#define SYS_LOCATE_ID 11 // Blinks LEDs to locate a device based on it's UID

#define SYS_SET_CAN_ID_ID 12