#include "foc.h"

#define SQRT_1_2 0.7071067812f
#define SQRT_2_3 0.8164965809f
#define SQRT_3 1.7320508076f


// Control mode toggles
bool foc_active = false;
bool position_control_enabled = false;
bool anti_cogging_enabled = false;

// Current Variables
int max_motor_current_mAmps = 1000;

int position_setpoint_filtered = 0;
int current_setpoint_limit_mA = 3000;
int current_Q_setpoint_mA = 0;
int current_D_setpoint_mA = 0;

int16_t current_A_mA = 0;
int16_t current_B_mA = 0;
int16_t current_C_mA = 0;

int16_t current_A_offset_mA = 0;
int16_t current_C_offset_mA = -51;

int16_t current_A_mA_filtered = 0;
int16_t current_B_mA_filtered = 0;
int16_t current_C_mA_filtered = 0;

int16_t current_Alpha_mA = 0;
int16_t current_Beta_mA = 0;

int16_t current_D_mA = 0;
int16_t current_Q_mA = 0;


// Voltage Variables
int8_t maximum_duty = 200;

int16_t voltage_D_mV = 0;
int16_t voltage_Q_mV = 0;

int16_t voltage_Alpha_mV = 0;
int16_t voltage_Beta_mV = 0;

int16_t voltage_a_mV = 0;
int16_t voltage_b_mV = 0;
int16_t voltage_c_mV = 0;

int direction = 1;
int hysteresis_offset_value = 4;
int hysteresis_offset = 0;
int prev_hysteresis_offset = 0;

// Gains
// TODO: Choose this based on motor resistance and inductance
float current_P_gain = 0.04f;

// Angle
int prev_encoder_position = 0;
int encoder_velocity = 0;
uint8_t electrical_angle = 0;
uint8_t electrical_angle_offset = 0;
int8_t electrical_mechanical_ratio = 21;

int position_setpoint = 0;

void set_current_setpoints(int D_setpoint_mA, int Q_setpoint_mA){
    current_Q_setpoint_mA = Q_setpoint_mA;
    current_D_setpoint_mA = D_setpoint_mA;
}


float q_hat = 0;
float q_err = 0;
float v_hat = 0;
const float loop_rate = 10000.0f; 
volatile const float dt = 1.0/loop_rate;

int adjusted_enc_angle = 0;

bool first_run = true;

void foc_interrupt(){
    // Calculate electrical angle as a uint8
    // 0 is aligned with phase A
    // 255 is just before wraparound

    if( (enc_angle_int - prev_encoder_position) > 0 && direction < 0 ){
        hysteresis_offset = -hysteresis_offset_value;
        direction = 1;
    } else if( (enc_angle_int - prev_encoder_position) < 0 && direction > 0 ){
        hysteresis_offset = hysteresis_offset_value;
        direction = -1;
    } else {
        // hysteresis_offset = 0;
    }

    adjusted_enc_angle = enc_angle_int + hysteresis_offset;
    electrical_angle = convert_to_electrical_angle(adjusted_enc_angle, electrical_mechanical_ratio, electrical_angle_offset);

    // Calculate velocity

    // "warm up" observer
    if(first_run == true){
        q_hat = (float) adjusted_enc_angle;
        prev_encoder_position = adjusted_enc_angle;
        first_run = false;
    }

    // Estimate new position
    q_hat += dt * v_hat;

    // If encoder has changed, update observer
    // if(prev_encoder_position != adjusted_enc_angle){
    q_err = adjusted_enc_angle - q_hat;
    v_hat += 10.0f * q_err - 0.1 * v_hat;
    // }

    encoder_velocity = round(v_hat);
    prev_encoder_position = adjusted_enc_angle;

    // Calculate motor currents and transform them
    // 9 is a magic number
    // TODO: Document 9
    current_A_mA =  (adc2_dma[0] - adc2_calib_offset[0]) * 9;
    current_C_mA =  (adc2_dma[1] - adc2_calib_offset[1]) * 9;
    current_B_mA =  - (current_A_mA + current_C_mA);

    // Perform an IIR filter on current to cut down on noise and injected vibrations
    // current_A_mA_filtered = current_A_mA;
    // current_B_mA_filtered = current_B_mA;
    // current_C_mA_filtered = current_C_mA;

    current_A_mA_filtered = current_A_mA_filtered * 0.05f + current_A_mA * 0.95f;
    current_B_mA_filtered = current_B_mA_filtered * 0.05f + current_B_mA * 0.95f;
    current_C_mA_filtered = current_C_mA_filtered * 0.05f + current_C_mA * 0.95f;

    // Perform clarke and park transform to get motor current in orthogonal rotor-centric coordinates
    clarke_transform(current_A_mA_filtered, current_B_mA_filtered, current_C_mA_filtered, &current_Alpha_mA, &current_Beta_mA);
    park_transform(current_Alpha_mA, current_Beta_mA, electrical_angle, &current_D_mA, &current_Q_mA);

    // Generate current setpoint 
    // TODO: Split this out into another timer interrupt

    if(drive_state == drive_state_position_control || drive_state == drive_state_anti_cogging_calibration){
        // position_setpoint_filtered = 0.99f * position_setpoint_filtered + position_setpoint * 0.01f;
        // position_setpoint_filtered = position_setpoint;
        // current_Q_setpoint_mA = 0.0f * current_Q_setpoint_mA + ((enc_angle_int - position_setpoint_filtered) * 100.0f) * 1.0f;
        // int vel_setpoint = 500;

        // OLD 10 vel, 1.5 position
        // int vel_setpoint = 10.0f * (position_setpoint - enc_angle_int - hysteresis_offset);
        // current_Q_setpoint_mA = -20.0f * (encoder_velocity-vel_setpoint) - 0.0f * vel_setpoint;
        // current_Q_setpoint_mA = -3.0f * encoder_velocity;
        // current_Q_setpoint_mA = 300.0f * (position_setpoint_filtered - (enc_angle_int + hysteresis_offset)) - 15.0f * encoder_velocity;
        current_Q_setpoint_mA = - 10.0f * (encoder_velocity - 1000);
        // current_Q_setpoint_mA = 5000; 

    } else {
        current_Q_setpoint_mA = 0;
    }

     // 1A current setpoint for testing

    // torque_setpoint = (position_setpoint - enc_angle_int) * 1.0f;
    // current_Q_setpoint_mA = bound( (int16_t) (position_setpoint - enc_angle_int) * 1.0f, -current_setpoint_limit_mA, current_setpoint_limit_mA);
    // current_Q_setpoint_mA = -1000;
    // Enforce limits on current
    current_Q_setpoint_mA = bound(current_Q_setpoint_mA, -current_setpoint_limit_mA, current_setpoint_limit_mA);

    // Feedforward for anti-cogging
    if(anti_cogging_enabled){
        current_Q_setpoint_mA += current_offsets[electrical_angle] * 1.0f;
    }

    float current_gain = 0.002f;
    // Q current P loop
    // TODO: Add an integral term?
    voltage_Q_mV = (current_Q_mA - current_Q_setpoint_mA) * current_gain - current_Q_setpoint_mA * 0.002f ;
    // voltage_Q_mV = -current_Q_setpoint_mA * 0.001f;
    // voltage_Q_mV = 20;
    voltage_Q_mV = (int16_t) bound(voltage_Q_mV, -250, 250);
    // D current P loop
    // voltage_D_mV = -(current_D_mA - current_D_setpoint_mA) * current_gain;
    voltage_D_mV = 0;
    voltage_D_mV = (int16_t) bound(voltage_D_mV, -250, 250);

    // Perform inverse park and clarke transform to convert from rotor-centric voltage into phase voltages
    inverse_park_transform(voltage_D_mV, voltage_Q_mV, electrical_angle, &voltage_Alpha_mV, &voltage_Beta_mV);
    inverse_clarke_transform(voltage_Alpha_mV, voltage_Beta_mV, &voltage_a_mV, &voltage_b_mV, &voltage_c_mV);
    // Find minimum voltage to offset phase voltages to be all positive
    // Might be worth experimenting with centering phases around V_supply/2 to avoid this
    // there might be additional consequences

    int16_t min_voltage_mV = int16_min3(voltage_a_mV, voltage_b_mV, voltage_c_mV);


    // If FOC is enabled, set voltages
    if(foc_active){
        // TODO, convert to setting actual voltages instead of duty cycle
        set_duty_phases(voltage_a_mV - min_voltage_mV, voltage_b_mV - min_voltage_mV, voltage_c_mV - min_voltage_mV);
    }
}

void clarke_transform(int16_t A, int16_t B, int16_t C, int16_t *alpha, int16_t *beta){
    *alpha = (int16_t) ( SQRT_2_3 * (1 * A  -  B / 2           -  C / 2         ) );
    *beta =  (int16_t) ( SQRT_2_3 * (0 * A  +  SQRT_3 * B / 2  -  SQRT_3 * C / 2) );
}

void park_transform(int16_t alpha, int16_t beta, uint8_t angle, int16_t *d, int16_t *q){
    *d = mult_sin_lut_int16(angle + 64, alpha) + mult_sin_lut_int16(angle, beta);
    *q = mult_sin_lut_int16(angle + 64, beta)  - mult_sin_lut_int16(angle, alpha);
}

void inverse_clarke_transform(int16_t alpha, int16_t beta, int16_t *a, int16_t *b, int16_t *c){
    *a = SQRT_2_3 * ( 1 * alpha );
    *b = SQRT_2_3 * ( - alpha / 2.0f + SQRT_3 * beta / 2.0f);
    *c = SQRT_2_3 * ( - alpha / 2.0f - SQRT_3 * beta / 2.0f);
}

void inverse_park_transform(int16_t d, int16_t q, uint8_t angle, int16_t *alpha, int16_t *beta){
    *alpha = mult_sin_lut_int16(angle + 64, d) + mult_sin_lut_int16(angle, q);
    *beta =  mult_sin_lut_int16(angle + 64, q) - mult_sin_lut_int16(angle, d);
}




/////////// LEGACY ///////////
// void apply_duty_at_electrical_angle_int(uint8_t angle, uint8_t magnitude){
//     angle = (angle + (1.0 / 4.0) * 256);

//     uint8_t A_value = mult_sin_lut_uint8((uint8_t) (angle + (0.0 / 3.0) * 256), magnitude);
//     uint8_t B_value = mult_sin_lut_uint8((uint8_t) (angle + (1.0 / 3.0) * 256), magnitude);
//     uint8_t C_value = mult_sin_lut_uint8((uint8_t) (angle + (2.0 / 3.0) * 256), magnitude);

//     set_duty_phase_A(A_value);
//     set_duty_phase_B(B_value);
//     set_duty_phase_C(C_value);
// }

// void spin_electrical_rads_sin(float revs, int calibration_voltage){
//     for(int i = 0; i<=255; i++){
//         // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
//         apply_voltage_at_electrical_angle_int((uint8_t) (int) (i * revs) % 254, calibration_voltage);
//         osDelay(5);
//     }
// }

// void spin_electrical_rev_forward(int calibration_voltage){
//     for(int i = 0; i<=255; i++){
//         // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
//         apply_voltage_at_electrical_angle_int(i, calibration_voltage);
//         osDelay(5);
//     }
// }


// void spin_electrical_rev_backward(int calibration_voltage){
//     for(int i = 0; i<=255; i++){
//         // apply_voltage_at_electrical_angle((i / 100.0f) * rads, calibration_voltage);
//         apply_voltage_at_electrical_angle_int(255 - i, calibration_voltage);
//         osDelay(5);
//     }
// }