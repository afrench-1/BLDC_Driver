#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include "stdbool.h"
#include "math.h"

#define AMP_GAIN 40.0f
#define SHUNT_VALUE_R 0.002f
#define ADC_MIDPOINT 2048.0f
#define ADC_MAX 4096.0f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern int16_t adc1_dma[4]; // V sense, temp mot, temp fet, temp stm
extern uint16_t adc2_dma[3];

extern uint16_t phase_resistance_mOhm[3];

extern uint16_t v_motor_mv;

/**
 * @brief Current sense 0 offset
 * 
 */
extern uint32_t adc2_calib_offset[3];

/**
 * @brief Current sense in AMPs
 * Call update_current_sense to update
 */
extern float current_sense[3];
extern float alpha_current;
extern float beta_current;

extern int enc_angle_int;
extern uint16_t enc_angle_uint12;

/**
 * @brief Get dip switch value given number on bank
 * 
 * @param dip_num Number of dip switch on the bank
 * @return Dip switch value (true = on)
 * 
 * @note The last switch (4) is can termination
 */
bool get_dip(int dip_num);

/**
 * @brief Encoder interrupt.
 * 
 */
void encoder_ISR();

/**
 * @brief Set absolute encoder offset. 
 * Sets the absolute encoder offset by power cycling IC then reading out startup pulses
 */
void set_encoder_absolute_offset();

/**
 * @brief Calibrate ADCs and start DMA on both
 * 
 * @note ADC1 DMA is [supply motor divider, internal STM temp sensor, motor temp NTC divider, MOSFET temp NTC divider], ADC2 DMA is [current shunt A, current shunt C, current shunt B]
 * 
 */
void start_ADC();

/**
 * @brief Calibrate ADC given handle
 * 
 * @param hadc handle to ADC to calibrate
 */
void calibrate_ADC(ADC_HandleTypeDef *hadc);

/**
 * @brief Runs calibration on the DRV current shunt amps.
 * Starts by running DRV internal calibration, then sets an external offset
 * 
 */
void calibrate_DRV_amps();

/**
 * @brief Re-calculate current sense (in amps)
 * Uses floating-point so ideally shouldn't be used in the FOC loop
 * 
 */
void update_current_sense();

/**
 * @brief Get supply voltage
 * 
 * @return Voltage in volts 
 */
float get_vsupply();

/**
 * @brief Fast integer approximation to get supply voltage in 
 * 
 * @return Voltage in milliVolts 
 */
uint16_t get_vsupply_mv_fast();


bool get_dip(int dip_num);

/**
 * @brief Get MOSFET temperature
 * 
 * @return fet temp in C
 */
float get_fet_temp();

/**
 * @brief Get motor temperature
 * 
 * @return motor temp in C 
 * 
 * @note For now this assumes a 10k NTC
 */
float get_mot_temp();

#endif