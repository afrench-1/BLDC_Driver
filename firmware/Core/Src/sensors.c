#include "sensors.h"
#include "app_timers.h"

int16_t adc1_dma[4];
uint16_t adc2_dma[3];
uint32_t adc2_calib_offset[3];

float current_sense[3];
float alpha_current;
float beta_current;

uint16_t phase_resistance[3]; // Phase resistance in milliohms

uint16_t v_supply_mv; // Motor input voltage in millivolts

int enc_angle_int;
uint16_t enc_angle_uint12;

bool start_up_pulses = false;
uint16_t start_up_pulse_count = 0;

bool get_dip(int dip_num){
  if(dip_num == 1){
    return !HAL_GPIO_ReadPin(DIP_1_GPIO_Port, DIP_1_Pin);
  }else if(dip_num == 2){
    return !HAL_GPIO_ReadPin(DIP_2_GPIO_Port, DIP_2_Pin);
  }else if(dip_num == 3){
    return !HAL_GPIO_ReadPin(DIP_3_GPIO_Port, DIP_3_Pin);
  }
  return false;
}

void encoder_ISR(){
  // TODO: Why
  if(HAL_GPIO_ReadPin(IFB_GPIO_Port, IFB_Pin) == HAL_GPIO_ReadPin(IFA_GPIO_Port, IFA_Pin)){
    enc_angle_int ++;
  }else{
    enc_angle_int --;
  }
}


void set_encoder_absolute_offset(){
  // TODO: This is very very hacky and I do not like it

  // GET PULSE COUNT
  // Disable magnetic encoder
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, 1);
  app_delay_ms(50);

  // Reset angle
  enc_angle_int = 0;
  // Enable magnetic encoder
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, 0);
  app_delay_ms(15);
  // }
  printf("lmao");

}

void calibrate_ADC(ADC_HandleTypeDef *hadc){
  if(HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK){
    Error_Handler();
  }
}

void start_ADC(){

  // Calibrate and start adc1 DMA (vsupply + temp)
  calibrate_ADC(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adc1_dma, 4);

  // Calibrate and start adc2 DMA (phase current shunts)
  calibrate_ADC(&hadc2);
  HAL_ADC_Start_DMA(&hadc2, adc2_dma, 3);
}

float get_vsupply(){
  // float R1 = 100.0f; // kOhms
  // float R2 = 6.8f; // KOhms
  // float adc_constant = 3.3f / 4096.0f * 1.00f;
  // float adc_v = adc1_dma[0] * adc_constant;
  // // volatile float voltage_divider_const = 
  // v_supply_mv = (adc_v * ((R1 + R2) / R2));
  volatile float v_supply = (adc1_dma[0]*0.000806f / 0.0637f)*1.0f;
  return v_supply;
}

uint16_t get_vsupply_mv_fast(){
  return adc1_dma[0] * 13;
}

void calibrate_DRV_amps(){
    // Perform DRV amp calibration
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, 0);
    HAL_Delay(2);
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, 1);
    // Wait for internal calibration
    HAL_Delay(10);

    // Perform external calibration
    // TODO: Have average over a set of samples
    // adc2_calib_offset[0] = 0;
    // adc2_calib_offset[1] = 0;
    // adc2_calib_offset[2] = 0;

    // Disable calibration
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, 0);

    int num_samples = 200;
    for(int i = 0; i < num_samples; i++){
      adc2_calib_offset[0] += adc2_dma[0];
      adc2_calib_offset[1] += adc2_dma[1];
      adc2_calib_offset[2] += adc2_dma[0];
      HAL_Delay(1);
    }

    adc2_calib_offset[0] /= num_samples;
    adc2_calib_offset[1] /= num_samples;
    adc2_calib_offset[2] /= num_samples;

    printf("lol");
}

void update_current_sense(){
  float filter_constant = 0.005f;
  // TODO: Set up proper phase numbering
  // current_sense[0] = (current_sense[0]) * (1 - filter_constant) + (((((int)adc2_dma[0] - (int)adc2_calib_offset[0]) / ADC_MAX) * 3.3f) / (AMP_GAIN * SHUNT_VALUE_R)) * filter_constant;
  // current_sense[1] = (current_sense[1]) * (1 - filter_constant) + (((((int)adc2_dma[1] - (int)adc2_calib_offset[1]) / ADC_MAX) * 3.3f) / (AMP_GAIN * SHUNT_VALUE_R)) * filter_constant;
  // // current_sense[2] = -current_sense[0] - current_sense[1];
  // current_sense[2] = (current_sense[2]) * (1 - filter_constant) + (((((int)adc2_dma[2] - (int)adc2_calib_offset[2]) / ADC_MAX) * 3.3f) / (AMP_GAIN * SHUNT_VALUE_R)) * filter_constant;

  current_sense[0] = (current_sense[0]) * (1 - filter_constant) + (((((int)(adc2_calib_offset[0]) - (int)(adc2_dma[0])) / ADC_MAX) * 3.3f*1.00f) / (AMP_GAIN * SHUNT_VALUE_R)) * filter_constant;
  current_sense[1] = (current_sense[1]) * (1 - filter_constant) + (((((int)(adc2_calib_offset[1]) - (int)(adc2_dma[1])) / ADC_MAX) * 3.3f*1.00f) / (AMP_GAIN * SHUNT_VALUE_R)) * filter_constant;
  current_sense[2] = -current_sense[0] - current_sense[1];
  // current_sense[2] = (current_sense[2]) * (1 - filter_constant) + (((((int)adc2_calib_offset[2] - (int)adc2_dma[2]) / ADC_MAX) * 3.3f*1.00f / (AMP_GAIN))) * filter_constant;


  // alpha_current = sqrtf(2.0f/3.0f) * (1.0f * current_sense[0] - 0.5f *          current_sense[1] - 0.5f *          current_sense[2]);
  // beta_current =  sqrtf(2.0f/3.0f) * (0.0f * current_sense[0] + sqrtf(3.0f)/2.0f * current_sense[1] - sqrtf(3.0f)/2.0f * current_sense[2]);
  // current_sense[0] = (current_sense[0]) * (1.0f - filter_constant) + adc2_dma[0] * filter_constant;
  // current_sense[1] = (current_sense[1]) * (1.0f - filter_constant) + adc2_dma[1] * filter_constant;
  // current_sense[2] = (current_sense[2]) * (1.0f - filter_constant) + adc2_dma[2] * filter_constant;
}

#define BETA_FET 4050.0f
// TODO: Allow for manual calibration of this, this is an approximation
#define BETA_MOT 4000.0f 
#define ROOM_TEMP 298.15f
#define ADC_MAX 4096.0f
#define R_1 10.0f
#define FET_R_ROOM_TEMP 47.0f
#define MOT_R_ROOM_TEMP 10.0f
#define KELVIN_TO_C 273.15f

// TODO: Allow for manual calibration of this
#define ADC_ADJUSTMENT_FACTOR_FET 150
#define ADC_ADJUSTMENT_FACTOR_MOT 100

float get_fet_temp(){
  // Calculate ntc resistance given the other parameters in the voltage divider
  float r_ntc = R_1 / ((ADC_MAX / ((float) adc1_dma[3] + ADC_ADJUSTMENT_FACTOR_FET)) - 1);
  
  float fet_temp_kelvin = (BETA_FET * ROOM_TEMP) /
                          (BETA_FET + (ROOM_TEMP * logf(r_ntc / FET_R_ROOM_TEMP)));

  // Convert from kelvin to C
  volatile float fet_temp_C = fet_temp_kelvin - KELVIN_TO_C;
  
  return fet_temp_C;
}

float get_mot_temp(){
  // Calculate ntc resistance given the other parameters in the voltage divider
  float r_ntc = R_1 / ((ADC_MAX / ((float) adc1_dma[2] + ADC_ADJUSTMENT_FACTOR_MOT)) - 1);
  
  float mot_temp_kelvin = (BETA_MOT * ROOM_TEMP) /
                          (BETA_MOT + (ROOM_TEMP * logf(r_ntc / MOT_R_ROOM_TEMP)));

  // Convert from kelvin to c
  volatile float mot_temp_C = mot_temp_kelvin - KELVIN_TO_C;
  
  return mot_temp_C;
}