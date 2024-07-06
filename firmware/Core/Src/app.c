#include "app.h"

enum appState app_state = app_state_idle;

uint8_t run_LED_colors[3];

void app_setup(){
    // Start and calibrate analog 
    // (at beginning since calibration works best with )
    start_ADC();

    // Start timers
    start_led_timers();
    start_drive_timers();
    start_app_timers();

    // Start comms
    init_and_start_can();

    // Set encoder offset
    set_encoder_absolute_offset();

    run_LED_colors[0] = 254; // Red
    run_LED_colors[1] = 0; // Green
    run_LED_colors[2] = 254; // Blue

    // Attempt to init drive
    drive_state = drive_state_init;
}


int led_clock = 0;

void app_status_led_task(){
    led_clock += 1;

    switch(drive_state){
        case drive_state_error:
            led_rgb(1.0, 0, 0);
            osDelay(50);
            led_rgb(0, 0, 0);
            osDelay(150);
            break;
        case drive_state_disabled:
            led_hsv(182.0f, 1.0f, sin(led_clock/50.0f) * 0.2f + 0.4f);
            break;
    }

    osDelay(10);
}