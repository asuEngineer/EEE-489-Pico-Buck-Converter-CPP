#include <stdio.h>
#include <iostream>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

const uint PWM_PIN = 16;   //(GP16)
const uint ADC_PIN = 26;  // ADC0 (GP26)
const uint LED_PIN = 25;  // built in LED


int PWM_frequency = 200000;
float max_charge_battery = 15;
float desired_output_voltage = 1;
float duty_cycle = 0; //this just intitalizes the variable to 0, do not change



int find_wrap() {
    return ((125 * (pow(10,6)))/PWM_frequency) - 1;
}

void setup_pwm() {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint slice_num2 = pwm_gpio_to_slice_num(LED_PIN);
    pwm_set_wrap(slice_num, find_wrap());  // Set PWM frequency to 200kHz
    pwm_set_wrap(slice_num2, find_wrap());
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), (find_wrap()*.2));  // Initial duty cycle 20%
    pwm_set_chan_level(slice_num2, pwm_gpio_to_channel(LED_PIN), (find_wrap()*.2));  //sets the LED pin to be the same for visual feedback
    pwm_set_enabled(slice_num, true);
    pwm_set_enabled(slice_num2, true);
}

float read_voltage() {
    adc_select_input(0); //ADC0
    uint16_t result = adc_read();
    //feedback voltage needs to be in range from 0 to 3.3V or the pico will burn up and die
    float V_feedback = result * 3.3 / (1 << 12);  // Convert ADC value to voltage, there are 12 bits of resolution, (1<<12) represents value of 4096
    //need to scale the feedback voltage properly so that it knows what the output voltage is.
    //this function needs to output the voltage that the load sees
    return V_feedback;
}

void update_pwm_duty_cycle(float duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint16_t level = duty_cycle * find_wrap();  // Convert duty cycle to level
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_PIN), level);
}

// Function to map the input range (0 to max_charge_battery) to the output range (0 to 3.3) for PWM function
float mapInputToOutput(float input_voltage_from_feedback) {
    // Ensure the input is within the expected range
    if (input_voltage_from_feedback < 0) input_voltage_from_feedback = 0;
    if (input_voltage_from_feedback > max_charge_battery) input_voltage_from_feedback = max_charge_battery;

    // Calculate the scaling factor
    float scalingFactor = 3.3 / max_charge_battery;

    // Map the input value to the output range
    float output = input_voltage_from_feedback * scalingFactor;

    return output;
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);
   // int rc = pico_led_init();
   // hard_assert(rc == PICO_OK);

    setup_pwm();

    while (true) {

        float desired_PWM_voltage_from_feedback = mapInputToOutput(read_voltage());
        
        duty_cycle = desired_PWM_voltage_from_feedback / 3.3;  // Normalize to 0-1 range
        printf("duty cycle = %f\n", duty_cycle);

        if (duty_cycle > 1.0) duty_cycle = 1.0;  // Limit duty cycle to 100%
        if (duty_cycle < 0.0) duty_cycle = 0.0;  // Prevent negative duty cycle

        if (read_voltage() < desired_output_voltage) update_pwm_duty_cycle(duty_cycle + 0.01);
        if (read_voltage() > desired_output_voltage) update_pwm_duty_cycle(duty_cycle - 0.01);

        sleep_ms(10);  // Control loop delay
    }

    return 0;
}


