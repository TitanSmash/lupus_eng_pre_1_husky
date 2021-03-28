#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include "esp_system.h"
#include <cctype>


BluetoothSerial SerialBT;

#define LED_STATUS 27 // red, status
#define LED_POWER  14 // green, power


#define MODE 4 
#define MOTOR_1_PIN_A 16
#define MOTOR_1_PIN_B 17
#define MOTOR_2_PIN_A 18
#define MOTOR_2_PIN_B 19
#define ADC_1 ADC2_CHANNEL_8 // 25
#define ADC_2 ADC2_CHANNEL_9 // 26


bool blinkBatteryWarning;


typedef enum {
	APP_MOTOR_1 = 0,
	APP_MOTOR_2,
    APP_MAX
} APP_COMMANDS;


static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle) {
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle) {
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num) {
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void configure_motors() {
    gpio_set_direction((gpio_num_t) MODE, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) MODE, 0); // USE LOW for In/In Mode, see Datasheet

    // Motor 1
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_1_PIN_A); // esp32 has two MCPWM units each with 3 dual PWM channels
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_1_PIN_B);

    // Motor 2
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_2_PIN_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_2_PIN_B);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000; // Hz
    pwm_config.cmpr_a = 0.f;      // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0.f;      // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); // Configure PWM0A & PWM0B with above settings
}

/**************************
 * BT signals/breaks
 * 
 * 
 * General transmission
 * 
 * [/st] : Start DataString
 * [/end]: End Data
 * 
 * 
 * Task & Data uploading
 * 
 * [/tsk] "task string": String with task
 * [/tsk_done]: task completed
 * 
 * 
 * Motor control
 * 
 * [/mot]: Motor
 * [/spd]: Motor Speed
 * 
 * 
 **************************/


void motor1_test (void * parameter){
    Serial.print("running motor...  ");
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    Serial.println("  ...motor stopped");
    vTaskDelete(NULL);
}

void show_status (void * parameter){
    Serial.print("Task is running on core: ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}



void battery_status (void * parameter) {
    while(true) {
        vTaskDelay(50000 / portTICK_PERIOD_MS); // run every 10 seconds

        uint32_t reading = analogReadMilliVolts(35);
        Serial.print("battery voltage: ");
        Serial.println(reading);
        if(reading < 340)
            blinkBatteryWarning = true;
        else if(reading < 350)
            gpio_set_level((gpio_num_t) LED_STATUS, 1); // indicate battery empty soon
        else
            blinkBatteryWarning = false;
    }

    
}


void setup()
{
    Serial.begin(115200);
    configure_motors();

    gpio_pad_select_gpio(LED_STATUS);
    gpio_set_direction((gpio_num_t) LED_STATUS, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_POWER);
    gpio_set_direction((gpio_num_t) LED_POWER, GPIO_MODE_OUTPUT);

    

    if(!SerialBT.begin("board 54")){
        Serial.println("An error occurred initializing Bluetooth");
    }
    Serial.println("please connect to device");

    xTaskCreate(&battery_status, "battery", 1000, NULL, 1, NULL);
    
    
}

char data_in;
bool data_done = false;
std::string message = "";
std::string content= "";
std::string action = ""; 


std::string decoder(std::string msg){
    std::string signal = "";
    content = "";
    int i = 0;

    Serial.println("called decoder func");
    for (; msg[0] == ' '; ){
        Serial.println("deleting space");
        msg.erase(0,1);
    }
    if (msg[0] == '['){
        for (; msg[i] != ']'; i++){
            signal.push_back(msg[i]);
        }
        signal.push_back(']');
        Serial.println(signal.c_str());
    }
    for (i++ ; i < msg.length(); i++){
        content.push_back(msg[i]);
    }
    return signal;
}




void loop()
{   
    
    if (SerialBT.available()){
        data_in = SerialBT.read();
        
        if (data_in != '\r'){
            message.push_back(data_in);
        } else if (data_in == '\r'){
            action = decoder(message);
            data_done = true;
            SerialBT.read();
        }
    }

    
    if (action == "[mot]" && data_done){
        message = "\0";
        action = "\0";
        Serial.println(message.c_str());
        xTaskCreate(&motor1_test, "run motor1 for 1 sec", 1000, NULL, 1, NULL);
        data_done = false;
    } else if (action == "[sub]" && data_done){
        message = "\0";
        action = "\0";
        xTaskCreate(&show_status, "show_tasks", 1000, NULL, 1, NULL);
        data_done = false;
    } else if (action == "[txt]" && data_done) {
        message = "\0";
        action = "\0";

        Serial.print("BT Text sent: ");
        Serial.println(content.c_str());

        data_done = false;
    } else if (data_done) {
        message = "\0";
        action = "\0";
        Serial.print("action not programmed: ");
        Serial.println(action.c_str());
        Serial.println("clearing message");
        data_done = false;
    }
}

