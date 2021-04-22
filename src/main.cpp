#include <Arduino.h>
//#include <stdint.h>
#include <string.h>
//#include <stdbool.h>
//#include <stdio.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include <string>
#include "esp_system.h"
//#include <cctype>
#include <sstream>
#include <display_code.h>



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
int mot_time;
int mot_speed = 100;

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
 *  BT commands
 * 
 *  Task related stuff (TODO)
 * 
 *  [add_tsk]"content"      =   String with task (TODO)
 *  [read_tsk]"content"     =   Outputs task at "content", if empty: list all
 *  [length_tsk]            =   Outputs length of task array
 *  [del_tsk]"content"      =   Deletes task at "content", if empty: delete latest
 *  [fin_tsk]"content"      =   Finished task at "content", if empty: finish task at [0]
 *  
 *  
 *  Motor controls
 *  
 *  [mot_1_f]"content"      =   Turn motor 1 forward for a time in ms
 *  [mot_1_b]"content"      =   Turn motor 1 backward for a time in ms
 *  [mot_2_f]"content"      =   Turn motor 2 forward for a time in ms
 *  [mot_2_b]"content"      =   Turn motor 2 backward for a time in ms
 *  
 *  [mot_1_f]1000
 * 
 **************************/

// motor control commands
void motor1_f (void * parameter){
    Serial.print("running motor 1 f...  ");
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70);
    vTaskDelay(mot_time / portTICK_PERIOD_MS);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    Serial.println("  ...motor stopped");
    vTaskDelete(NULL);
}

void motor1_b (void * parameter){
    Serial.print("running motor 1 b...  ");
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 60);
    vTaskDelay(mot_time / portTICK_PERIOD_MS);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    Serial.println("  ...motor stopped");
    vTaskDelete(NULL);
}

void motor2_f (void * parameter){
    Serial.print("running motor 2 f...  ");
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 100);
    vTaskDelay(mot_time / portTICK_PERIOD_MS);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    Serial.println("  ...motor stopped");
    vTaskDelete(NULL);
}

void motor2_b (void * parameter){
    Serial.print("running motor 2 b...  ");
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, 100);
    vTaskDelay(mot_time / portTICK_PERIOD_MS);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
    Serial.println("  ...motor stopped");
    vTaskDelete(NULL);
}


// show status func
void show_status (void * parameter){
    Serial.print("Task is running on core: ");
    Serial.println(xPortGetCoreID());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}

//create new vtask
/*
*   void your_task(void * parameter){
*       vTaskDelay(100 / portTICK_PERIOD_MS);
*       vTaskDelete(NULL);
*   }
*   std::string content <- String after cmd
*   
*
*/





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
    // read data from buffer, add to message variable and decode it
    if (SerialBT.available()){
        data_in = SerialBT.read();
        
        if (data_in != '\r'){
            message.push_back(data_in);
        } else if (data_in == '\r'){
            action = decoder(message);  // decoder function
            data_done = true;           // without this, unfinished data would be looked at
            SerialBT.read();            // clear buffer
        }
    }

    /*  Main command processor 
    *   
    *   To read and call a new function, add a else if as below
    * 
    * 
    *   _______________________________________________________________________________
    *   
    *   else if(action == "[cmd]" && data_done) {
    *       message = "\0";
    *       action = "\0";
    *       xTaskCreate(&command_func, "command desc", 10000, NULL, 1, NULL);
    *       data_done = false;
    *   }
    *   ________________________________________________________________________________  
    * 
    *   In the function, you can read the variable content for the data string. Modifying 
    *   Datatypes has to be done within the function itself.
    * 
    */
    
    
    if (action == "[sub]" && data_done){
        message = "\0";
        action = "\0";

        xTaskCreate(&show_status, "show_tasks", 10000, NULL, 1, NULL);

        data_done = false;
    } 
    else if (action == "[txt]" && data_done) {
        message = "\0";
        action = "\0";

        Serial.print("BT Text sent: ");
        Serial.println(content.c_str()); // because we are using std::string (cpp sting), we need to transform it to a c-string with .c_str()

        data_done = false;
    } 
    else if (action == "[mot_1_f]" && data_done){
        message = "\0";
        action = "\0";
        std::istringstream(content) >> mot_time;
        Serial.println(mot_time);
        xTaskCreate(&motor1_f, "run motor1 forwards", 10000, NULL, 1, NULL);
        data_done = false;
    }
    else if (action == "[mot_1_b]" && data_done){
        message = "\0";
        action = "\0";
        
        std::istringstream(content) >> mot_time;
        Serial.println(message.c_str());
        xTaskCreate(&motor1_b, "run motor1 backwards", 1000, NULL, 1, NULL);
        data_done = false;
    }
    else if (action == "[mot_2_f]" && data_done){
        message = "\0";
        action = "\0";
        
        std::istringstream(content) >> mot_time;
        Serial.println(message.c_str());
        xTaskCreate(&motor2_f, "run motor2 forwards", 1000, NULL, 1, NULL);
        data_done = false;
    }
    else if (action == "[mot_2_b]" && data_done){
        message = "\0";
        action = "\0";
        
        std::istringstream(content) >> mot_time;
        Serial.println(message.c_str());
        xTaskCreate(&motor2_b, "run motor2 backwards", 1000, NULL, 1, NULL);
        data_done = false;
    }

    else if (action == "[set_speed]" && data_done){
        message = "\0";
        action = "\0";
        
        std::istringstream(content) >> mot_speed;
        Serial.println(message.c_str());
        SerialBT.print("Motor speed is: " );
        SerialBT.println(mot_speed);
        xTaskCreate(&motor2_b, "run motor2 backwards", 1000, NULL, 1, NULL);
        data_done = false;
    }

    else if (data_done) {
        message = "\0";
        action = "\0";
        Serial.print("action not programmed: ");
        display_serial(content);
        Serial.println("clearing message");
        data_done = false;
    }
}

