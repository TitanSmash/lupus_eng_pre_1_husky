#include <Arduino.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <sstream>
#include <display_code.h>
#include <Wire.h>
#include "Adafruit_GFX.h"
#include "OakOled.h"
#include "Adafruit_I2CDevice.h"
#include "SPI.h"
#include <OakOLED.h>
#include <display_code.h>


#define SCK 21 //Display Pins
#define SDA 22
std::string main_string = "";

OakOLED oled;

void display_serial(std::string toOutput) {
    Serial.print("calling from other file: ");
    Serial.print(toOutput.c_str());
}

void display_setup() {
    Serial.println("init display");
    // Wire.begin(22, 21);
    
    oled.begin();
    oled.setTextColor(1);
    oled.setTextSize(2);
    oled.clearDisplay();
    oled.println("Display On");
    oled.display();
}

void display_loop_par(void * parameter){

        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.println(main_string.c_str());
        oled.display();
    
    vTaskDelete(NULL);
}

void display_loop(std::string loopOutput){

    main_string = loopOutput;
    xTaskCreate(&display_loop_par, "display stuff", 10000, NULL, 3, NULL);
}





//TODO: Explore display capabilities
/* TODO: Create display OS:
* - idle rotation
* - Different eyes/expressions
* - Callable changes (task added, task done)
*
* - (if possible) Animations
*/

//hello
//hello
