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

#define SCK 33 //Display Pins
#define SDA 32


OakOLED oled;

void display_serial(std::string toOutput) {
    Serial.print("calling from other file: ");
    Serial.print(toOutput.c_str());
}

void display_setup() {

    Wire.begin(SCK, SDA);

    oled.begin();
    oled.setTextColor(1);
    oled.setTextSize(3);
    oled.clearDisplay();
    oled.println("Display On");
    oled.display();
}

void display_loop(std::string loopOutput){

    for(int i=0; i<64; i++){
        oled.clearDisplay();
        oled.setCursor(i, 0);
        oled.println("var");
        oled.setCursor(64-i, 40);
        oled.println(loopOutput.c_str());
        oled.display();
        delay(50);
    }
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
