#include <Arduino.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <sstream>
#include <display_code.h>

void display_serial(std::string toOutput) {
    Serial.print("calling from other file: ");
    Serial.print(toOutput.c_str());
}

//TODO: Explore display capabilities
/* TODO: Create display OS:
* - idle rotation
* - Different eyes/expressions
* - Callable changes (task added, task done)
*
* - (if possible) Animations
*/

