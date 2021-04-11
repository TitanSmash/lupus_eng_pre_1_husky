#include <Arduino.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <sstream>



void display_serial(std::string toOutput);
void display_test();