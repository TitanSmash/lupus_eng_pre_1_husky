#include <Arduino.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <sstream>
#include <OakOLED.h>
#include <Wire.h>



void display_serial(std::string toOutput);
void display_test();
void display_setup();
void display_loop(std::string loopOutput);