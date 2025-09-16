#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define S_SCL 22
#define S_SDA 21

// Display object
extern Adafruit_SSD1306 display;

// Display functions
bool initDisplay();
void displayStatus(const char* message);
void displayProgress(const char* message, int step, int total);
void displayError(const char* error);
void displaySuccess(const char* message);
void clearDisplay();

#endif