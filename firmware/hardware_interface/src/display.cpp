#include "display.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool initDisplay() {
    // Initialize I2C with default pins (SDA=21, SCL=22 on ESP32)
    Wire.begin(S_SDA, S_SCL);
    
    // Initialize display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        return false;
    }
    
    // Clear display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("LeRemix Starting...");
    display.display();
    
    return true;
}

void displayStatus(const char* message) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("LeRemix Status:");
    display.println();
    display.println(message);
    display.display();
}

void displayProgress(const char* message, int step, int total) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.println();
    display.printf("Step %d/%d:", step, total);
    display.println();
    display.println(message);
    
    // Progress bar
    int barWidth = 100;
    int barHeight = 8;
    int barX = 14;
    int barY = 50;
    
    display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
    int fillWidth = (step * (barWidth - 2)) / total;
    display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);
    
    display.display();
}

void displayError(const char* error) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ERROR!");
    display.println();
    display.println(error);
    display.display();
}

void displaySuccess(const char* message) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("SUCCESS!");
    display.println();
    display.println(message);
    display.display();
}

void clearDisplay() {
    display.clearDisplay();
    display.display();
}