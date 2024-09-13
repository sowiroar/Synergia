/**
 * @file synergia_oled_display.ino
 * @brief This code is an initial test to display data on an OLED screen.
 * 
 * This program initializes the I2C communication and the OLED display,
 * then shows sample data on the screen. The data displayed includes sensor
 * readings such as gas concentration, temperature, and humidity.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128    ///< OLED display width in pixels
#define SCREEN_HEIGHT 64    ///< OLED display height in pixels

#define OLED_RESET -1       ///< No reset pin required for I2C
#define SCREEN_ADDRESS 0x3C ///< I2C address of the OLED display (commonly 0x3C)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C Pins
#define SDA_PIN 21 ///< I2C SDA pin
#define SCL_PIN 22 ///< I2C SCL pin

/**
 * @brief Arduino setup function. Initializes the I2C communication and the OLED display.
 */
void setup() {
  // Initialize I2C with the defined pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Failed to initialize the OLED display"));
    for (;;); // Halt if initialization fails
  }

  // Clear the display and configure text settings
  display.clearDisplay();
  display.setTextSize(1);           ///< Set text size
  display.setTextColor(SSD1306_WHITE); ///< Set text color to white

  // Display the data
  display.setCursor(0, 0);  ///< Set cursor to (0,0)
  display.println("Captured data"); ///< Display title
  display.setCursor(0, 20); ///< Set cursor for gas data
  display.println("MQ7:225"); ///< Display gas sensor reading
  display.setCursor(65, 20); ///< Set cursor for temperature data
  display.println("TEMP:37"); ///< Display temperature reading
  display.setCursor(0, 40); ///< Set cursor for humidity data
  display.println("HUM:60"); ///< Display humidity reading
  display.display(); ///< Show the content on the display
  delay(2000);
}

/**
 * @brief Arduino loop function. It runs repeatedly, but no action is needed for this test.
 */
void loop() {
  // No actions are needed in the loop for this test
}
