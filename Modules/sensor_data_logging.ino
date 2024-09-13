/**
 * @file sensor_data_logging.ino
 * @brief Logs data from DHT22, MQ2, MQ7 sensors to SD card and displays it on OLED.
 * 
 * This program reads data from the DHT22 (temperature and humidity), MQ2, and MQ7 gas sensors.
 * It logs the data to an SD card and displays the information on an OLED screen.
 */

#include <SD.h>
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"

// Pins for SD module
const int CS_PIN = 5;    ///< Chip Select pin for SD module
const int MQ7_PIN = 34;  ///< Pin where MQ7 sensor is connected
const int MQ2_PIN = 14;  ///< Pin where MQ2 sensor is connected

// File to store data
File myFile;

// Function declarations
void initializeSD();
void openSDFile();
String floatToString(float value, int decimals);
void logData();

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1       ///< No reset pin for I2C
#define SCREEN_ADDRESS 0x3C ///< I2C address for the OLED screen
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C pins for OLED display
#define SDA_PIN 21
#define SCL_PIN 22

// Create DHT object
DHT dht(14, DHT22); ///< DHT object on pin 14 configured for DHT22 sensor

/**
 * @brief Arduino setup function. Initializes serial communication, sensors, OLED, and SD card.
 */
void setup() {
  Serial.begin(9600); ///< Initialize serial communication at 9600 baud rate

  // Initialize I2C for the OLED screen
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize DHT sensor
  dht.begin();

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Failed to initialize OLED screen"));
    for (;;); // Stop if initialization fails
  }

  // Initialize SD card
  initializeSD();
  openSDFile();
}

/**
 * @brief Arduino loop function. Logs and displays sensor data.
 */
void loop() {
  // Call function to capture and log data
  logData();
  delay(2000); ///< Wait 2 seconds before the next reading
}

/**
 * @brief Initializes the SD card.
 */
void initializeSD() {
  // Initialize the SD card
  if (!SD.begin(CS_PIN)) {
    Serial.println("Failed to initialize SD card");
    return;
  }
  Serial.println("SD card initialized successfully");
}

/**
 * @brief Opens or creates the file on the SD card where data will be stored.
 */
void openSDFile() {
  myFile = SD.open("dataLog.txt", FILE_WRITE);
  if (!myFile) {
    Serial.println("Failed to open file on SD card");
    return;
  }
  Serial.println("File opened successfully");
}

/**
 * @brief Logs sensor data to SD card and displays it on OLED.
 */
void logData() {
  // Open the file for appending data
  myFile = SD.open("/dataLog.txt", FILE_APPEND);
  if (myFile) {
    // Read MQ7 sensor value
    int mq7Value = analogRead(MQ7_PIN);
    float mq7Voltage = mq7Value * (3.3 / 4095.0); ///< Convert ADC value to voltage

    // Read MQ2 sensor value
    int mq2Value = analogRead(MQ2_PIN);
    float mq2Voltage = mq2Value * (3.3 / 4095.0); ///< Convert ADC value to voltage

    // Read temperature and humidity from DHT sensor
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Log data to SD card
    myFile.print("MQ7 Value: ");
    myFile.print(mq7Value);
    myFile.print("\tMQ2 Value: ");
    myFile.println(mq2Value);
    myFile.print("\tTemperature: ");
    myFile.println(temperature);
    myFile.print("\tHumidity: ");
    myFile.println(humidity);
    myFile.close(); // Close file after writing

    // Print values to Serial Monitor
    Serial.print("MQ7 Value: ");
    Serial.print(mq7Value);
    Serial.print("\tMQ7 Voltage: ");
    Serial.println(mq7Voltage);
    Serial.print("MQ2 Value: ");
    Serial.print(mq2Value);
    Serial.print("\tMQ2 Voltage: ");
    Serial.println(mq2Voltage);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("ºC ");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println("% ");
    Serial.println("Data logged to SD card successfully");

    // Update OLED display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Display data on OLED
    display.setCursor(0, 10);
    display.println("Data:");
    display.setCursor(0, 20);
    display.println("MQ7: " + String(mq7Value));
    display.setCursor(0, 30);
    display.println("MQ2: " + String(mq2Value));
    display.setCursor(0, 40);
    display.println("Temp: " + String(temperature) + "C");
    display.setCursor(0, 50);
    display.println("Hum: " + String(humidity) + "%");

    display.display(); // Update OLED screen
  } else {
    Serial.println("Error writing to file");
  }
}
