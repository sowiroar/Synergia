/**
 * @file mz19_sensor.ino
 * @brief Reads CO2 and temperature values from the MHZ19 sensor and displays them on the serial monitor.
 * 
 * This program uses the MHZ19 sensor to measure CO2 concentration and temperature. 
 * The data is printed to the serial monitor every 2 seconds.
 */

#include <MHZ19.h>
#include <SoftwareSerial.h>

// RX pin of the Arduino connected to the TX pin of the MHZ19 sensor
#define RX_PIN 17 ///< RX pin for MHZ19
// TX pin of the Arduino connected to the RX pin of the MHZ19 sensor
#define TX_PIN 16 ///< TX pin for MHZ19

// MHZ19 sensor object
MHZ19 myMHZ19; 
// Serial connection required by the MHZ19 sensor
SoftwareSerial mySerial(RX_PIN, TX_PIN); 

// Timer for measurement intervals
unsigned long timer = 0;

/**
 * @brief Arduino setup function. Initializes serial communication and the MHZ19 sensor.
 * 
 * It also enables auto-calibration for the MHZ19 sensor.
 */
void setup() {
  Serial.begin(9600); ///< Initialize serial communication at 9600 baud rate
  mySerial.begin(9600); ///< Initialize the SoftwareSerial for the MHZ19 sensor
  myMHZ19.begin(mySerial); ///< Start communication with the MHZ19 sensor

  // Enable auto calibration (use myMHZ19.autoCalibration(false) to disable)
  myMHZ19.autoCalibration(); 
}

/**
 * @brief Arduino loop function. Takes CO2 and temperature readings every 2 seconds.
 * 
 * This function reads CO2 levels and temperature from the MHZ19 sensor and prints the 
 * values to the serial monitor.
 */
void loop() {
  // Take measurements every 2 seconds
  if (millis() - timer >= 2000) {
    
    // Get the current CO2 level in ppm
    int co2Level = myMHZ19.getCO2();

    // Display the CO2 level on the serial monitor
    Serial.print("CO2 (ppm): ");                      
    Serial.println(co2Level);  

    // Get the current temperature in Celsius
    int8_t temperature = myMHZ19.getTemperature();

    // Display the temperature on the serial monitor
    Serial.print("Temperature (C): ");                  
    Serial.println(temperature);  

    // Store the current time to control the elapsed time
    timer = millis();
  }
}
