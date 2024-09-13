/**
 * @file mq7_sensor_reading.ino
 * @brief Reads the analog value from the MQ7 sensor and converts it to voltage.
 * 
 * This program reads the analog value from the MQ7 gas sensor and calculates the corresponding voltage
 * based on a 12-bit resolution. The results are printed on the serial monitor.
 */

#include <Arduino.h>

// Define the pin where the MQ7 sensor is connected
#define MQ7_PIN 34 ///< GPIO pin connected to the MQ7 sensor

/**
 * @brief Arduino setup function. Initializes serial communication and ADC resolution.
 */
void setup() {
  Serial.begin(9600);  ///< Initialize serial communication at 9600 baud rate
  analogReadResolution(12); ///< Set ADC resolution to 12 bits (4096 levels)
}

/**
 * @brief Arduino loop function. Reads the MQ7 sensor value and calculates the voltage.
 */
void loop() {
  int sensorValue = analogRead(MQ7_PIN); ///< Read the sensor value from GPIO34
  float voltage = sensorValue * (3.3 / 4095.0); ///< Convert the sensor value to voltage
  
  // Print the sensor value and voltage to the serial monitor
  Serial.print("Sensor value: ");
  Serial.print(sensorValue);
  Serial.print(" - Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  
  delay(1000); ///< Wait 1 second before the next reading
}
