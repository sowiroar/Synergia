/**
 * @file pms5003_sensor.ino
 * @brief Reads dust concentration data from the PMS5003 sensor using ESP32 and prints it to the serial monitor.
 * 
 * This program reads PM1.0, PM2.5, and PM10 dust concentrations from the PMS5003 sensor 
 * and displays the values on the serial monitor. The sensor is connected via hardware UART (Serial2) on the ESP32.
 */

#include "PMS.h"  ///< Library for PMS sensors

// Define the pins for the PMS5003 sensor connection
#define RX_PIN 16  ///< RX pin of ESP32 connected to TX of PMS5003
#define TX_PIN 17  ///< TX pin of ESP32 connected to RX of PMS5003

// Initialize the Serial2 port for PMS5003 communication
PMS pms(Serial2);  ///< PMS5003 object using Serial2
PMS::DATA data;    ///< Data object to store the sensor readings

/**
 * @brief Arduino setup function. Initializes serial communication and PMS5003 sensor.
 * 
 * This function sets up the Serial communication and the Serial2 port for the PMS5003 sensor.
 * It waits 4 seconds for the sensor to warm up before starting data collection.
 */
void setup() 
{ 
  Serial.begin(9600);  ///< Initialize serial communication at 9600 baud
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  ///< Initialize Serial2 at 9600 baud with specified pins for PMS5003
  Serial.println("Warming up");
  delay(4000);  ///< Wait 4 seconds to allow the PMS5003 sensor to stabilize
}

/**
 * @brief Arduino loop function. Reads and prints dust concentration data every second.
 * 
 * This function checks if the PMS5003 sensor has valid data, reads the PM1.0, PM2.5, and PM10
 * values, and prints them to the serial monitor. The loop runs continuously every 1 second.
 */
void loop() 
{
  if (pms.read(data))  ///< Read data from the PMS5003 sensor
  {
    // Print the dust concentration data to the serial monitor
    Serial.println("Dust Concentration");
    Serial.println("PM1.0: " + String(data.PM_AE_UG_1_0) + " (ug/m3)");
    Serial.println("PM2.5: " + String(data.PM_AE_UG_2_5) + " (ug/m3)");
    Serial.println("PM10: " + String(data.PM_AE_UG_10_0) + " (ug/m3)");
    
    delay(1000);  ///< Wait 1 second before the next reading
  }
}
