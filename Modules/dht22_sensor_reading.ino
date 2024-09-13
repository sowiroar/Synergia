/**
 * @file dht22_sensor_reading.ino
 * @brief Reads humidity and temperature values using the DHT22 sensor.
 * 
 * This program reads the temperature and humidity data from a DHT22 sensor
 * and prints the values to the serial monitor.
 */

#include <Adafruit_Sensor.h>
#include "DHT.h"

// Create DHT object
DHT dht(14, DHT22); ///< DHT object connected to pin 14 and configured for DHT22 sensor

/**
 * @brief Arduino setup function. Initializes serial communication and the DHT22 sensor.
 */
void setup()
{
  Serial.begin(9600); ///< Initialize serial communication at 9600 baud rate
  dht.begin();        ///< Initialize the DHT22 sensor
}

/**
 * @brief Arduino loop function. Reads and prints the temperature and humidity values.
 */
void loop()
{
  // Read humidity and temperature
  float humi = dht.readHumidity();    ///< Read humidity
  float temp = dht.readTemperature(); ///< Read temperature
  
  // Print temperature and humidity to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("ºC ");
  Serial.print("Humidity: ");
  Serial.println(humi);

  delay(1000); ///< Wait 1 second before the next reading
}
