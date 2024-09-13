/**
 * @file sd_card_initialization.ino
 * @brief Test for initializing the microSD card using SPI communication.
 * 
 * This program initializes the SPI communication and attempts to initialize the microSD card.
 * If the initialization is successful, it will print a message; otherwise, it will display an error.
 */

#include <SD.h>
#include <SPI.h>

// Pin definitions for the microSD module
#define chipSelect 5  ///< CS pin for SPI communication (micro SD)
#define SD_SCK_PIN 18 ///< Serial Clock (SCK) pin
#define SD_MOSI_PIN 23 ///< Master Out Slave In (MOSI) pin
#define SD_MISO_PIN 19 ///< Master In Slave Out (MISO) pin

/**
 * @brief Arduino setup function. Initializes the SPI communication and the microSD card.
 */
void setup() {
  Serial.begin(9600); ///< Initialize serial communication at 9600 baud rate
  SPI.begin(); ///< Initialize SPI communication

  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize the SD card");
    return; ///< Exit the function if initialization fails
  }
}

/**
 * @brief Arduino loop function. In this example, the loop does nothing.
 */
void loop() {
  // The loop does nothing in this example
}
