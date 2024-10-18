#include <Arduino.h>

//OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//SD
#include <SD.h>  // Librería para manejar la tarjeta SD
#include <FS.h>  // Librería para manejar el sistema de archivos

//TEMPERATURA Y HUMEDAD DHT22
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//PSM
#include "PMS.h"
#include <SoftwareSerial.h> 

//MHZ19b
#include "MHZ19.h"
#include <SoftwareSerial.h>  

//Conexión MQTT
#include <WiFi.h>
#include <PubSubClient.h>

//----------------------------------Pantalla oled-----------------------------------------------

/**
 * @brief Definición de dimensiones de la pantalla OLED.
 * 
 * SCREEN_WIDTH: Ancho del display OLED en píxeles.
 * SCREEN_HEIGHT: Altura del display OLED en píxeles.
 */
#define SCREEN_WIDTH 128 ///< OLED display width, in pixels
#define SCREEN_HEIGHT 64 ///< OLED display height, in pixels

/**
 * @brief Configuración del pin de reset de la pantalla OLED.
 * 
 * OLED_RESET: Pin de reset (usar -1 si se comparte el pin de reset de Arduino).
 * SCREEN_ADDRESS: Dirección I2C del OLED (0x3D para pantallas de 128x64, 0x3C para 128x32).
 */
#define OLED_RESET     -1 ///< Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< Dirección I2C de la pantalla OLED (0x3D para 128x64, 0x3C para 128x32)

/**
 * @brief Inicialización del objeto display para la pantalla OLED.
 * 
 * @param SCREEN_WIDTH Ancho de la pantalla.
 * @param SCREEN_HEIGHT Altura de la pantalla.
 * @param &Wire Objeto Wire para la comunicación I2C.
 * @param OLED_RESET Pin de reset de la pantalla OLED.
 */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/**
 * @brief Array estático para almacenar una imagen de logotipo en formato de matriz de bits.
 * 
 * Se utiliza PROGMEM para almacenar la imagen en la memoria flash en lugar de la RAM.
 */
static const unsigned char PROGMEM logo[] =
{
// 'imagen_2024-09-21_230453788', 128x64px
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0xfc, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xfb, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0x31, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0x31, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0x11, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0x3f, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0x89, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0x98, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xdc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xee, 0x7f, 0x9c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe4, 0xff, 0x8c, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe4, 0x7f, 0x96, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0x3e, 0x16, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0x80, 0x76, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xc3, 0x07, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x73, 0xff, 0xff, 0x9c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf3, 0xff, 0xff, 0x9e, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xe7, 0xff, 0xff, 0x9f, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0xe7, 0xff, 0xff, 0x9f, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xef, 0xff, 0xff, 0x9f, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0x7f, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0x7f, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0xff, 0xff, 0x7f, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xff, 0x7f, 0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xff, 0x7f, 0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xfe, 0x7f, 0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xfe, 0x7f, 0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xfe, 0x7f, 0xff, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xfc, 0xff, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xfc, 0xff, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xff, 0xf8, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0xff, 0xf1, 0xff, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xe1, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x7f, 0xc0, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x0c, 0x3f, 0xfe, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0x03, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

//-------------------------------------SD----------------------------------------------------------
/**
 * @brief Configuración SD
 * Se encarga de definir e iniciar la tarjeta SD para guardar los datos de los sensores
 */

#define chipSelect 5  // CS pin for SPI comunication (micro SD)
#define SD_SCK_PIN 18    // Pin Serial Clock (SCK)
#define SD_MOSI_PIN 23   // Pin Master Out Slave In (MOSI)
#define SD_MISO_PIN 19   // Pin Master In Slave Out (MISO)
File myFile;


//--------------------------------- Sensor temperatura y humedad DHT11----------------------------
/**
 * @brief Configuración inicial para la pantalla OLED.
 * Se encarga de definir el objeto para el sensor e inicializar las variables para guardar los datos del sensor
 */
DHT dht(14, DHT22);
float hum = dht.readHumidity();
float tem = dht.readTemperature();

//------------------------------------Sensor de **** MQ-135---------------------------------------
/**
 * @brief Configuración inicial para sensor MQ-135
 * Se encarga de definir  inicializar la variable para guardar los datos del sensor a
 */
#define MQ135_PIN 34
int sensorValue;

//------------------------------------Sensor ***PMS--------------------------------------------------
/**
 * @brief Configuración inicial para sensor PMS.
 * Define los pines para la conexión UART del sensor diferente a los demás porque seriales porque estan ocupados
 * e inicia el objeto para el sensor
 */
// Define the pins for the PMS5003 sensor connection
#define RX_PIN_P 12  ///< RX pin of ESP32 connected to TX of PMS5003
#define TX_PIN_P 13  ///< TX pin of ESP32 connected to RX of PMS5003
SoftwareSerial pmsSerial(RX_PIN_P , TX_PIN_P);  ///< Objeto SoftwareSerial para PMS5003
// Initialize the Serial2 port for PMS5003 communication
PMS pms(pmsSerial);  ///< PMS5003 object using Serial2
PMS::DATA data;    ///< Data object to store the sensor readings

//Definción de variables para almacenar datos
int pm1;
int pm2;
int pm10;

//--------------------------------------MHZ19B
/**
 * @brief Configuración inicial para sensor MHZ19B.
 * Define los pines para la conexión UART en el serial2 
 * e inicia el objeto para el sensor y las variables correspondientes
 */
// RX pin of the Arduino connected to the TX pin of the MHZ19 sensor
#define RX_PIN 17 ///< RX pin for MHZ19
// TX pin of the Arduino connected to the RX pin of the MHZ19 sensor
#define TX_PIN 16 ///< TX pin for MHZ19
MHZ19 myMHZ19; 
// Serial connection required by the MHZ19 sensor
SoftwareSerial mySerial(RX_PIN, TX_PIN); 
int co2Level;
int8_t temperature;

//-----------------------------CONFIGURACIÓN DE RED--------------------------------------------------

const char* ssid = "ORBIN";
const char* password = "1053834012";
const char* mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

//-----------------------------PROTOTIPO DE FUNCIONES--------------------------------------------------
/**
 * @brief Definición de los prototipos de funciones creadas por los desarrolladores
 */
void obtener_tem_hum();
void MQ135();
void SensMHZ19();
void abrirArchivoSD();
void cargarDatos();
void PMS_b();
void errorSD();
void SDbien();

// Función para conectar a WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Función callback para recibir mensajes
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Enciende el LED
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Apaga el LED
  }
}

// Función para reconectar a MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish("outTopic", "ESP32 connected");
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Enviar datos de los sensores de forma independiente a través de MQTT
void sendSensorData() {
  // Humedad
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", hum);
  client.publish("humidity", msg);
  Serial.print("Humedad (%): ");
  Serial.println(hum);
  
  // Temperatura DHT22
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", tem);
  client.publish("temperature", msg);
  Serial.print("Temperatura DHT22 (°C): ");
  Serial.println(tem);

  // Temperatura MH-Z19B
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", temperature);
  client.publish("temperature2", msg);
  Serial.print("Temperatura MH-Z19B (°C): ");
  Serial.println(temperature);
  
  // Nivel de CO2
  snprintf(msg, MSG_BUFFER_SIZE, "%.d", co2Level);
  client.publish("co2", msg);
  Serial.print("Nivel de CO2 (ppm): ");
  Serial.println(co2Level);
  
  // Valor del Sensor MQ135
  snprintf(msg, MSG_BUFFER_SIZE, "%.d", sensorValue);
  client.publish("mq135", msg);
  Serial.print("Sensor MQ135 (Digital): ");
  Serial.println(sensorValue);
  
  // Partículas PM1.0
  snprintf(msg, MSG_BUFFER_SIZE, "%.d", pm1);
  client.publish("pm1", msg);
  Serial.print("PM1.0 (µg/m³): ");
  Serial.println(pm1);
  
  // Partículas PM2.5
  snprintf(msg, MSG_BUFFER_SIZE, "%.d", pm2);
  client.publish("pm2.5", msg);
  Serial.print("PM2.5 (µg/m³): ");
  Serial.println(pm2);
  
  // Partículas PM10
  snprintf(msg, MSG_BUFFER_SIZE, "%.d", pm10);
  client.publish("pm10", msg);
  Serial.print("PM10 (µg/m³): ");
  Serial.println(pm10);
  
  Serial.println("---------------------------");
  Serial.println("Datos de sensores publicados");
}


void setup() {
 Serial.begin(9600);
  
  // ----------------------OLED---------------------------------------
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.print("INICIANDO \n SISTEMA...");
  display.display(); 
  delay(2000);

  //--------------------------------SD-----------------------------------------
  pinMode(chipSelect,OUTPUT);
  SPI.begin();
  if (!SD.begin(chipSelect)) {
    errorSD();
    return;
  }
  SDbien();
  delay(1000);
  //----------------------DHT22-------------------------------------------------

  //Inicialización del DHT22
  dht.begin();

  //----------------------------------MQ 135------------------------------------
  analogReadResolution(12); // Configura la resolución del ADC a 12 bits (4096 niveles)


  //-----------------------------------------PMS--------------------------------------
  pmsSerial.begin(9600);  ///< Inicializa SoftwareSerial a 9600 baudios
  delay(4000);

  //---------------------------------------MHZ19B-------------------------------------------------
  mySerial.begin(9600);
  myMHZ19.begin(mySerial);
  // Turn auto calibration ON (OFF autoCalibration(false))
  myMHZ19.autoCalibration();
  
  display.clearDisplay();
  
  
  // Limpia la pantalla
  display.clearDisplay();
  
  // Muestra la imagen en la pantalla
  display.drawBitmap(0, 0, logo, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(2000);
  
  // Limpia la pantalla nuevamente para dibujar nuevos gráficos
  display.clearDisplay();

  //---------------------------------------MQTT-------------------------------------------------

  pinMode(BUILTIN_LED, OUTPUT);     // Inicializar el pin del LED integrado como salida
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

/**
 * @brief Función principal de bucle que se ejecuta repetidamente.
 * En este caso, actualiza la pantalla OLED mostrando diferentes textos.
 */
void loop() {
  obtener_tem_hum();
  MQ135();
  SensMHZ19();
  PMS_b();
  cargarDatos();
  //Serial.println(hum);
  //Serial.println(tem);

  display.display(); 
  delay(1000);
  display.clearDisplay();
  // Limpia la pantalla
  display.clearDisplay();

  //MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) { // Enviar cada 2 segundos
    lastMsg = now;
    sendSensorData(); // Enviar los valores simulados de los sensores
  }
}

// ------------ FUNCIONES DESARROLLADAS--------------------------------

void obtener_tem_hum(){
  hum = dht.readHumidity();
  tem = dht.readTemperature();
  display.setCursor(0, 3);
  display.print("H: ");
  display.print(hum);
  display.println(" %");
  display.print("T: ");
  display.print(tem);
  display.println(" C");
}

void SensMHZ19(){
  co2Level = myMHZ19.getCO2();

  // Display the CO2 level on the serial monitor 
  display.setCursor(0, 39);
  display.print("CO2 (ppm): ");                      
  display.println(co2Level);  
  // Get the current temperature in Celsius
  temperature = myMHZ19.getTemperature();

  // Display the temperature on the serial monitor
  //Serial.print("Temperature (C): ");                  
  //Serial.println(temperature);  
  display.print("Te: ");                  
  display.println(temperature); 
}

void MQ135(){
  
  sensorValue = digitalRead(MQ135_PIN); // Lee el valor del sensor desde el pin GPIO34
  // Imprime el valor del sensor y el voltaje en el monitor serial
  //Serial.print("Valor del sensor: ");
  //Serial.print(sensorValue);
  //Serial.print(" - MQ135: ");
  //Serial.println(sensorValue);
  display.setCursor(0, 21);
  display.print("mq: ");
  display.println(sensorValue);
}

void PMS_b(){
  if (pms.read(data)){  ///< Read data from the PMS5003 sensor 
    // Print the dust concentration data to the serial monitor
    pm1 = data.PM_AE_UG_1_0;
    pm2 = data.PM_AE_UG_2_5;
    pm10 = data.PM_AE_UG_10_0;

    //Serial.println("Dust Concentration");
    //Serial.println("PM1.0: " + String(pm1) + " (ug/m3)");
    //Serial.println("PM2.5: " + String(pm2) + " (ug/m3)");
    //Serial.println("PM10: " + String(pm10) + " (ug/m3)");
    /*
    display.setCursor(0, 50);
    display.print("PM1.0: ");
    display.print(data.PM_AE_UG_1_0);
    display.println("ug/m3");
    display.print("PM2.5: ");
    display.println(data.PM_AE_UG_2_5);
    display.println("ug/m3");
    display.print("PM10: ");
    display.println(data.PM_AE_UG_10_0);
    display.println("ug/m3");*/
  }
}


// Función para abrir archivo donde se guardan los datos
void abrirArchivoSD(){
  // Abrir o crear el archivo donde se almacenarán los datos
  myFile = SD.open("datosCefiro.txt", FILE_WRITE);
  if (!myFile) {
    Serial.println("Error al abrir el archivo en la SD");
    return;
  }
  Serial.println("Archivo abierto exitosamente");
}

void cargarDatos() {
  // Escribir los valores en el archivo de la tarjeta SD
  myFile = SD.open("/dF_Cefiro.txt", FILE_APPEND);
  if (myFile) {


    // Guardar los datos en la SD
    myFile.print("Valor MQ135: ");
    myFile.print(sensorValue);
    myFile.print(",Temp: ");
    myFile.print(tem);
    myFile.print(",Hum: ");
    myFile.print(hum);
    myFile.print(",Temp_MHZ: ");
    myFile.print(temperature);
    myFile.print(",CO2 (ppm): ");
    myFile.print(co2Level);
    myFile.print("PM_1.0: ");
    myFile.print(pm1);
    myFile.print("PM_2.5: ");
    myFile.print(pm2);
    myFile.print("PM_10: ");
    myFile.println(pm10);
    myFile.close();  // Cerrar el archivo después de escribir

    // Mostrar los valores en el Monitor Serie

    Serial.print("Valor MQ135: ");
    Serial.print(sensorValue);
    Serial.print(",Temperature: ");
    Serial.print(tem);
    Serial.print("ºC ");
    Serial.print(",Humidity: ");
    Serial.print("% ");
    Serial.print(hum);
    Serial.print(",Temp MHZ: ");                      
    Serial.print(temperature); 
    Serial.print(",CO2 (ppm): ");                      
    Serial.print(co2Level); 
    Serial.print(",PM_1.0: ");                      
    Serial.print(pm1); 
    Serial.print(",PM_2.5: ");                      
    Serial.print(pm2); 
    Serial.print(",PM_10: ");                      
    Serial.println(pm10); 
    Serial.println("Datos guardados correctamente en la SD");

  } else {
    Serial.println("Error al escribir en el archivo");
  }
}

void errorSD(){
    display.clearDisplay();
    display.setCursor(0, 10);
    Serial.println(" Error al inicializar la tarjeta SD");
    display.print("FALLA DEL SISTEMA...\nerror: SD");
    display.display(); 
}

void SDbien(){
  Serial.println("Tarjeta SD inicializada correctamente");
  display.clearDisplay();
  display.drawBitmap(0,0,logo,128,64,WHITE);
  display.display(); 
}