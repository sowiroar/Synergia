#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2

// Configuración de red
const char* ssid = "ORBIN";
const char* password = "1053834012";
const char* mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

// Valores simulados de los sensores
float humidity = 90.0;         // Humedad simulada (%)
float temperatureDHT22 = 22.5; // Temperatura simulada DHT22 (°C)
float temperatureMHZ19B = 23.0;// Temperatura simulada MH-Z19B (°C)
float co2Level = 450.0;        // Nivel de CO2 simulado (ppm)
int sensorValueMQ135 = 1;      // Valor digital MQ135 (1 o 0)
float pm1_0 = 10.0;            // PM1.0 simulada (µg/m³)
float pm2_5 = 25.0;            // PM2.5 simulada (µg/m³)
float pm10 = 40.0;             // PM10 simulada (µg/m³)


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
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", humidity);
  client.publish("humidity", msg);
  Serial.print("Humedad (%): ");
  Serial.println(humidity);
  
  // Temperatura DHT22
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", temperatureDHT22);
  client.publish("temperature", msg);
  Serial.print("Temperatura DHT22 (°C): ");
  Serial.println(temperatureDHT22);
  
  // Temperatura MH-Z19B
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", temperatureMHZ19B);
  client.publish("temperature2", msg);
  Serial.print("Temperatura MH-Z19B (°C): ");
  Serial.println(temperatureMHZ19B);
  
  // Nivel de CO2
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", co2Level);
  client.publish("co2", msg);
  Serial.print("Nivel de CO2 (ppm): ");
  Serial.println(co2Level);
  
  // Valor del Sensor MQ135
  snprintf(msg, MSG_BUFFER_SIZE, "%d", sensorValueMQ135);
  client.publish("mq135", msg);
  Serial.print("Sensor MQ135 (Digital): ");
  Serial.println(sensorValueMQ135);
  
  // Partículas PM1.0
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", pm1_0);
  client.publish("pm1", msg);
  Serial.print("PM1.0 (µg/m³): ");
  Serial.println(pm1_0);
  
  // Partículas PM2.5
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", pm2_5);
  client.publish("pm2.5", msg);
  Serial.print("PM2.5 (µg/m³): ");
  Serial.println(pm2_5);
  
  // Partículas PM10
  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", pm10);
  client.publish("pm10", msg);
  Serial.print("PM10 (µg/m³): ");
  Serial.println(pm10);
  
  Serial.println("---------------------------");
  Serial.println("Datos de sensores publicados");
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Inicializar el pin del LED integrado como salida
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
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