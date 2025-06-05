//7in1 Sensor Code:
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <esp_now.h>
#include <HardwareSerial.h>
 
// Replace with your Firebase project credentials
#define FIREBASE_HOST "***" // Your Firebase Database URL
#define FIREBASE_AUTH "***" // Your Firebase Database Secret
 
// WiFi credentials
char ssid[] = "_";   // Wi-Fi namef
char pass[] = "_";      // Wi-Fi password
 
// RS485 Pins for Soil Sensor
#define RS485RX 16
#define RS485TX 17
 
// CO2 Sensor communication pins and request/response
HardwareSerial mySerial(1);
byte request[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // Request CO2 level
byte response[9]; // Array to hold the response from the CO2 sensor
 
unsigned char byteRequest[8] = {0X01, 0X03, 0X00, 0X00, 0X00, 0X07, 0X04, 0X08};
unsigned char byteResponse[19] = {};
 
// Firebase objects
FirebaseData firebaseData;
FirebaseAuth firebaseAuth;  
FirebaseConfig firebaseConfig;
 
// Receiver's MAC Address for ESP-NOW
uint8_t receiverMAC[] = {0x34, 0x5F, 0x45, 0xAB, 0xD7, 0xF4};
 
// Structure to hold soil sensor data
typedef struct {
  float soilHumidity;
} SensorData;
 
SensorData sensorData;
// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}
 
void setup() {
  // Start Serial for debugging
  Serial.begin(115200);
 
  // Start Serial2 for RS485 communication (soil sensor)
  Serial2.begin(4800, SERIAL_8N1, RS485RX, RS485TX);
 
  // Start the CO2 sensor communication
  mySerial.begin(9600, SERIAL_8N1, 26, 27);  // Use correct TX, RX pins for your hardware
 
  // Connect to Wi-Fi
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
 
  // Configure Firebase
  firebaseConfig.database_url = FIREBASE_HOST;
  firebaseAuth.user.email = "_"; // Leave empty if not using Firebase Authentication
  firebaseAuth.user.password = "_"; // Leave empty if not using Firebase Authentication
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
 
  // Initialize Firebase
  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);
  Serial.println("Connected to Firebase!");
 
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_send_cb(onDataSent);
 
  // Add the receiver's MAC Address
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
 
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
 
  // Give CO2 sensor time to initialize
  delay(2000);
  Serial.println("CO2 Sensor Ready");
}
 
void loop() {
  // Read CO2 data from the sensor
  mySerial.write(request, sizeof(request));
  delay(1000);
 
  if (mySerial.available() >= 9) {
    mySerial.readBytes(response, 9);
    int co2ppm = (response[2] * 256 ) + response[3];
    Serial.print("CO2 Level: ");
    Serial.print(co2ppm);
    Serial.println(" ppm");
 
    // Upload CO2 data to Firebase
    if (Firebase.ready()) {
      Firebase.setInt(firebaseData, "/SensorData/6CO2Level", co2ppm);
      Serial.println("CO2 data uploaded to Firebase!");
    }
  } else {
    Serial.println("No response from CO2 sensor...");
  }
 
  // Read soil sensor data (humidity)
  Serial2.write(byteRequest, sizeof(byteRequest));
  if (Serial2.available() >= sizeof(byteResponse)) {
    Serial2.readBytes(byteResponse, sizeof(byteResponse));
    unsigned int soilHumidity = (byteResponse[3] << 8) | byteResponse[4];
    sensorData.soilHumidity = (float)soilHumidity / 10.0;
    Serial.print("Soil Humidity: ");
    Serial.println(sensorData.soilHumidity);
 
    // Send soil sensor data via ESP-NOW
    esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
 
    // Upload soil sensor data to Firebase
    unsigned int soilTemperature = (byteResponse[5] << 8) | byteResponse[6];
    unsigned int soilConductivity = (byteResponse[7] << 8) | byteResponse[8];
    unsigned int soilPH = (byteResponse[9] << 8) | byteResponse[10];
    unsigned int nitrogen = (byteResponse[11] << 8) | byteResponse[12];
    unsigned int phosphorus = (byteResponse[13] << 8) | byteResponse[14];
    unsigned int potassium = (byteResponse[15] << 8) | byteResponse[16];
 
    if (Firebase.ready()) {
      Firebase.setFloat(firebaseData, "/SensorData/6SoilHumidity", sensorData.soilHumidity);
      Firebase.setFloat(firebaseData, "/SensorData/6SoilTemperature", soilTemperature / 10.0);
      Firebase.setInt(firebaseData, "/SensorData/6SoilConductivity", soilConductivity);
      Firebase.setFloat(firebaseData, "/SensorData/6SoilPH", soilPH / 10.0);
      Firebase.setInt(firebaseData, "/SensorData/6Nitrogen", nitrogen);
      Firebase.setInt(firebaseData, "/SensorData/6Phosphorus", phosphorus);
      Firebase.setInt(firebaseData, "/SensorData/6Potassium", potassium);

