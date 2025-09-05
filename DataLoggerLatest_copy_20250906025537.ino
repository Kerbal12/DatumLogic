#define BLYNK_TEMPLATE_ID "TMPL358Y-gi_B"
#define BLYNK_TEMPLATE_NAME "DatumLogiq"
#define BLYNK_AUTH_TOKEN "HEXmPAp-X_IuVEJfFGtJ1W6Q3-tRUrPo"
#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>  // Library for BMP180
#include <BH1750.h>           // Library for BH1750
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>
#include "time.h"
#define RESET_TIME 3000

#define BUTTON_PIN 35
#define BUTTON_PIN2 34
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define DHTPIN 13
#define DHTTYPE DHT22
#define MOISTURE_PIN 36
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 25
#define SD_CS_PIN 21  // Chip select pin for SD card
#define SD_MISO_PIN 5
#define SD_MOSI_PIN 18
#define SD_SCK_PIN 19
// SIM800 pins on ESP32 using TX2 and RX2
#define SIM800_RX_PIN 16  // GPIO16 (D16) on ESP32, connected to SIM800 TX
#define SIM800_TX_PIN 17  // GPIO17 (D17) on ESP32, connected to SIM800 RX
#define LED1 32
#define LED2 33
#define relay1 22
#define relay2 23
HardwareSerial sim800Serial(2);  // Use Serial2 for SIM800 communication

// Initialize sensors
Adafruit_BMP085 bmp180;  // Initialize BMP180 sensor
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;  // Initialize BH1750 sensor
unsigned long busyClearTime = 3000;
bool clearBusyPending = false;

// Initialize RTC
RTC_DS3231 rtc;
unsigned long currentMillis;
String GAS_ID = "AKfycby7uiarkGvFCAQ6jcWveCIGUXLpb8wf9JyASDCvlQX6txIxYvXEelqd7RHNZcOHiCokGg";
const char* host = "script.google.com";
// APN settings for Vodafone Idea (VI) in India
const char apn[] = "www";
const char gprsUser[] = "";
const char gprsPass[] = "";
int displayMode = 0;
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
const int dryValue = 1188;  
const int wetValue = 614;

String url = "/macros/s/" + GAS_ID + "/exec?";
unsigned long previousMillis = 0;
String postData;
int interval = 60000;  // 10 seconds for testing
char auth[] = "HEXmPAp-X_IuVEJfFGtJ1W6Q3-tRUrPo";
// Variables for sensor data
float temperature = -1;
float humidity = -1;
float moisturePercentage = -1;
float pressure = -1;  // Default to -1
float lux = -1;       // Light intensity in lux
int pumpThresholdValue;
int luxThresholdValue;
bool blynkControlLightsActive = false;
bool blynkControlPumpActive = false;
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // Adjust for your timezone
const int daylightOffset_sec = 0;
int logStatus;
bool sdFileExist;
bool isLogging = false;
bool backUpRequired = true;
String ssid;
String password;
int lastCSQ = -1;
bool gsmConnected = false;

WiFiManager wifiManager;
void logDataUsingCore0() {
  // Create a task that runs glowLED() on core 0
  xTaskCreatePinnedToCore(
    [](void *pvParameters) {
      logData(); 
      vTaskDelete(NULL); // Delete task once done
    },
    "logDataUsingCore0",  // Task name
    16384,              // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Task priority
    NULL,              // Task handle
    0                  // Core 0
  );
}
void setRTC(int cond) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
  } else if (cond == 0) {
    // Only fallback if cond == 0 and local time is unavailable
    rtc.adjust(DateTime(2024, 7, 29, 22, 14, 0));
  }
}
String getLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return String("");  // Return an empty string on failure
  }

  // Format the date and time as a string
  char timeString[40];
  snprintf(timeString, sizeof(timeString), "(%d-%02d-%02d %02d:%02d:%02d)\n", 
           timeinfo.tm_year + 1900,  // tm_year is years since 1900
           timeinfo.tm_mon + 1,      // tm_mon is months since January (0-11)
           timeinfo.tm_mday,         // tm_mday is day of the month (1-31)
           timeinfo.tm_hour,         // tm_hour is hours since midnight (0-23)
           timeinfo.tm_min,          // tm_min is minutes (0-59)
           timeinfo.tm_sec);         // tm_sec is seconds (0-59)

  return String(timeString);  // Return as a String
}

void setup() {
  // Configure LED2 for PWM (GPIO 33, channel 0)
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  Serial.print("Init");
  WiFi.begin();
  setupBlynk();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  pinMode(relay1, OUTPUT); 
  digitalWrite(relay1, HIGH); 
  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, HIGH); 
  // Lambda function to show initialization message on display
  auto showInitializationMessage = [](const String& message) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(28, 0);
    display.print("DATA LOGGER");  // Always display at the top
    display.setCursor(0, 24);      // Start printing messages below the header
    display.print(message);
    display.display();
  };

  sim800Serial.begin(9600, SERIAL_8N1, SIM800_RX_PIN, SIM800_TX_PIN);  // Initialize SIM800 serial communication
  unsigned long startMillis = millis();
  while (millis() - startMillis < 5000) {  // Wait up to 5 seconds for SIM800 to respond
    if (sim800Serial.available()) {
      String response = sim800Serial.readString();
      if (response.indexOf("OK") >= 0) {
        Serial.println("SIM800 module initialized.");
        break;
    }
  }
}

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C communication
  Serial.begin(115200);  // Initialize Serial communication
  delay(1000);  // Allow time for the Serial Monitor to initialize

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (1);  // Halt the program if initialization fails
  }
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);
  display.setCursor(0, 0);
  display.print("DL");
  display.display();
  delay(1500);
  // Initialize RTC
  if (!rtc.begin()) {
    showInitializationMessage("RTC initialization failed");
    Serial.println("Couldn't find RTC");
    delay(500);
    while (1);  // Halt the program if initialization fails
  } else {
    setRTC(1);  // Initialize RTC
    showInitializationMessage("RTC Initialized");
    Serial.println("RTC Initialized");
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW); 
  }
  delay(500);

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time!");
    setRTC(0);  // Reset RTC time if lost power
  }

  // Initialize DHT22
  dht.begin();
  if (isnan(dht.readTemperature())) {
    showInitializationMessage("DHT22 Initialization failed");
  } else {
    showInitializationMessage("DHT22 Initialized");
    Serial.println("DHT22 Initialized");
  }
  delay(500);

  // Initialize BMP180
  if (!bmp180.begin()) {
    Serial.println("Couldn't find BMP180 sensor! Setting pressure to -1.");
    showInitializationMessage("BMP180 Initialization failed");
  } else {
    showInitializationMessage("BMP180 Initialized");
    Serial.println("BMP180 Initialized");
  }
  delay(500);

  // Initialize BH1750
  if (!lightMeter.begin()) {
    Serial.println("Couldn't find BH1750 sensor! Setting lux to -1.");
    showInitializationMessage("BH1750 Initialization failed");
  } else {
    showInitializationMessage("BH1750 Initialized");
    Serial.println("BH1750 Initialized");
  }
  delay(500);

  // Initialize SD card
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);  // Initialize SPI communication for SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Initialization of SD card failed!");
    showInitializationMessage("SD Card Init Failed");
    delay(500);
    while (1);  // Halt the program if initialization fails
  } else {
    showInitializationMessage("SD Card Initialized");
    Serial.println("SD Card Initialized");
  }
  delay(500);
  showInitializationMessage("SIM800L and GPRS getting Ready...");
  Serial.println("Initializing GPRS...");
  if (!initGPRS()) {
    Serial.println("Failed to initialize GPRS.");
    showInitializationMessage("Failed to initialize GPRS.");
    delay(500);
  } else {
    Serial.println("GPRS initialized successfully.");
    showInitializationMessage("GPRS Ready!");
    delay(500);
  }
  // Final message
  display.clearDisplay();
  display.setCursor(28, 0);
  display.setTextSize(1);
  display.print("DATA LOGGER");  // Always display at the top
  display.setCursor(0, 24);
  display.setTextSize(2);
  display.print("SYSTEM    ACTIVATED");  // Updated text size and alignment
  display.display();
  delay(500);

  // Set up the moisture sensor and button pins
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON_PIN2, INPUT);

}


unsigned long previousMillisTemp = 0;  
unsigned long previousMillisHumidity = 0;  
unsigned long previousMillisMoisture = 0;  
unsigned long previousMillisPressure = 0;  
unsigned long previousMillisLight = 0;

// Adjust read intervals for each sensor (in milliseconds)
unsigned long readIntervalTemp = 1000;    // Temperature sensor read interval
unsigned long readIntervalHumidity = 1000; // Humidity sensor read interval
unsigned long readIntervalMoisture = 1000; // Moisture sensor read interval
unsigned long readIntervalPressure = 1000; // Pressure sensor read interval
unsigned long readIntervalLight = 1000;    // Light intensity sensor read interval

// Read functions with frequency control
void readTemperature(){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisTemp >= readIntervalTemp) {
        previousMillisTemp = currentMillis;
        temperature = dht.readTemperature();
        if (isnan(temperature)) temperature = -1;
    }
}

void readHumidity(){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisHumidity >= readIntervalHumidity) {
        previousMillisHumidity = currentMillis;
        humidity = dht.readHumidity();
        if (isnan(humidity)) humidity = -1;
    }
}


void readMoisture() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisMoisture >= readIntervalMoisture) {
    previousMillisMoisture = currentMillis;
    int moistureValue = analogRead(MOISTURE_PIN);
    moisturePercentage = map(moistureValue, dryValue, wetValue, 0, 100);
    moisturePercentage = constrain(moisturePercentage, 0, 100);
  }
}

void readPressure(){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisPressure >= readIntervalPressure) {
        previousMillisPressure = currentMillis;
        pressure = bmp180.begin() ? bmp180.readPressure() / 100.0F : -1; 
    }
}

void readLightIntensity(){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisLight >= readIntervalLight) {
        previousMillisLight = currentMillis;
        lux = lightMeter.readLightLevel();
    }
}
void loop() {
  handleLEDs();
  handleDisplayAndButtonActions();
  Blynk.run();
  readHumidity();
  readTemperature();
  readMoisture();
  readLightIntensity();
  readPressure();
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    logDataUsingCore0();
  }

if(blynkControlLightsActive){
  blynkCheckAndControlLights();
}
if(blynkControlPumpActive){
  blynkCheckAndControlPump();
}
}
void manualLog() {
  logDataUsingCore0();              // call your existing log function
  previousMillis = millis();  // reset timer so next auto log waits full interval
}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin();
    Serial.print("Attempting to connect to WiFi");
    int maxTries = 3;
    while (WiFi.status() != WL_CONNECTED && maxTries > 0) {
      delay(1000);
      Serial.print(".");
      maxTries--;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected");
    } else {
      Serial.println("WiFi connection failed");
    }
  }
}


bool sendDataViaWiFi() {
  if (WiFi.isConnected()) {
    WiFiClientSecure client;
    client.setInsecure();  // Disable certificate verification for the HTTPS request
    const int httpsPort = 443;

    // Attempt to connect to the host
    if (!client.connect(host, httpsPort)) {
      Serial.println("WiFi connection to host failed");
      return false;
    }

    // Send the HTTP POST request
    Serial.print(url);
    client.print("POST " + url + " HTTP/1.1\r\n");
    client.print(String("Host: ") + host + "\r\n");
    client.print("Content-Type: application/json\r\n");
    client.print("Content-Length: " + String(postData.length()) + "\r\n");
    client.print("Connection: close\r\n\r\n");
    client.print(postData);


    delay(3000);  // Give time for the response to be received

    // Read the response (optional)
    while (client.connected() || client.available()) {
      client.read();
    }

    Serial.println("Data sent via WiFi successfully");
    return true;
  }

  Serial.println("WiFi not connected");
  return false;
}

void handleDisplayAndButtonActions() {
  unsigned long currentMillis = millis();

  int button1State = digitalRead(BUTTON_PIN);
  int button2State = digitalRead(BUTTON_PIN2);

  // === Dual-button RESET ===
  static unsigned long dualButtonStart = 0;
  if (button1State == HIGH && button2State == HIGH) {
    if (dualButtonStart == 0) dualButtonStart = currentMillis;
    if (currentMillis - dualButtonStart >= RESET_TIME) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(28, 0);
      display.print("DATA LOGGER");
      display.setCursor(0, 24);
      display.setTextSize(2);
      display.print("SYSTEM    RESETTING");
      display.display();
      delay(1000);
      display.clearDisplay();
      display.display();
      ESP.restart();
    }
  } else {
    dualButtonStart = 0;
  }

  // === Button 1: Short & Long Press Handling ===
  static int lastButton1State = LOW;
  static unsigned long button1PressedTime = 0;
  static bool button1LongPressHandled = false;
  const unsigned long shortPressThreshold = 50;
  const unsigned long longPressThreshold = RESET_TIME;

  if (button1State != lastButton1State) {
    if (button1State == HIGH) {
      button1PressedTime = currentMillis;
      button1LongPressHandled = false;
    } else if (button1State == LOW) {
      unsigned long pressDuration = currentMillis - button1PressedTime;
      if (pressDuration >= shortPressThreshold && pressDuration < longPressThreshold && !button1LongPressHandled) {
        // === SHORT PRESS: WiFi + GSM Status ===
        updateGSMStatus(); // make sure this fn updates lastCSQ and gsmConnected
        display.clearDisplay();
        display.setTextSize(1);

        display.setCursor(0, 0);
        display.print("WiFi: ");
        if (WiFi.status() == WL_CONNECTED) {
          display.print("OK");
          display.setCursor(0, 10);
          int rssi = WiFi.RSSI();
          int wifiBars = (rssi >= -50) ? 5 :
                         (rssi >= -60) ? 4 :
                         (rssi >= -70) ? 3 :
                         (rssi >= -80) ? 2 :
                         (rssi >= -90) ? 1 : 0;
          display.print("Signal: ");
          for (int i = 0; i < wifiBars; i++) display.print(".");
        } else {
          display.print("DISCONNECTED");
        }

        display.setCursor(0, 26);
        display.print("GSM: ");
        if (gsmConnected) {
          display.print("OK");
          int gsmBars = (lastCSQ >= 20) ? 5 :
                        (lastCSQ >= 15) ? 4 :
                        (lastCSQ >= 10) ? 3 :
                        (lastCSQ >= 5)  ? 2 :
                        (lastCSQ >= 1)  ? 1 : 0;
          display.setCursor(0, 36);
          display.print("Signal: ");
          for (int i = 0; i < gsmBars; i++) display.print(".");
        } else {
          display.print("NO SIGNAL");
        }

        display.display();
        delay(1500);
      }
    }
  }

  if (button1State == HIGH && (currentMillis - button1PressedTime >= longPressThreshold) && !button1LongPressHandled) {
    // === LONG PRESS: WiFi Reset ===
    Serial.println("WiFi config reset triggered by button1 long press.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(28, 0);
    display.print("DATA LOGGER");
    display.setCursor(0, 24);
    display.setTextSize(2);
    display.print("WIFI SETUP");
    display.display();
    delay(1000);
    wifiManager.resetSettings();
    setupWifi();
    setupBlynk();
    button1LongPressHandled = true;
  }

  lastButton1State = button1State;

  // === Button 2: Short & Long Press Handling ===
  static int lastButton2State = LOW;
  static unsigned long button2PressedTime = 0;
  static bool button2LongPressHandled = false;

  if (button2State != lastButton2State) {
    if (button2State == HIGH) {
      button2PressedTime = currentMillis;
      button2LongPressHandled = false;
    } else if (button2State == LOW) {
      unsigned long pressDuration = currentMillis - button2PressedTime;
      if (pressDuration >= shortPressThreshold && pressDuration < longPressThreshold && !button2LongPressHandled) {
        displayMode++;
        if (displayMode > 6) displayMode = 0;
        Serial.print("Display Mode Changed: ");
        Serial.println(displayMode);
      }
    }
  }

  if (button2State == HIGH && (currentMillis - button2PressedTime >= longPressThreshold) && !button2LongPressHandled) {
    // === LONG PRESS: Manual Log ===
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(28, 0);
    display.print("DATA LOGGER");
    display.setCursor(0, 24);
    display.setTextSize(2);

    if (isLogging) {
      display.print("BUSY");
      Serial.println("Manual log skipped: BUSY");
    } else {
      display.print("LOGGING   DATA");
      Serial.println("Manual log triggered.");
      manualLog();
    }

    display.display();
    delay(1000);
    button2LongPressHandled = true;
  }

  lastButton2State = button2State;

  // === Display update ===
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);

  switch (displayMode) {
    case 0: {
      DateTime now = rtc.now();
      display.print("Time, Date");
      display.setCursor(0, 24);
      display.printf("%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      display.setCursor(0, 48);
      display.printf("%02d/%02d/%04d", now.day(), now.month(), now.year());
      break;
    }
    case 1:
      display.print("C.S.M.S.");
      display.setCursor(0, 24);
      display.print("Moisture: ");
      display.setCursor(4, 48);
      readMoisture();
      display.print(moisturePercentage); display.println(" %");
      break;
    case 2:
      display.print("BMP180");
      display.setCursor(0, 24);
      display.print("Pressure: ");
      display.setCursor(4, 48);
      readPressure();
      display.print(pressure); display.println(" hPa");
      break;
    case 3:
      display.print("BH1750");
      display.setCursor(0, 24);
      display.print("Light: ");
      display.setCursor(4, 48);
      readLightIntensity();
      display.print(lux); display.println(" lux");
      break;
    case 4:
      display.print("DHT22");
      display.setCursor(0, 24);
      display.print("Humidity: ");
      display.setCursor(4, 48);
      readHumidity();
      display.print(humidity); display.println(" %");
      break;
    case 5:
      display.setTextSize(2);
      display.print("DHT22");
      display.setCursor(0, 24);
      display.print("Temp: ");
      display.setCursor(4, 48);
      readTemperature();
      display.print(temperature); display.println(" C");
      break;
    case 6: {
      display.setTextSize(1);
      DateTime now = rtc.now();
      display.setCursor(0, 0);
      display.printf("Time: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
      display.setCursor(0, 10);
      display.print("Moist: ");
      readMoisture();
      display.print(moisturePercentage); display.println("%");
      display.setCursor(0, 20);
      display.print("Press: ");
      readPressure();
      display.print(pressure); display.println(" hPa");
      display.setCursor(0, 30);
      display.print("Light: ");
      readLightIntensity();
      display.print(lux); display.println(" lux");
      display.setCursor(0, 40);
      display.print("Temp: ");
      readTemperature();
      display.print(temperature); display.println(" C");
      display.setCursor(0, 50);
      display.print("Humi: ");
      readHumidity();
      display.print(humidity); display.println(" %");
      break;
    }
  }

  display.display();
}



bool initGPRS() {
  // Initialize SIM800 module
  Serial.println("Sending AT command...");
  sim800Serial.println("AT");
  delay(1000);
  printResponse();

  // Check SIM card presence
  Serial.println("Checking SIM card presence...");
  sim800Serial.println("AT+CPIN?");
  delay(1000);
  printResponse();

  // Configure connection type: GPRS
  Serial.println("Configuring GPRS connection type...");
  sim800Serial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  delay(1000);
  printResponse();

  // Set APN
  Serial.println("Setting APN...");
  sim800Serial.print("AT+SAPBR=3,1,\"APN\",\"");
  sim800Serial.print(apn);
  sim800Serial.println("\"");
  delay(1000);
  printResponse();

  // Enable GPRS
  Serial.println("Enabling GPRS...");
  sim800Serial.println("AT+SAPBR=1,1");
  delay(3000);
  printResponse();

  // Check GPRS status
  Serial.println("Checking GPRS status...");
  sim800Serial.println("AT+SAPBR=2,1");
  delay(1000);
  if (sim800Serial.find("+SAPBR: 1,1")) {
    Serial.println("GPRS connection established.");
    return true;
  } else {
    Serial.println("Failed to establish GPRS connection.");
    return false;
  }
}


bool sendDataViaGSM() {
  Serial.println("Sending data to Google Sheets...");

  //  Kill leftover HTTP session — must be first
  sim800Serial.println("AT+HTTPTERM");
  delay(1000); // Let it clean

  //  Init HTTP
  sim800Serial.println("AT+HTTPINIT");
  delay(1000);
  if (!printResponse()) return false;

  //  SSL enable
  sim800Serial.println("AT+HTTPSSL=1");
  delay(1000);
  if (!printResponse()) return false;

  //  Set URL
  sim800Serial.print("AT+HTTPPARA=\"URL\",\"https://script.google.com");
  sim800Serial.print(url);
  sim800Serial.println("\"");
  delay(1000);
  if (!printResponse()) return false;

  //  Set Content-Type
  sim800Serial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(1000);
  if (!printResponse()) return false;

  //  Send data payload
  sim800Serial.print("AT+HTTPDATA=");
  sim800Serial.print(postData.length());
  sim800Serial.println(",10000");
  delay(1000);  // Let it respond with DOWNLOAD
  if (!waitFor("DOWNLOAD", 3000)) {
    Serial.println("Failed to get DOWNLOAD prompt");
    return false;
  }

  sim800Serial.print(postData);  // Actual data
  delay(1000);
  if (!printResponse()) return false;

  //  Trigger HTTP POST
  sim800Serial.println("AT+HTTPACTION=1");
  delay(6000);
  if (!printResponse()) return false;

  //  Read response
  sim800Serial.println("AT+HTTPREAD");
  delay(2000);
  while (sim800Serial.available()) {
    Serial.println(sim800Serial.readStringUntil('\n'));
  }

  //  Clean exit
  sim800Serial.println("AT+HTTPTERM");
  delay(1000);

  Serial.println("Data successfully sent via GSM.");
  return true;
}

bool waitFor(const char* keyword, unsigned long timeout) {
  unsigned long start = millis();
  String response = "";
  while (millis() - start < timeout) {
    while (sim800Serial.available()) {
      char c = sim800Serial.read();
      response += c;
      if (response.indexOf(keyword) != -1) return true;
    }
  }
  return false;
}



bool printResponse() {
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) { // Wait up to 2 seconds for response
    if (sim800Serial.available()) {
      String line = sim800Serial.readStringUntil('\n');
      Serial.println(line);
      // Check if the response indicates success
      if (line.indexOf("OK") != -1 || line.indexOf("200") != -1) {
        return true;
      }
    }
  }

  Serial.println("Error: Unexpected response or timeout.");
  return false;
}
void displaylogging(){
    digitalWrite(LED1,HIGH);
    delay(500);
    digitalWrite(LED1,LOW);
    delay(500);
    digitalWrite(LED1, HIGH);
    delay(500);
    digitalWrite(LED1,LOW);    
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(28, 0);
    display.print("DATA LOGGER");  // Always display at the top
    display.setCursor(0, 24);
    display.setTextSize(1);  // Start printing messages below the header
    display.print("DATA IS BEING LOGGED PLEASE WAIT A FEW    SECONDS...");
    display.display();
    display.setTextSize(2);
    }

BLYNK_WRITE(V9) {
    int buttonState = param.asInt(); // Get the button state
    if (buttonState == 1) {         // If button is pressed
        readTemperature();
        Blynk.virtualWrite(V0, temperature); // Send temperature to V0
    }
}
BLYNK_WRITE(V10) {
    int buttonState = param.asInt(); 
    if (buttonState == 1) {        
        readHumidity();
        Blynk.virtualWrite(V1, humidity);
    }
}
BLYNK_WRITE(V11) {
    int buttonState = param.asInt(); 
    if (buttonState == 1) {        
        readLightIntensity();
        Blynk.virtualWrite(V2, lux);
    }
}
BLYNK_WRITE(V12) {
    int buttonState = param.asInt(); 
    if (buttonState == 1) {        
        readPressure();
        Blynk.virtualWrite(V3, pressure);
    }
}
BLYNK_WRITE(V13) {
    int buttonState = param.asInt(); 
    if (buttonState == 1) {        
        readMoisture();
        Blynk.virtualWrite(V4, moisturePercentage);
    }
}

BLYNK_WRITE(V6) {
    int relay2State = param.asInt(); 
    digitalWrite(relay2, !relay2State); // Update relay state
    blynkControlLightsActive = false;  // Disable automatic control
    Blynk.virtualWrite(V16, 0);        // Sync V16 to OFF in the app
}




// These are the proper functions to read the virtual pin values when updated

BLYNK_WRITE(V8) {
  luxThresholdValue = param.asInt();  // Update luxThresholdValue with the value from V8
}
BLYNK_WRITE(V16) {
    int setLightsThreshold = param.asInt();
    blynkControlLightsActive = setLightsThreshold;
    if (!blynkControlLightsActive) {
        digitalWrite(relay2, HIGH); // Turn off lights
        Blynk.virtualWrite(V6, 0); // Sync button state
    }
}


void blynkCheckAndControlLights() {
    if (!blynkControlLightsActive) return; // Skip if manual override is active

    if (lux < luxThresholdValue) {
        if (digitalRead(relay2) != LOW) { // Avoid redundant updates
            digitalWrite(relay2, LOW);    // Turn lights ON
            Blynk.virtualWrite(V6, 1);   // Sync button state
        }
    } else {
        if (digitalRead(relay2) != HIGH) { // Avoid redundant updates
            digitalWrite(relay2, HIGH);   // Turn lights OFF
            Blynk.virtualWrite(V6, 0);   // Sync button state
        }
    }
}

BLYNK_WRITE(V5) {
    int relay1State = param.asInt(); 
    digitalWrite(relay1, !relay1State); // Update relay state
    blynkControlPumpActive = false;    // Disable automatic control
    Blynk.virtualWrite(V15, 0);       // Sync auto mode (V15) to OFF
}
BLYNK_WRITE(V17) {
    backUpRequired = param.asInt();
}

BLYNK_WRITE(V7) {
  pumpThresholdValue = param.asInt();
}

BLYNK_WRITE(V15) {
    int setPumpThreshold = param.asInt(); 
    blynkControlPumpActive = setPumpThreshold;
    if (!blynkControlPumpActive) {
        digitalWrite(relay1, HIGH); // Turn off pump
        Blynk.virtualWrite(V5, 0); // Sync manual button state
    }
}

BLYNK_WRITE(V14) {
  int logDataState = param.asInt();
  if ((logDataState) && (!isLogging)) {
    manualLog();
  }
}

void blynkCheckAndControlPump() {
    if (!blynkControlPumpActive) return; // Skip if manual override is active

    if (moisturePercentage < pumpThresholdValue) {
        if (digitalRead(relay1) != LOW) { // Avoid redundant updates
            digitalWrite(relay1, LOW);    // Activate relay
            Blynk.virtualWrite(V5, 1);   // Update manual button state
        }
    } else {
        if (digitalRead(relay1) != HIGH) { // Avoid redundant updates
            digitalWrite(relay1, HIGH);   // Deactivate relay
            Blynk.virtualWrite(V5, 0);   // Update manual button state
        }
    }
}


BLYNK_WRITE(V18) {
  int blynkInterval = param.asInt();
  if (blynkInterval == 0) {  // Correct comparison operator (==) instead of assignment (=)
    interval = 600000;  // 10 minutes in milliseconds
  } else {
    interval = blynkInterval * 60000;  // Convert minutes to milliseconds
  }
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V9,V10,V11,V12,V13,V5,V6,V14,V7,V8,V16,V18,V15,V17); // Sync states 
}
int currentTempID = 0;

String makeJSONEntry(String timestamp, String temp, String hum, String soil, String press, String lux) {
  return "{\"timestamp\":\"" + timestamp + "\", " +
         "\"temperature\":" + temp + ", " +
         "\"humidity\":" + hum + ", " +
         "\"soilmoisture\":" + soil + ", " +
         "\"atmpressure\":" + press + ", " +
         "\"lightintensity\":" + lux + "}";
}

bool sendData() {
  int cycleCount = 0;
  bool dataSent = false;

  while (cycleCount < 3 && !dataSent) {
    // reconnectWiFi();  // Uncomment if you have this function

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Attempting to send data via WiFi...");
      Serial.println(postData);
      dataSent = sendDataViaWiFi();
      Serial.println(dataSent);
    }

    if (!dataSent) {
      Serial.println("WiFi failed, attempting GSM...");
      dataSent = sendDataViaGSM();
      Serial.println(dataSent);
    }

    if (!dataSent) {
      cycleCount++;
      Serial.println("Retrying WiFi and GSM cycle... (Attempt " + String(cycleCount) + ")");
      delay(1000);  // Small delay before retrying
    } else {
      break; // Success, break out of loop
    }
  }

  if (!dataSent) {
    Serial.println("Failed to send data after 3 attempts.");
    return false;
  }

  Serial.println("Data sent successfully.");
  return true;
}

void logData() {
  isLogging = true;
  Serial.println("Starting logData...");

  if (isnan(temperature)) temperature = -1;
  if (isnan(humidity)) humidity = -1;

  String timestamp = getFormattedTimestamp();  // Generate once

  logToSD("/dataLog.csv", timestamp, temperature, humidity, moisturePercentage, pressure, lux);

  if (!sdFileExist || !backUpRequired) {
    if (!sdFileExist) Serial.println("Backup File does not exist.");

    postData = "{\"data\":[";
    postData += makeJSONEntry(timestamp, String(temperature), String(humidity),
                              String(moisturePercentage), String(pressure), String(lux));
    postData += "]}";

    Serial.println("Post Data: " + postData);
    sendData();
  } else {
    logToSD("/temp.csv", timestamp, temperature, humidity, moisturePercentage, pressure, lux);
    Serial.println("Processing existing log file...");
    if (!backup()) {
      isLogging = false;
      return;
    }
  }

  isLogging = false;
}


bool backup() {
  const char* backupFilename = "/temp.csv";
  bool allSent = true;

  while (true) {
    if (!SD.exists(backupFilename)) {
      Serial.println("temp file does not exist. Nothing to send.");
      return false;
    }

    File backupFile = SD.open(backupFilename, FILE_READ);
    if (!backupFile) {
      Serial.println("Failed to open backup file for reading.");
      return false;
    }

    Serial.println("Reading backup file...");
    postData = "{\"data\":[";
    int entryCount = 0;
    bool firstEntry = true;

    while (backupFile.available() && entryCount < 10) {
      String line = backupFile.readStringUntil('\n');
      line.trim();
      if (line.isEmpty()) continue;

      // Split line into columns
      String dataArray[8];
      int startIndex = 0;
      for (int i = 0; i < 7; i++) {
        int commaIndex = line.indexOf(',', startIndex);
        if (commaIndex == -1) {
          dataArray[i] = line.substring(startIndex);
          break;
        }
        dataArray[i] = line.substring(startIndex, commaIndex);
        startIndex = commaIndex + 1;
      }
      dataArray[7] = line.substring(startIndex);  // light

      // Clean timestamp
      dataArray[0].trim(); // date
      dataArray[1].trim(); // time
      String timestamp = dataArray[0] + " " + dataArray[1];

      if (!firstEntry) postData += ",";
      firstEntry = false;

      postData += makeJSONEntry(timestamp, dataArray[2], dataArray[3],
                                dataArray[4], dataArray[5], dataArray[7]);

      entryCount++;
    }
    backupFile.close();
    postData += "]}";

    if (entryCount == 0) {
      Serial.println("No more entries to send.");
      return allSent;
    }

    Serial.println("Constructed JSON:");
    Serial.println(postData);

    if (sendData()) {
      Serial.println("Data sent successfully. Deleting sent entries...");

      // Trim first 10 lines from temp.csv
      backupFile = SD.open(backupFilename, FILE_READ);
      String remaining = "";
      for (int i = 0; i < 10 && backupFile.available(); i++) {
        backupFile.readStringUntil('\n');
      }
      while (backupFile.available()) {
        remaining += backupFile.readStringUntil('\n') + "\n";
      }
      backupFile.close();

      backupFile = SD.open(backupFilename, FILE_WRITE);
      if (backupFile) {
        backupFile.print(remaining);
        backupFile.close();
        Serial.println("Sent entries deleted.");
      } else {
        Serial.println("Failed to rewrite backup file.");
        return false;
      }
    } else {
      Serial.println("Failed to send data. Will retry next time.");
      allSent = false;
      break;
    }
  }

  return allSent;
}




void logToSD(const char* filename, String timestamp, float temp, float hum, float moisture, float press, float light) {
  Serial.print("Logging data to ");
  Serial.println(filename);
  File dataFile = SD.open(filename, FILE_APPEND);

  if (dataFile) {
    if (String(filename).indexOf("dataLog") >= 0) sdFileExist = true;

    Serial.println("Opened log file for appending.");

    // Use passed timestamp instead of generating again
    int spaceIndex = timestamp.indexOf(' ');
    String datePart = timestamp.substring(0, spaceIndex);
    String timePart = timestamp.substring(spaceIndex + 1);

    dataFile.print(datePart);
    dataFile.print(", ");
    dataFile.print(timePart);
    dataFile.print(", ");
    dataFile.print(temp);
    dataFile.print(", ");
    dataFile.print(hum);
    dataFile.print(", ");
    dataFile.print(moisture);
    dataFile.print(", ");
    dataFile.print(press);
    dataFile.print(", ");
    dataFile.println(light);

    dataFile.close();
    Serial.println("Data logged successfully.");
  } else {
    Serial.println("Failed to open log file.");
    if (String(filename).indexOf("dataLog") >= 0) sdFileExist = false;
  }
}



void setupWifi(){
  // Automatically connect to WiFi or start the portal if not connected
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(28, 0);
    display.print("Setup Wifi");  // Always display at the top
    display.setCursor(0, 24);      // Start printing messages below the header
    display.print("Open WiFi settings on your phone, connect to 'DATUM LOGIC'.");
    display.display();
    display.setTextSize(2);
  if (!wifiManager.autoConnect("DAUTUM LOGIC")) {
    Serial.println("Failed to connect and hit timeout. Restarting...");
    ESP.restart();  // Restart the ESP32 if connection fails
  }
}
String getFormattedTimestamp() {
  DateTime now = rtc.now();

  char buffer[20];  // Enough for "dd/mm/yy HH:MM:SS"
  sprintf(buffer, "%02d/%02d/%02d %02d:%02d:%02d",
          now.day(), now.month(), now.year() % 100,
          now.hour(), now.minute(), now.second());

  return String(buffer);
}


void setupBlynk() {
  ssid = WiFi.SSID(); 
  password = WiFi.psk();
  const char* ssidChar = ssid.c_str();
  const char* passChar = password.c_str();

  WiFi.begin(ssidChar, passChar); // Initiate WiFi connection
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500); // Retry for 10 seconds
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.config(auth); // Configure Blynk (non-blocking)
    Blynk.connect();    // Attempt to connect to Blynk server (non-blocking)
  } else {
    Serial.println("WiFi connection failed!");
    // Handle connection failure (e.g., retry logic or fallback)
  }
}
void handleLEDs() {
  static unsigned long prevMillisLED2 = 0;
  static bool led2State = LOW;
  const unsigned long intervalLED2 = 500;

  static unsigned long prevMillisLED1 = 0;
  static bool led1On = false;

  unsigned long currentMillis = millis();

  //  LED2: Fast blink while logging
  if (isLogging) {
    if (currentMillis - prevMillisLED2 >= intervalLED2) {
      prevMillisLED2 = currentMillis;
      led2State = !led2State;
      digitalWrite(LED2, led2State);
    }
  } else {
    led2State = LOW;
    digitalWrite(LED2, LOW);
  }

  if (!led1On && currentMillis - prevMillisLED1 >= 5000) {
    digitalWrite(LED1, HIGH);
    led1On = true;
    prevMillisLED1 = currentMillis;
  }

  if (led1On && currentMillis - prevMillisLED1 >= 100) {
    digitalWrite(LED1, LOW);
    led1On = false;
  }
}
void updateGSMStatus() {
  gsmConnected = false;
  lastCSQ = 0;

  // Flush serial
  while (sim800Serial.available()) sim800Serial.read();

  // === Network Registration Check ===
  sim800Serial.println("AT+CREG?");
  delay(200);
  while (sim800Serial.available()) {
    String line = sim800Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("+CREG:")) {
      if (line.indexOf(",1") >= 0 || line.indexOf(",5") >= 0) {
        gsmConnected = true;
      }
    }
  }

  // === Signal Strength ===
  sim800Serial.println("AT+CSQ");
  delay(200);
  while (sim800Serial.available()) {
    String line = sim800Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("+CSQ:")) {
      int colonIndex = line.indexOf(":");
      int commaIndex = line.indexOf(",");
      if (colonIndex != -1 && commaIndex != -1) {
        String csqStr = line.substring(colonIndex + 1, commaIndex);
        csqStr.trim();
        lastCSQ = csqStr.toInt();
      }
    }
  }
}