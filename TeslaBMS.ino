// LED States
// Green flash = all packs found
// Purple flash = searching for BMBs
// Blue flash = wifi connected, normal operation

// --------------------- Includes ---------------------
#include <Arduino.h>
#include "Logger.h"
#include "SerialConsole.h"
#include "BMSModuleManager.h"
#include "SystemIO.h"
#include "MQTTClient.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include "secrets.h"
//#include <LittleFS.h> // Will be used later to host the Web frontend to show stats
#include <Adafruit_NeoPixel.h> // To drive ws2812 LED
//#include "esp_adc_cal.h" // For calibration to improve readings via ADC - not used in this proejct.

// ------------------------- Init variables -------------------------
// -- Define the RX and TX pins used to talk to the BMBs --
#define BMB_RX_PIN      16
#define BMB_TX_PIN      17
#define BMB_FAULT_PIN   4

// -- MQTT Auth configuration --
#define MQTT_USER SECRET_MQTT_USER // update this in secrets.h
#define MQTT_PASSWORD SECRET_MQTT_PASSWORD // update this in secrets.h
#define MQTT_CLIENT_NAME "BMSClient"

// -- Speed at which we talk to tesla BMBs --
// Possible settings are 631578,612500,617647,608695
#define BMS_BAUD        631578

// -- LED(s) configuration --
#define PIN             48    // GPIO to which the LED(s strip) is/are connected
#define NUMPIXELS       1    // Number of LEDs
#define BRIGHTNESS      15   // Adjust brightness (0-255)
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ------------------------- Global Vars -------------------------
Preferences preferences;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
BMSModuleManager bms(&server);
EEPROMSettings settings;
SerialConsole console;
uint32_t lastUpdate1;
uint32_t lastUpdate2;
uint32_t lastUpdate3;
String bmsJson;
float balanceVoltage = 3.95f;
float balanceHyst = 0.007f;
//const char* volt_str;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long rebootTime = 0;
int packsConfigured;
String systemName = "esp32-teslabms";
// -- Stuff below here needs to be configured in secrets.h --
String ftpServer = SECRET_FTP_SERVER_IP;
String ftpUser = SECRET_FTP_USER;
String ftpPassword = SECRET_FTP_PASSWORD;
String wifiSSID = SECRET_WIFI_SSID;
String wifiPassword = SECRET_WIFI_PASSWORD;
String webUsername = SECRET_WEBUI_USER;
String webPassword = SECRET_WEBUI_PASS;
String mqttServerIP = SECRET_MQTT_SERVER_IP;
String mqtt_Topic = SECRET_MQTT_TOPIC;
// -- Stuff above here needs to be configured in secrets.h --


// ------------------ MQTT ------------------
String mqttServer;
const int mqttPort = 1883;
int websocketsPort = 5081;
int mqttRetryCount = 0;
const int maxMqttRetries = 3;
String mqttTopic = mqtt_Topic;
const char* mqttClientName = MQTT_CLIENT_NAME;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;

// --- Function to manage LED behaviour ---
void setLED(uint8_t r, uint8_t g, uint8_t b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

// --- Function for fast blue LED flash
void flashBlue(int flashes = 5) {
    for (int i = 0; i < flashes; i++) {
        setLED(0, 0, 255);
        delay(100);
        setLED(0, 0, 0);
        delay(100);
    }
}

// --- Function for fast purple LED flash
void flashPurple(int flashes = 5) {
    for (int i = 0; i < flashes; i++) {
        setLED(128, 0, 255);
        delay(100);
        setLED(0, 0, 0);
        delay(100);
    }
}

// --- Function for fast Green LED flash
void flashGreen(int flashes = 5) {
    for (int i = 0; i < flashes; i++) {
        setLED(0, 255, 0);
        delay(100);
        setLED(0, 0, 0);
        delay(100);
    }
}

// --- Attempt to connect to mqtt; fail and continue if it doesn't work after a few retries ---
void connectMQTT() {
    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 3000;
    mqttRetryCount = 0; // Reset retry count
    while (mqttRetryCount < maxMqttRetries && (millis() - startAttemptTime) < timeout) {
//        Serial.println("Connecting to MQTT...");
        if (client.connect(
        (systemName + "_" + mqttClientName).c_str(), mqttUser, mqttPassword)) {
//            Serial.println("Connected to MQTT broker with authentication");
            mqttRetryCount = 0; // Reset on success
            return;
        } else {
            Serial.print("Failed with state ");
            Serial.println(client.state());
            mqttRetryCount++;
            delay(2000);
        }
    }
    Serial.println("Unable to connect to MQTT broker. Proceeding without MQTT.");
}

void loadSettings()
{
        Logger::console("Resetting to factory defaults");
        settings.version = EEPROM_VERSION;
        settings.checksum = 0;
        settings.canSpeed = 500000;
        settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
        settings.OverVSetpoint = 4.1f;
        settings.UnderVSetpoint = 2.75f;
        settings.OverTSetpoint = 65.0f;
        settings.UnderTSetpoint = -10.0f;
        settings.balanceVoltage = 3.95f;
        settings.balanceHyst = 0.007f;
        settings.logLevel = 1;
        
    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
}

/* - CAN bus code - not used - needs to be cleaned up in the future.
void initializeCAN()
{
    uint32_t id;
    CAN0.begin(settings.canSpeed);
    if (settings.batteryID < 0xF)
    {
        //Setup filter for direct access to our registered battery ID
        id = (0xBAul << 20) + (((uint32_t)settings.batteryID & 0xF) << 16);
        CAN0.setRXFilter(0, id, 0x1FFF0000ul, true);
        //Setup filter for request for all batteries to give summary data
        id = (0xBAul << 20) + (0xFul << 16);
        CAN0.setRXFilter(1, id, 0x1FFF0000ul, true);
    }
}
*/

void setup() 
{
    preferences.begin("settings", true); // "settings" is the namespace

    // Load saved settings
    mqttServer = preferences.getString("mqttServer", mqttServerIP);
    systemName = preferences.getString("systemName");
    wifiSSID = preferences.getString("wifiSSID", wifiSSID);
    wifiPassword = preferences.getString("wifiPassword", wifiPassword);
    balanceVoltage = preferences.getFloat("balanceVoltage", 3.95f);
    balanceHyst = preferences.getFloat("balanceHyst", 0.007f);
    packsConfigured = preferences.getInt("packsConfigured", 16);
    ftpPassword = preferences.getString("ftpPassword", ftpPassword);
    ftpUser = preferences.getString("ftpUser", ftpUser);
    ftpServer = preferences.getString("ftpServer", ftpServer);
    webUsername = preferences.getString("webUsername", webUsername);
    webPassword = preferences.getString("webPassword", webPassword);
    preferences.end();

// -- Initialise the LED and show red to indicate boot --
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();
    setLED(255, 0, 0);  // Solid RED on boot

/*
    if (!LittleFS.begin(true)) {
      Serial.println("An error occurred while mounting LittleFS");
    return;
}
*/

  delay(2000);  //For easy debugging. It takes a few seconds for USB to come up properly
  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");

    // Attempt to connect to the Wi-Fi network
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  Serial.println("Attempting to connect to Wi-Fi SSID: " + wifiSSID);

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 20000; // 20 seconds timeout

  while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime) < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected successfully.");
    Serial.println("IP address: " + WiFi.localIP().toString());
  } else {
    // If connection fails, start the AP for configuration
    Serial.println("Wi-Fi connection failed. Starting local AP...");
    WiFi.softAP("bms-esp", "12345678");
    Serial.println("AP started. Connect to 'bms-esp' to configure Wi-Fi settings.");
    Serial.println("IP address: " + WiFi.softAPIP().toString());
  }
  server.addHandler(&ws);
  Serial.println("WebSocket server started on port " + String(websocketsPort));

  SERIAL.begin(BMS_BAUD, SERIAL_8N1, BMB_RX_PIN, BMB_TX_PIN);
  SERIALCONSOLE.println("Started serial interface to BMS.");
  pinMode(BMB_FAULT_PIN, INPUT); // Setup FAULT hardware line as input
  Serial.println("Load EEPROM");
  loadSettings();
  Serial.println("Load custom settings");
  preferences.begin("settings", true); // "settings" is the namespace
//  settings.balanceVoltage = preferences.getFloat("balanceVoltage"), 3.95f);
//  settings.balanceHyst = preferences.getFloat("balanceHyst", 0.007f);
  settings.balanceVoltage = balanceVoltage;
  settings.balanceHyst    = balanceHyst;
  preferences.end();
  //  Serial.println("Initialize CAN");
  //  initializeCAN();
  Serial.println("System IO setup");
  systemIO.setup();
  Serial.println("Done.");
  Serial.println("Finding BMS Boards...");
  bms.findBoards();
  Serial.println("Done.");
  Serial.println("Renumbering board IDs...");
  bms.renumberBoardIDs();
  // This block is for printing info at bootup
  if (numFoundModules < packsConfigured)
        {
          Serial.println("Found " + String(numFoundModules) + " out of " + String(packsConfigured) + " packs. Restarting search.");
          Serial.println("BMB RX / TX pins are set to " + String(BMB_RX_PIN) + " / " + String(BMB_TX_PIN));
        }
  else  {
          Serial.println("Found all " + String(numFoundModules) + " packs! Search ended.");
        }

  //Logger::setLoglevel(Logger::Debug);

  lastUpdate1 = 0;
  lastUpdate2 = 0;
  lastUpdate3 = 0;

  Serial.println("BMS clear faults");
  bms.clearFaults();
  Serial.println("End of setup");
  Serial.println("Send ? line to get help. d to get detailed updates, p to get summary updates.");
  Serial.printf("Loaded balanceVoltage: %.2f\r\n", settings.balanceVoltage);
  Serial.printf("Loaded balanceHyst: %.3f\r\n", settings.balanceHyst);
  delay(1000);

    // Connect to MQTT broker
    client.setBufferSize(512);
    client.setServer(mqttServer.c_str(), mqttPort);
    //client.setCallback(callback);

    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
      connectMQTT();
    }

    // Define REST API endpoint
    server.on("/", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request){
        handleRoot(request);
    });

    server.on("/update", WebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
        request->redirect("/");
        handleUpdate(request);
    });

    server.on("/reboot", WebRequestMethod::HTTP_POST, [](AsyncWebServerRequest *request) {
      if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
        return request->requestAuthentication();
      }
      request->send(303, "text/plain", "Rebooting...");
      // Delay before reboot to allow response to finish
      // Use task to avoid blocking
      xTaskCreate([](void *param) {
          delay(1000);
          esp_restart();
      }, "RebootTask", 2048, nullptr, 1, nullptr);
  });

    server.on("/batterystats", HTTP_GET, [](AsyncWebServerRequest *request){
      if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
        return request->requestAuthentication();
      }
      String json = bms.buildJsonData();
      bms.handleBatteryStats(request, json);
  });

    server.on("/updatefw", HTTP_POST,
      [](AsyncWebServerRequest *request) {
        if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
        return request->requestAuthentication();
        }
        // Send an HTML page that redirects after 1 second
        String message = R"rawliteral(
          <html>
            <head>
              <meta http-equiv="refresh" content="1; url=/" />
            </head>
            <body>
              <h2>Update complete. Restarting...</h2>
            </body>
          </html>
        )rawliteral";
        request->send(200, "text/html", message);
      },
      [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!index) {
          Serial.printf("UploadStart: %s\n", filename.c_str());
          Update.begin(UPDATE_SIZE_UNKNOWN);
        }
        Update.write(data, len);
        if (final) {
          if (Update.end(true)) {
            Serial.printf("Success: %u bytes\n", index + len);
            delay(2000);
            esp_restart();
          } else {
            Update.printError(Serial);
          }
        }
      }
    );

    // Start the server
    Serial.println("Starting HTTP server...");
    server.begin();
    Serial.println("HTTP server started.");

    // OTA Setup
    ArduinoOTA
        .onStart([]() {
          String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
          Serial.println("Start updating " + type);
        })
        .onEnd([]() {
          Serial.println("End");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
          Serial.printf("Progress: %u%%\r", (progress * 100) / total);
        })
        .onError([](ota_error_t error) {
          Serial.printf("Error[%u]: ", error);
          if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
          else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
          else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
          else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
          else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("OTA Ready");
    //Serial.println("IP address: " + WiFi.localIP().toString());
    // Handle for case where the local AP is running and not in STA mode connected to a wifi AP.
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("IP address: " + WiFi.localIP().toString());
    } else {
        Serial.println("AP IP address: " + WiFi.softAPIP().toString());
    }
}

/*
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
*/

void loop() 
{
    console.loop(); // For interacting with the debug menu over serial

    if (millis() - lastUpdate3 >= 1000) { // 1-second tasks
      lastUpdate3 = millis();
      // Manage the LED state based on wi-fi connection
      if (WiFi.status() != WL_CONNECTED) {
        setLED(255, 70, 0);  // Solid Yellow - no Wi-Fi
      } else {
      }
}

    if (millis() - lastUpdate1 >= 3000) { // 3-second tasks
      lastUpdate1 = millis();
      
      if (WiFi.status() == WL_CONNECTED) {
      flashBlue(3);

      if (!client.connected()) {
          // Uncomment for additional assistance w/ MQTT - print state.
          /*
          Serial.println(client.state());
          Serial.println("MQTT appears to have disconnected!");
          */
          connectMQTT();
        }
        bms.getAllVoltTemp();
        bms.balanceCells();
        //String volt_str = bms.csvData();
        //client.publish(mqttTopic, volt_str.c_str());

        // Base MQTT topic for publishing
        //const char* baseTopic = "bms";

        // Publish individual cell and temperature data
      }
    }

    ArduinoOTA.handle(); // Handle OTA events

    if (millis() - lastUpdate2 >= 10000) { // 10-second tasks
        lastUpdate2 = millis();
        // -- debug statistics - keep commented if not needed to prevent noise in serial log -- //        
        /*
        printChipTemp();
        Serial.println("");
        Serial.print("Free heap: ");
        Serial.print(ESP.getFreeHeap() / 1024);
        Serial.println(" kB");
        */

    if (WiFi.status() == WL_CONNECTED) {
      flashBlue(3);

        if (numFoundModules == packsConfigured){
          bms.publishIndividualData(client, "homeassistant/sensor/bms/", systemName);
          bmsJson = bms.buildJsonData();
          bms.sendBatteryStats(systemName, ftpServer, ftpUser, ftpPassword, bmsJson);
          bms.broadcastBatteryStats(&ws, bmsJson);
        }

      }

        if (numFoundModules < packsConfigured){
          bms.findBoards();
          bms.renumberBoardIDs();
          if (numFoundModules < packsConfigured){
            Serial.println("Found " + String(numFoundModules) + " out of " + String(packsConfigured) + " packs. Restarting search.");
            Serial.println("Check connections - BMB RX / TX pins are set to " + String(BMB_RX_PIN) + " / " + String(BMB_TX_PIN));
            flashPurple(3);
          }
          else {
            Serial.println("Found all " + String(numFoundModules) + " packs! Search ended.");
            flashGreen(3);
          }
        }
    }
}

void handleRoot(AsyncWebServerRequest *request) {

  // Authentication
  if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
      return request->requestAuthentication();
  }

  // Frontend
  String html = "<html><head><title>BMS - " + systemName + "</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { background-color: #181818; color: #f5f5f5; font-family: Arial, sans-serif; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; height: 100vh; }";
  html += "h1 { color: #f5f5f5; text-align: center; }";
  html += "input[type='text'], input[type='password'] { background-color: #333333; color: #f5f5f5; border: 1px solid #444444; padding: 8px; margin: 5px 0; border-radius: 4px; width: 250px; }";
  html += "input[type='text']:focus, input[type='password']:focus { border-color: #555555; outline: none; }";
  html += "input[type='submit'] { background-color: #444444; color: #f5f5f5; border: none; padding: 10px 20px; cursor: pointer; border-radius: 4px; width: 100%; margin-top: 10px; }";
  html += "ul { list-style-type: none; padding: 0; }";
  html += "li { display: flex; justify-content: space-between; margin: 10px 0; }";
  html += "li span { display: inline-block; align-content: center; }";
  html += "li .subject { font-weight: bold; }";
  html += ".data { text-align: right; }";
  html += "input[type='submit']:hover { background-color: #555555; }";
  html += ".container { background-color: #222222; padding: 20px; border-radius: 8px; width: 100%; max-width: 400px; }";
  html += "</style>";

  html += "</head><body>";

  html += "<div class='container'>";
  html += "<h1>Tesla Module Balancer</h1>";
  html += "<ul>";
  html += "<li><span class='subject'>System Name:</span> <span class='data'>" + systemName + "</span></li>";
  html += "<li><span class='subject'>MQTT Server:</span> <span class='data'>" + mqttServer + "</span></li>";
  html += "<li><span class='subject'>Pack Count:</span> <span class='data'>" + String(numFoundModules) + "</span></li>";
  html += "<li><span class='subject'>Cell Low/High:</span> <span class='data'>" + String(lowestCellVolt, 3) + " | " + String(highestCellVolt, 3) + "</span></li>";
  html += "<li><span class='subject'>Temp Low/High:</span> <span class='data'>" + String(lowestPackTemp) + " | " + String(highestPackTemp) + "</span></li>";
  html += "<li><span class='subject'><a href='/batterystats' style='color: #f5f5f5;'>Battery Stats</a></span></li>";
  html += "</ul>";

  html += "<h1>Settings</h1>";
  html += "<form action='/update' method='POST'>";
  html += "<ul>";
  html += "<li><span class='subject'>MQTT Server:</span><span class='data'><input type='text' name='mqttServer' value='" + mqttServer + "'></span></li>";
  html += "<li><span class='subject'>System Name:</span><span class='data'><input type='text' name='systemName' value='" + systemName + "'></span></li>";
  html += "<li><span class='subject'>Packs Configured:</span><span class='data'><input type='text' name='packsConfigured' value='" + String(packsConfigured) + "'></span></li>";
  html += "<li><span class='subject'>Balance Voltage:</span><span class='data'><input type='text' name='balanceVoltage' value='" + String(balanceVoltage, 2) + "'></span></li>";
  html += "<li><span class='subject'>Balance Hysteresis:</span><span class='data'><input type='text' name='balanceHyst' value='" + String(balanceHyst, 3) + "'></span></li>";
  html += "<li><span class='subject'>Wi-Fi SSID:</span><span class='data'><input type='text' name='wifiSSID' value='" + wifiSSID + "'></span></li>";
  html += "<li><span class='subject'>Wi-Fi Password:</span><span class='data'><input type='password' name='wifiPassword' value='" + wifiPassword + "'></span></li>";
  html += "<li><span class='subject'>FTP Server:</span><span class='data'><input type='text' name='ftpServer' value='" + ftpServer + "'></span></li>";
  html += "<li><span class='subject'>FTP Username:</span><span class='data'><input type='text' name='ftpUser' value='" + ftpUser + "'></span></li>";
  html += "<li><span class='subject'>FTP Password:</span><span class='data'><input type='password' name='ftpPassword' value='" + ftpPassword + "'></span></li>";
  // WebUI Auth
  html += "<li><span class='subject'>WebUI Username:</span><span class='data'><input type='text' name='webUsername' value='" + webUsername + "'></span></li>";
  html += "<li><span class='subject'>WebUI Password:</span><span class='data'><input type='password' name='webPassword' value='" + webPassword + "'></span></li>";
  html += "<input type='submit' value='Update Settings'><br />";
  html += "</form>";
  html += "<form action='/reboot' method='POST'>";
  html += "<input type='submit' value='Reboot'>";
  html += "</form>";
  html += "</ul>";
  html += "<h1>ESP32 OTA Update</h1>";
  html += "<form method=\"POST\" action=\"/updatefw\" enctype=\"multipart/form-data\">";
  html += "<input type=\"file\" name=\"firmware\">";
  html += "<button type=\"submit\">Upload Firmware</button>";
  html += "</form>";
  html += "</div>";

  html += "</body></html>";

  request->send(200, "text/html", html);
}

// Handle the form submission
void handleUpdate(AsyncWebServerRequest *request) {
  // Authentication
  if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
      return request->requestAuthentication();
  }

  // Frontend stuff
  bool wifiChanged = false;
  if (request->hasParam("mqttServer", true)) {
    mqttServer = request->getParam("mqttServer", true)->value();
    preferences.begin("settings", false);
    preferences.putString("mqttServer", mqttServer);
    preferences.end();
  }

  if (request->hasParam("systemName", true)) {
    systemName = request->getParam("systemName", true)->value();
    preferences.begin("settings", false);
    preferences.putString("systemName", systemName);
    preferences.end();
  }

  if (request->hasParam("wifiSSID", true)) {
    String newSSID = request->getParam("wifiSSID", true)->value();
    if (newSSID != wifiSSID) {
      wifiSSID = newSSID;
      preferences.begin("settings", false);
      preferences.putString("wifiSSID", wifiSSID);
      preferences.end();
      wifiChanged = true;
    }
  }

  if (request->hasParam("wifiPassword", true)) {
    String newPassword = request->getParam("wifiPassword", true)->value();
    if (newPassword != wifiPassword) {
      wifiPassword = newPassword;
      preferences.begin("settings", false);
      preferences.putString("wifiPassword", wifiPassword);
      preferences.end();
    }
  }

  if (request->hasParam("ftpServer", true)) {
    ftpServer = request->getParam("ftpServer", true)->value();
    preferences.begin("settings", false);
    preferences.putString("ftpServer", ftpServer);
    preferences.end();
  }

  if (request->hasParam("ftpUser", true)) {
    String newUser = request->getParam("ftpUser", true)->value();
    if (newUser != ftpUser) {
      ftpUser = newUser;
      preferences.begin("settings", false);
      preferences.putString("ftpUser", ftpUser);
      preferences.end();
    }
  }

  if (request->hasParam("ftpPassword", true)) {
    String newFtpPass = request->getParam("ftpPassword", true)->value();
    if (newFtpPass != ftpPassword) {
      ftpPassword = newFtpPass;
      preferences.begin("settings", false);
      preferences.putString("ftpPassword", ftpPassword);
      preferences.end();
    }
  }

  if (request->hasParam("webUsername", true)) {
      webUsername = request->getParam("webUsername", true)->value();
      preferences.begin("settings", false);
      preferences.putString("webUsername", webUsername);
      preferences.end();
  }

  if (request->hasParam("webPassword", true)) {
      webPassword = request->getParam("webPassword", true)->value();
      preferences.begin("settings", false);
      preferences.putString("webPassword", webPassword);
      preferences.end();
  }

  if (request->hasParam("packsConfigured", true)) {
    String packsStr = request->getParam("packsConfigured", true)->value();
    packsConfigured = packsStr.toInt();
    preferences.begin("settings", false);
    preferences.putInt("packsConfigured", packsConfigured);
    preferences.end();
  }

  if (request->hasParam("balanceHyst", true)) {
    float newHyst = request->getParam("balanceHyst", true)->value().toFloat();
    if (newHyst != balanceHyst) {
      balanceHyst = newHyst;
      preferences.begin("settings", false);
      preferences.putFloat("balanceHyst", balanceHyst);
      preferences.end();
      settings.balanceHyst = balanceHyst;
      Serial.printf("Set balanceHyst: %.3f\r\n", settings.balanceHyst);
    }
  }

  if (request->hasParam("balanceVoltage", true)) {
    float newVoltage = request->getParam("balanceVoltage", true)->value().toFloat();
    if (newVoltage != balanceVoltage) {
      balanceVoltage = newVoltage;
      preferences.begin("settings", false);
      preferences.putFloat("balanceVoltage", balanceVoltage);
      preferences.end();
      settings.balanceVoltage = balanceVoltage;
      Serial.printf("Set balanceVoltage: %.2f\r\n", settings.balanceVoltage);
    }
  }

  if (wifiChanged) {
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Reconnecting to Wi-Fi...");
    }
  }

  // Send confirmation message - is this needed?
  //server.send(200, "text/html", "<html><body { background-color: #181818; color: #f5f5f5; font-family: Arial, sans-serif; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; height: 100vh; }><h1>Settings Updated</h1><p>Reconnecting to Wi-Fi...</p></body></html>");
  delay(2000);  // Allow some time for Wi-Fi to reconnect
  // esp_restart();  // Restart ESP to apply changes - doesn't appear to be necessary, most changes apply fine without restar
}