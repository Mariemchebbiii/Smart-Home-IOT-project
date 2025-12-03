#include <Arduino.h> // Must be included first

// --- Library Includes ---
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <time.h>
#include <FS.h>           // Filesystem for config/web files (SPIFFS)
#include <ArduinoJson.h>  // For config file parsing (Ensure v6 in platformio.ini)
#include <PubSubClient.h> // MQTT
#include <Ticker.h>       // For scheduled tasks
#include <EEPROM.h>       // For potential future config / WiFiManager uses
#include <ESP8266HTTPClient.h> // Not explicitly used in this logic, but good to keep if needed later
#include <NTPClient.h>    // Network Time Protocol
#include <WiFiUdp.h>      // UDP for NTP
// #include <Adafruit_BMP280.h>  // Optional pressure sensor (Keep if needed)
#include <Adafruit_Sensor.h> // Required by DHT and other Adafruit sensors

// --- Device identification (Can be overridden by build_flags in platformio.ini) ---
#ifndef DEVICE_NAME
#define DEVICE_NAME "mariem & sarra SmartDisplay"
#endif
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "2.1.2-NetDiag-PIO" // Default if not set in platformio.ini
#endif

// --- Hardware configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1   // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDRESS  0x3C // Default I2C address for SSD1306
#define DHTPIN        14   // D5 on NodeMCU
#define DHTTYPE       DHT22
#define BUZZER_PIN    15   // D8 on NodeMCU
#define SWITCH_RING   12   // D6 on NodeMCU
#define SWITCH_PASS   2    // D4 on NodeMCU (often marked BUILTIN_LED on ESP-12)
#define LED_PIN       LED_BUILTIN // Use standard built-in LED definition
#define LED_D0_PIN    16   // D0 on NodeMCU/Wemos (GPIO16)
#define LED_D3_PIN    0    // D3 on NodeMCU/Wemos (GPIO0) - WARNING: Boot mode sensitive!
#define FORCE_WIFI_FORGET_ON_BOOT // *** ADD THIS LINE ***


// --- Enable/disable features (Can be overridden by build_flags in platformio.ini) ---
// If flags are defined in platformio.ini, these lines don't have effect.
// If flags are *not* defined in platformio.ini, uncommenting here enables the feature.
#ifndef ENABLE_OTA
// #define ENABLE_OTA
#endif
#ifndef ENABLE_MQTT
// #define ENABLE_MQTT
#endif
#ifndef ENABLE_DATA_LOGGING
// #define ENABLE_DATA_LOGGING
#endif
// #define ENABLE_DEEP_SLEEP // Only uncomment AND add build flag if actually using deep sleep

// --- Constants ---
const int SENSOR_UPDATE_INTERVAL_MS = 5000;      // Update sensors every 5 seconds
const int DISPLAY_UPDATE_INTERVAL_MS = 250;      // Refresh display ~4 times/second
const int DATA_LOG_INTERVAL_S = 300;             // Log data every 5 minutes (300 seconds)
const int MQTT_PUBLISH_INTERVAL_MS = 60000;      // Publish to MQTT every minute
const int NTP_UPDATE_INTERVAL_MS = 3600000;      // Update time from NTP every hour
const int HTTP_PORT = 80;
const int MAX_FAILED_ATTEMPTS = 3;
const int LOCKOUT_DURATION_MS = 300000;          // 5 minutes lockout
const int HISTORY_SIZE = 144;                    // For ENABLE_DATA_LOGGING: 12 hours of data at 5-min intervals

// --- Server Definitions (Can be overridden by build_flags in platformio.ini) ---
#ifndef NTP_SERVER
#define NTP_SERVER "pool.ntp.org" // Default if not set in platformio.ini
#endif
#ifndef MQTT_SERVER
#define MQTT_SERVER "broker.mqtt-dashboard.com" // Default if not set in platformio.ini
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883 // Standard MQTT port
#endif

// --- Global Objects ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHTPIN, DHTTYPE);
ESP8266WebServer server(HTTP_PORT);
ESP8266HTTPUpdateServer httpUpdater;
WiFiUDP ntpUDP;
// Corrected: Use defined NTP_SERVER and set interval based on constant
// NTPClient internal update interval is less relevant if we force update in our logic
NTPClient timeClient(ntpUDP, NTP_SERVER, 0, NTP_UPDATE_INTERVAL_MS);
WiFiClient espClient; // For MQTT and HTTPClient if used
PubSubClient mqttClient(espClient);
Ticker sensorTicker;    // Ticker to *trigger* sensor reads
Ticker displayTicker;   // Ticker to *trigger* display updates
Ticker dataLogTicker;   // Ticker to *trigger* data logging
Ticker blinkTicker;     // Ticker for blinking LED in AP mode

// --- Volatile Flags for Ticker Callbacks ---
volatile bool timeToUpdateSensors = false;
volatile bool timeToUpdateDisplay = false;
volatile bool timeToLogData = false;

// --- Data Logging Struct and Variables ---
#ifdef ENABLE_DATA_LOGGING
struct DataPoint {
    float temperature;
    float humidity;
    time_t timestamp;
};
DataPoint history[HISTORY_SIZE];
int historyIndex = 0;
bool historyWrapped = false;
#endif // ENABLE_DATA_LOGGING

// --- Security Variables ---
char devicePassword[64] = "1234"; // Default password, overwritten by config
unsigned long failedAttempts = 0;
bool lockout = false;
unsigned long lockoutEndTime = 0;

// --- MQTT State ---
bool mqttConnected = false;
unsigned long lastMqttAttempt = 0;
const long MQTT_RECONNECT_INTERVAL = 5000; // Interval between MQTT reconnect attempts

// --- Display and Operational State ---
enum DisplayMode { NORMAL, SMILE, SAD, RAGE, COOL, SURPRISED, SLEEPY, WEATHER };
DisplayMode currentMode = NORMAL;
bool ringActive = false; // Doorbell state
float currentTemp = NAN; // Initialize as Not-a-Number
float currentHumidity = NAN;
float temperatureHistory[24]; // Store recent interval readings for graph
float humidityHistory[24];
int historyPointer = 0; // For the simple 24-point history used in drawGraph
unsigned long lastSensorUpdate = 0; // Keep track of when sensors were last *successfully* read
unsigned long lastMqttPublish = 0;
unsigned long lastNtpUpdate = 0;
char timeString[20] = "--:--:--"; // Default time string
char dateString[20] = "--/--/----"; // Default date string
int animationFrame = 0;
bool configMode = false;
bool ledD0State = false; // false = OFF, true = ON
bool ledD3State = false; // false = OFF, true = ON // Flag to indicate if WiFiManager AP is active

// --- State for quieter NTP failure logging ---
static unsigned long lastNtpFailLogTime = 0;
const unsigned long NTP_FAIL_LOG_INTERVAL = 60000; // Log NTP failure max once per minute

// --- Function Prototypes ---
void setupDisplay();
void setupWiFi();
void setupOTA();
void setupServer();
void setupMQTT();
void setupNTP();
void saveConfig();
void loadConfig();
void performSensorUpdate();   // Renamed from updateSensors
void performDisplayUpdate();  // Renamed from updateDisplay
void performDataLog();        // Renamed from logSensorData
void handleRoot();
void handleAPIStatus();
void handleAPIHistory();
void handleSubmit();
void handleReset();
void handleUpdate();
void handleModeChange(DisplayMode newMode);
void handleNotFound();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishSensorData();
void reconnectMQTT();
void drawNormal();
void drawAnimatedFace(const char* face);
void drawWeather();
void drawGraph();
void drawProgressBar(int x, int y, int width, int height, int progress);
void playTone(int frequency, int duration); // Simple tone
void playMelody(const int melody[], const int durations[], int size); // Sequence of tones
void blinkLED(int times);
void goToSleep(int sleepTimeSeconds); // Only useful if ENABLE_DEEP_SLEEP is defined and used
void rebootDevice();
void factoryReset();
void checkButtons();
void handleToggleLedD0();
void handleToggleLedD3();
bool checkPassword(String inputPassword);
String getContentType(String filename);
bool handleFileRead(String path);
String generateReport();
String formatUptime();
String getModeName(DisplayMode mode);



// --- HTML/CSS/JS Content (Stored in PROGMEM) ---
const char MAIN_CSS[] PROGMEM = R"=====(
* { box-sizing: border-box; font-family: 'Arial', sans-serif; }
body { margin: 0; padding: 20px; background: #f5f5f5; color: #333; }
.container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; box-shadow: 0 0 10px rgba(0,0,0,0.1); border-radius: 8px; }
h1, h2, h3 { color: #2c3e50; margin-top: 0; }
h1 { text-align: center; margin-bottom: 20px; }
.card { background: #fff; border: 1px solid #e0e0e0; border-radius: 5px; padding: 15px; margin-bottom: 20px; }
.readings { display: flex; justify-content: space-around; margin-bottom: 10px; text-align: center; }
.reading { flex: 1; padding: 0 10px; }
.reading h3 { font-size: 1em; margin-bottom: 5px; color: #34495e; }
.value { font-size: 1.8em; font-weight: bold; color: #2980b9; }
.temp .value { color: #e74c3c; }
.humid .value { color: #3498db; }
.controls { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 10px; justify-content: center; }
button, input[type='password'] { padding: 10px 15px; font-size: 1em; border: none; border-radius: 4px; cursor: pointer; background: #3498db; color: white; transition: background 0.3s ease; }
button:hover { background: #2980b9; }
button.active { background: #2ecc71; }
button.active:hover { background: #27ae60; }
input[type='password'] { flex-grow: 1; margin-right: 10px; background: #ecf0f1; color: #333; border: 1px solid #bdc3c7; cursor: text; }
#password-form { display: flex; align-items: center; }
#password-message { color: #e74c3c; font-size: 0.9em; margin-top: 5px; height: 1.2em; }
.mqtt-status { position: fixed; bottom: 5px; right: 10px; padding: 5px 10px; border-radius: 4px; font-size: 0.8em; color: white; z-index: 10; }
.mqtt-status.connected { background-color: #2ecc71; }
.mqtt-status.disconnected { background-color: #e74c3c; }
.footer { text-align: center; margin-top: 20px; font-size: 0.8em; color: #7f8c8d; }
.graph-container { height: 200px; position: relative; }
canvas { max-width: 100%; }
table { width: 100%; border-collapse: collapse; font-size: 0.9em; }
table, th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
th { background-color: #f2f2f2; }
@media (max-width: 600px) { .readings { flex-direction: column; } .reading { margin-bottom: 15px; } .controls { flex-direction: column; } button, input[type='password'] { width: 100%; } #password-form { flex-direction: column; gap: 10px; } input[type='password'] { margin-right: 0; } }
)=====";

const char MAIN_JS[] PROGMEM = R"=====(
let historyChart;
let statusInterval;

function fetchAndUpdateStatus() {
  fetch('/api/status')
    .then(response => response.json())
    .then(data => {
      if (data.temperature !== null) document.getElementById('temperature').textContent = `${data.temperature.toFixed(1)}°C`; else document.getElementById('temperature').textContent = '--.-°C';
      if (data.humidity !== null) document.getElementById('humidity').textContent = `${data.humidity.toFixed(1)}%`; else document.getElementById('humidity').textContent = '--.-%';
      document.getElementById('time').textContent = data.time;
      document.getElementById('uptime').textContent = data.uptime;

      const modeButtons = document.querySelectorAll('.controls button[onclick^="location.href"]');
      modeButtons.forEach(btn => {
          const btnMode = btn.getAttribute('onclick').match(/\/(\w+)'/)[1];
          if (btnMode.toLowerCase() === data.mode.toLowerCase()) {
              btn.classList.add('active');
          } else {
              btn.classList.remove('active');
          }
      });

      const mqttStatus = document.getElementById('mqtt-status');
      mqttStatus.textContent = `MQTT: ${data.mqtt_connected ? 'Connected' : (data.ip_address !== 'Offline' ? 'Connecting' : 'Offline')}`;
      mqttStatus.className = `mqtt-status ${data.mqtt_connected ? 'connected' : 'disconnected'}`;

      document.querySelector('.footer p').textContent = `Smart Display v${data.version} | IP: ${data.ip_address} | RSSI: ${data.rssi} dBm | Heap: ${data.heap_free}`;

    }).catch(error => console.error('Error fetching status:', error));
}

function fetchAndDrawChart() {
    fetch('/api/history')
        .then(response => response.json())
        .then(data => {
            const ctx = document.getElementById('history-chart').getContext('2d');
            const chartData = {
                labels: data.labels || [], // Use labels from server if available, otherwise empty
                datasets: [
                    {
                        label: 'Temperature (°C)',
                        data: data.temperature || [],
                        borderColor: 'rgb(231, 76, 60)',
                        backgroundColor: 'rgba(231, 76, 60, 0.1)',
                        yAxisID: 'yT',
                        tension: 0.1,
                        spanGaps: true // Connect lines over null data points
                    },
                    {
                        label: 'Humidity (%)',
                        data: data.humidity || [],
                        borderColor: 'rgb(52, 152, 219)',
                        backgroundColor: 'rgba(52, 152, 219, 0.1)',
                        yAxisID: 'yH',
                        tension: 0.1,
                        spanGaps: true // Connect lines over null data points
                    }
                ]
            };

            const options = {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: {
                       title: { display: true, text: 'History Index' } // Label depends on what data.labels contains
                    },
                    yT: {
                       type: 'linear',
                       display: true,
                       position: 'left',
                       title: { display: true, text: 'Temperature (°C)' },
                       suggestedMin: 10, // Optional: Set reasonable min/max or let Chart.js auto-scale
                       suggestedMax: 40
                    },
                    yH: {
                       type: 'linear',
                       display: true,
                       position: 'right',
                       title: { display: true, text: 'Humidity (%)' },
                       suggestedMin: 0,
                       suggestedMax: 100,
                       grid: { drawOnChartArea: false } // only draw axis and ticks
                    }
                },
                 plugins: {
                     tooltip: { mode: 'index', intersect: false },
                     title: { display: true, text: 'Recent Sensor Readings' }
                 },
                 interaction: {
                    mode: 'nearest',
                    axis: 'x',
                    intersect: false
                 }
            };

            if (historyChart) {
                 historyChart.data = chartData;
                 historyChart.options = options; // Re-apply options if needed
                 historyChart.update();
            } else {
                historyChart = new Chart(ctx, { type: 'line', data: chartData, options: options });
            }
        })
        .catch(error => console.error('Error fetching history:', error));
}

document.addEventListener('DOMContentLoaded', function() {
    fetchAndUpdateStatus(); // Initial fetch
    fetchAndDrawChart();    // Initial fetch
    statusInterval = setInterval(fetchAndUpdateStatus, 5000); // Update status every 5s
    // Update chart less frequently if desired, e.g., every minute
    setInterval(fetchAndDrawChart, 60000);

    const passwordForm = document.getElementById('password-form');
    const passwordInput = document.getElementById('password');
    const messageDiv = document.getElementById('password-message');

    if (passwordForm) {
        passwordForm.addEventListener('submit', function(e) {
            e.preventDefault();
            const password = passwordInput.value;
            const formData = new FormData();
            formData.append('pwd', password);

            messageDiv.textContent = 'Checking...';

            fetch('/submit', {
                method: 'POST',
                body: formData
            })
            .then(response => response.text().then(text => ({ ok: response.ok, status: response.status, text })))
            .then(({ ok, status, text }) => {
                if (ok) {
                    messageDiv.textContent = 'Access Granted!';
                    messageDiv.style.color = '#2ecc71';
                } else {
                    messageDiv.textContent = text || `Access Denied (Error ${status})`;
                    messageDiv.style.color = '#e74c3c';
                }
                 passwordInput.value = ''; // Clear password field
                 // Optionally clear the message after a few seconds
                 setTimeout(() => { messageDiv.textContent = ''; }, 5000);
            })
            .catch(error => {
                console.error('Password submit error:', error);
                messageDiv.textContent = 'Error submitting password.';
                 messageDiv.style.color = '#e74c3c';
                 passwordInput.value = ''; // Clear password field
                 setTimeout(() => { messageDiv.textContent = ''; }, 5000);
            });
        });
    }
});
)=====";


// --- Melodies and Faces ---
const int UNLOCK_MELODY[] = {2000, 2500, 3000, 3500, 4000};
const int UNLOCK_DURATIONS[] = {50, 50, 50, 50, 100};
const int LOCK_MELODY[] = {4000, 3500, 3000, 2500, 2000};
const int LOCK_DURATIONS[] = {50, 50, 50, 50, 100};
const int ALERT_MELODY[] = {4000, 1000, 4000, 1000};
const int ALERT_DURATIONS[] = {100, 100, 100, 100};

const char* smileFaces[] = {":)", ":))", ":-)"};
const char* sadFaces[] = {":(", ":((", ":-("};
const char* rageFaces[] = {">:(", ">:((", ">:-("};
const char* coolFaces[] = {"B)", "B-)", "B))"};
const char* surprisedFaces[] = {":o", ":O", ":-O"};
const char* sleepyFaces[] = {"-_-", "zzZ", "-_-z"};

// --- Weather Bitmaps (PROGMEM) ---
const uint8_t PROGMEM sunny_bmp[] = { 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x03, 0x80, 0x00, 0x03, 0x07, 0xC0, 0xA0, 0x02, 0x18, 0x30, 0x80, 0x06, 0x20, 0x08, 0xC0, 0x04, 0x40, 0x04, 0x40, 0x04, 0x80, 0x02, 0x20, 0x0C, 0x80, 0x02, 0x20, 0x04, 0x80, 0x02, 0x20, 0x04, 0x40, 0x04, 0x40, 0x06, 0x20, 0x08, 0xC0, 0x02, 0x10, 0x10, 0x80, 0x03, 0x0F, 0xE0, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t PROGMEM cloud_bmp[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x08, 0x40, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x40, 0x08, 0x00, 0x00, 0x40, 0x04, 0x00, 0x00, 0x40, 0x04, 0x00, 0x07, 0xC0, 0x04, 0x00, 0x08, 0x40, 0x04, 0x00, 0x10, 0x20, 0x04, 0x00, 0x10, 0x10, 0x04, 0x00, 0x20, 0x08, 0x04, 0x00, 0x20, 0x08, 0x04, 0x00, 0x20, 0x08, 0x04, 0x00, 0x10, 0x10, 0x08, 0x00, 0x10, 0x20, 0x10, 0x00, 0x0F, 0xC0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t PROGMEM rainy_bmp[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x08, 0x40, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


// ========================================================================
//   SETUP FUNCTION
// ========================================================================
void setup() {
  // --- Basic Initialization ---
  #ifndef FORCE_WIFI_FORGET_ON_BOOT
  // This will cause a compile error if the define is MISSING
  // If it compiles, the define IS active.
  #error "BUILD ERROR: FORCE_WIFI_FORGET_ON_BOOT is NOT defined! Check your code."
#endif
  Serial.begin(115200); // Ensure this matches monitor_speed in platformio.ini
  Serial.println(F("\n\n=== Advanced Smart Display Booting (PlatformIO) ==="));
  Serial.println(F("Version: " FIRMWARE_VERSION));
  Serial.print(F("Using NTP Server: ")); Serial.println(NTP_SERVER); // Log selected server
  Serial.print(F("Using MQTT Broker: ")); Serial.print(MQTT_SERVER); Serial.print(":"); Serial.println(MQTT_PORT); // Log selected broker

  // Hardware Pins
  pinMode(LED_D0_PIN, OUTPUT);
  digitalWrite(LED_D0_PIN, LOW); // Start with LED D0 OFF
  pinMode(LED_D3_PIN, OUTPUT);
  digitalWrite(LED_D3_PIN, LOW); // Start with LED D3 OFF
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially
  pinMode(SWITCH_RING, INPUT_PULLUP);
  pinMode(SWITCH_PASS, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Turn LED ON during setup

  // --- Filesystem & Config ---
  Serial.println(F("Initializing EEPROM..."));
  EEPROM.begin(512); // Small size needed, WiFiManager might use it
  Serial.println(F("Initializing SPIFFS..."));
  if (SPIFFS.begin()) {
    Serial.println(F("SPIFFS mounted successfully."));
    loadConfig(); // Load configuration after mounting FS
  } else {
    Serial.println(F("ERROR: Failed to mount SPIFFS! Check flash config or format."));
    // Handle failure - maybe try formatting?
    // Serial.println(F("Formatting SPIFFS...")); SPIFFS.format(); ESP.restart();
  }
  // --- Hardware Components ---
  setupDisplay(); // Initialize display early for boot messages

  Serial.println(F("Initializing DHT Sensor..."));
  dht.begin();
  Serial.println(F("DHT initialized."));

  // --- Network Connection ---
  setupWiFi(); // Handle WiFi connection and AP portal

  // --- Network Services (Only if WiFi is Connected) ---
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("Proceeding with network services initialization..."));

    Serial.println(F("Initializing NTP Client..."));
    setupNTP();
    delay(50); // Small delay between network initializations

    Serial.println(F("Initializing Web Server..."));
    setupServer();
    delay(50);

    #ifdef ENABLE_OTA
      Serial.println(F("Initializing OTA Update Service..."));
      setupOTA();
      delay(50);
    #else
       Serial.println(F("OTA is disabled in configuration."));
    #endif // ENABLE_OTA

    #ifdef ENABLE_MQTT
      Serial.println(F("Initializing MQTT Client..."));
      setupMQTT(); // Configures MQTT client
      delay(50);
    #else
      Serial.println(F("MQTT is disabled in configuration."));
    #endif // ENABLE_MQTT

  } else {
    Serial.println(F("WARNING: WiFi not connected after setupWiFi(). Skipping network services. Check credentials/router."));
  }

  // --- Timers & Final Steps ---
  Serial.println(F("Attaching Timers (Tickers)..."));

  // Attach tickers to simply set flags
  sensorTicker.attach_ms(SENSOR_UPDATE_INTERVAL_MS, [](){ timeToUpdateSensors = true; });
  displayTicker.attach_ms(DISPLAY_UPDATE_INTERVAL_MS, [](){ timeToUpdateDisplay = true; });
  #ifdef ENABLE_DATA_LOGGING
    Serial.println(F("Attaching Data Logging Timer..."));
    dataLogTicker.attach(DATA_LOG_INTERVAL_S, [](){ timeToLogData = true; });
  #else
    Serial.println(F("Data Logging is disabled in configuration."));
  #endif // ENABLE_DATA_LOGGING
  Serial.println(F("Tickers attached."));

  Serial.println(F("------------------------------------"));
  Serial.println(F("Setup Complete! Entering main loop."));
  if(WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Current IP Address: ")); Serial.println(WiFi.localIP());
  } else {
     Serial.println(F("Device is OFFLINE."));
  }
  Serial.println(F("------------------------------------"));

  digitalWrite(LED_PIN, LOW); // Turn LED OFF, setup complete
  playMelody(UNLOCK_MELODY, UNLOCK_DURATIONS, 5); // Play startup melody
} // END OF SETUP

// ========================================================================
//   MAIN LOOP
// ========================================================================
void loop() {
  // --- Handle Core Network Services ---
  if (WiFi.status() == WL_CONNECTED) {
      server.handleClient(); // Process incoming HTTP requests

      #ifdef ENABLE_OTA
        ArduinoOTA.handle();   // Process OTA updates
      #endif

      #ifdef ENABLE_MQTT
        if (!mqttClient.connected()) {
           reconnectMQTT(); // Try to reconnect if disconnected
        }
        // Must be called regularly to process MQTT messages and keepalive
        if (mqttClient.connected()) {
             mqttClient.loop();
        }
      #endif

      // Handle MDNS separately only if connected
       MDNS.update();
  } else {
     // If WiFi is disconnected...
     // Optionally handle AP mode blink here if configMode is true outside the setup callback context
     if (configMode && blinkTicker.active() == false ) { // Check if ticker isn't already running
         blinkTicker.attach(0.2, []() { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); });
     }

     // If we were connected to MQTT and lose WiFi, update state
     if (mqttConnected) {
         mqttConnected = false;
         Serial.println("WiFi Disconnected - MQTT state set to false");
     }
  }

  // --- Check Flags Set by Tickers ---
  // Perform actions triggered by timers in the main loop context

  if (timeToUpdateSensors) {
    timeToUpdateSensors = false; // Reset flag immediately
    performSensorUpdate();       // Call the actual worker function
  }

  if (timeToUpdateDisplay) {
    timeToUpdateDisplay = false; // Reset flag
    performDisplayUpdate();      // Call the actual worker function
  }

  #ifdef ENABLE_DATA_LOGGING
    if (timeToLogData) {
      timeToLogData = false;    // Reset flag
      performDataLog();         // Call the actual worker function
    }
  #endif

  // --- Handle Other Loop Tasks ---
  checkButtons(); // Check physical button states

  // Manage lockout timer
  if (lockout && millis() >= lockoutEndTime) {
    lockout = false;
    failedAttempts = 0;
    Serial.println(F("Lockout period ended"));
  }

  // --- Yield for Background Tasks ---
  // Important for ESP8266 stability, especially with networking
  yield();

} // END OF LOOP


// ========================================================================
//   SETUP HELPER FUNCTIONS
// ========================================================================

void setupDisplay() {
  Serial.println(F("Initializing Display (SSD1306)..."));
  // Initiate the Wire library for I2C
  // Parameters: SDA, SCL (optional, uses default pins if not specified for the board)
  Wire.begin(); // Use default SDA/SCL pins for the board (D2/D1 on NodeMCU usually)

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("ERROR: SSD1306 allocation failed! Check Connections/Address. Halting."));
    while (true) { blinkLED(1); delay(500); } // Halt on display failure
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.cp437(true); // Use full 256 char 'Code Page 437' font
  display.setCursor(0,0);
  display.println(F("Display Init OK"));
  display.display();
  Serial.println(F("Display initialized."));
  delay(500); // Show message briefly
}

// ========================================================================
//   WiFi Setup Function (with Optional Credential Reset)
// ========================================================================
void setupWiFi() {
  Serial.println(F("--- Starting WiFi Setup ---"));
  configMode = false; // Ensure flag is reset initially

  // --- Optional: Force WiFi credential reset on boot ---
  #ifdef FORCE_WIFI_FORGET_ON_BOOT
      Serial.println(F("!!! FORCE_WIFI_FORGET_ON_BOOT is defined. Resetting WiFi settings..."));

      // Method 1: Erase SDK-level WiFi Credentials
      // This is the primary way to forget the network stored by the ESP core.
      if (WiFi.disconnect(true)) { // Pass 'true' to erase persistent credentials
           Serial.println(F("SDK WiFi credentials erased successfully."));
      } else {
           Serial.println(F("Warning: SDK WiFi credential erase command failed (might be okay if never connected)."));
      }
      delay(200); // Small delay to allow the operation to settle

      // Method 2: Reset WiFiManager specific settings
      // This clears any settings WiFiManager might store separately (e.g., in SPIFFS/LittleFS JSON)
      // Creating a temporary instance just for this purpose.
      WiFiManager tempWiFiManager;
      tempWiFiManager.resetSettings();
      Serial.println(F("WiFiManager settings reset complete."));
      Serial.println(F("Device will now enter AP mode for configuration."));
      // Note: After this, the ESP has no saved WiFi info.
      // WiFiManager's autoConnect will fail the internal connection check
      // and proceed directly to starting the AP portal.
  #endif // FORCE_WIFI_FORGET_ON_BOOT
  // --- End of Optional Reset Section ---


  // --- Proceed with standard WiFiManager setup ---
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Ensure text color is set
  display.setTextSize(1);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println(F("WiFi Connecting..."));
  #ifdef FORCE_WIFI_FORGET_ON_BOOT
      display.println(F("(Reset active)")); // Indicate reset happened
  #endif
  display.display();
  delay(100); // Show message briefly

  // Declare the WiFiManager instance that will handle the connection process
  WiFiManager wifiManager;

  // --- Configure WiFiManager ---
  // Generate a unique AP name based on chip ID to avoid conflicts
  String apName = String(DEVICE_NAME) + "-" + String(ESP.getChipId(), HEX);
  Serial.print(F("Config Portal AP Name: ")); Serial.println(apName);

  // Set portal timeout (seconds) - how long the AP stays active if no connection
  wifiManager.setConfigPortalTimeout(180); // 3 minutes

  // --- Configure Callbacks ---
  // Callback when AP mode starts
  wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
      Serial.println(F("Entered WiFiManager AP Mode"));
      configMode = true; // Set flag indicating AP is active
      display.clearDisplay();
      display.setTextSize(1); // Reset text size just in case
      display.setCursor(0, 0);
      display.println(F("WiFi Setup Mode"));
      display.setCursor(0, 16);
      display.println(F("Connect to:"));
      display.setCursor(0, 26);
      display.println(myWiFiManager->getConfigPortalSSID()); // Display the actual AP name
      display.setCursor(0, 36);
      display.println(F("IP: 192.168.4.1"));
      display.display();

      // Start blinking the LED to indicate AP mode
      if (blinkTicker.active() == false) { // Avoid restarting ticker if already running
          blinkTicker.attach_ms(200, []() { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); }); // Blink faster
      }
      Serial.println(F("AP Callback: Display updated, LED blinking."));
  });

  // Callback when configuration is saved (via portal)
  wifiManager.setSaveConfigCallback([]() {
      Serial.println(F("WiFiManager: Credentials Saved/Entered via portal. Attempting connection..."));
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("Credentials Saved"));
      display.println(F("Connecting..."));
      display.display();

      // Stop blinking LED as we transition away from AP mode
      if (blinkTicker.active()) {
          blinkTicker.detach();
          digitalWrite(LED_PIN, LOW); // Ensure LED is off after detaching
      }
      configMode = false; // No longer strictly in AP mode, trying to connect
      Serial.println(F("Save Callback: Stopped LED blink, configMode=false"));
  });

  // --- Start the connection process ---
  Serial.println(F("Starting WiFiManager autoConnect..."));
  // If FORCE_WIFI_FORGET_ON_BOOT was defined, autoConnect will immediately enter AP mode
  // because the initial internal check for saved credentials will fail.
  // If the define is commented out, it will first try to connect using saved credentials.
  bool connectResult = wifiManager.autoConnect(apName.c_str());

  Serial.print(F("WiFiManager autoConnect finished. Result: "));
  Serial.println(connectResult ? F("Success") : F("Failed/Timeout/Exited"));
  delay(50); // Short pause

  // --- Handle Connection Result ---
  if (!connectResult) {
      Serial.println(F("Failed to connect WiFi via autoConnect (or portal timed out/exited)."));
      // Display error message
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println(F("WiFi Failed!"));
      display.println(F("Check Config Portal"));
      display.println(F("or Reset Device."));
      display.println(F("Restarting in 5s..."));
      display.display();
      blinkLED(5); // Fast blink error
      delay(5000);
      ESP.restart(); // Restart to try again (or enter AP mode again if reset is forced)
  }

  // --- Post-Connection Cleanup and Setup ---
  // Ensure AP mode flag and blinker are off if we successfully connected
  if (configMode) { // Should be false if connected, might be true if timeout happened but somehow connectResult was true (unlikely)
      Serial.println(F("Post-connect: configMode was true, setting false and stopping blinker."));
      configMode = false;
      if (blinkTicker.active()) {
          blinkTicker.detach();
          digitalWrite(LED_PIN, LOW);
      }
  } else if (blinkTicker.active()) {
       // If somehow blinker is still active after successful connect without configMode being true
       Serial.println(F("Post-connect: Blinker active but configMode false, stopping blinker."));
       blinkTicker.detach();
       digitalWrite(LED_PIN, LOW);
  }


  // Log success details and start mDNS (only if actually connected)
  if (WiFi.status() == WL_CONNECTED) { // Double-check connection status is really connected
      Serial.println(F("\nWiFi Successfully Connected!"));
      Serial.print(F("SSID: ")); Serial.println(WiFi.SSID());
      Serial.print(F("IP Address: ")); Serial.println(WiFi.localIP());
      Serial.print(F("RSSI: ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));

      // Update display with connection info
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("WiFi Connected!"));
      display.print(F("IP:"));
      display.println(WiFi.localIP());
      // You could add SSID here too if space permits
      // display.print(WiFi.SSID());
      display.display();
      delay(1500); // Show connection info briefly

      // Start mDNS for local network discovery (e.g., http://SmartDisplay.local)
      Serial.println(F("Starting mDNS responder..."));
      if (MDNS.begin(DEVICE_NAME)) {
          MDNS.addService("http", "tcp", HTTP_PORT);
          Serial.print(F("mDNS responder started. Access device at: http://"));
          Serial.print(DEVICE_NAME);
          Serial.println(F(".local"));
      } else {
          Serial.println(F("Error starting mDNS responder!"));
      }
  } else {
       // This case should theoretically be caught by the !connectResult check earlier,
       // but added as a safeguard.
       Serial.println(F("Error: WiFiManager reported success, but WiFi.status() is not WL_CONNECTED!"));
       // Handle this unexpected state, maybe restart?
       display.clearDisplay(); display.setCursor(0,0); display.println("WiFi ERR!"); display.println("Status Mismatch"); display.println("Restarting..."); display.display();
       delay(5000); ESP.restart();
  }

  Serial.println(F("--- Finished WiFi Setup ---"));

} // --- End of setupWiFi ---
void setupNTP() {
  timeClient.begin(); // Initialize the NTP client
  Serial.println(F("NTP Client started. Requesting initial time update..."));

    // --- Diagnostic log before first update attempt ---
    Serial.print(F("  Diag - NTP Init: ms=")); Serial.print(millis());
    Serial.print(F(", WiFi Status=")); Serial.print(WiFi.status());
    Serial.print(F(", RSSI=")); Serial.print(WiFi.RSSI());
    Serial.print(F(", Heap=")); Serial.println(ESP.getFreeHeap());
    IPAddress ntpServerIP;
    if (WiFi.hostByName(NTP_SERVER, ntpServerIP)) {
        Serial.print(F("  Diag - NTP Init: Resolved '")); Serial.print(NTP_SERVER); Serial.print(F("' to ")); Serial.println(ntpServerIP);
    } else {
        Serial.print(F("  Diag - NTP Init: FAILED to resolve '")); Serial.print(NTP_SERVER); Serial.println("'");
    }
    // --- End Diagnostic ---

  // Attempt to get the time immediately. Force update ignores the internal interval.
  if(timeClient.forceUpdate()) {
      lastNtpUpdate = millis(); // Record time of successful update
      Serial.print(F("NTP time updated successfully: ")); Serial.println(timeClient.getFormattedTime());
  } else {
      Serial.println(F("Initial NTP update failed. Will retry later based on sensor update interval."));
      lastNtpFailLogTime = millis(); // Set fail time so we don't log again immediately in the loop
  }
}

void setupServer() {
  // --- Standard page handlers ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);
  server.on("/reset", HTTP_GET, handleReset);   // Simple reset/factory trigger via GET
  server.on("/update", HTTP_GET, handleUpdate); // Redirect to firmware update page



  server.on("/toggleled0", HTTP_GET, handleToggleLedD0);
  server.on("/toggleled3", HTTP_GET, handleToggleLedD3);

  // --- API endpoints ---
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/history", HTTP_GET, handleAPIHistory);

  // --- Mode change handlers ---
  server.on("/normal", HTTP_GET, []() { handleModeChange(NORMAL); });
  server.on("/smile", HTTP_GET, []() { handleModeChange(SMILE); });
  server.on("/sad", HTTP_GET, []() { handleModeChange(SAD); });
  server.on("/rage", HTTP_GET, []() { handleModeChange(RAGE); });
  server.on("/cool", HTTP_GET, []() { handleModeChange(COOL); });
  server.on("/surprised", HTTP_GET, []() { handleModeChange(SURPRISED); });
  server.on("/sleepy", HTTP_GET, []() { handleModeChange(SLEEPY); });
  server.on("/weather", HTTP_GET, []() { handleModeChange(WEATHER); });

  // --- Static asset handlers (served from PROGMEM) ---
  server.on("/main.css", HTTP_GET, []() { server.send(200, "text/css", MAIN_CSS); });
  server.on("/main.js", HTTP_GET, []() { server.send(200, "text/javascript", MAIN_JS); });

  // --- Special Handlers ---
  server.on("/report", HTTP_GET, []() { server.send(200, "text/html", generateReport()); });

  // Setup HTTP Updater (serves firmware update page at /firmware)
  // The second argument is the username, third is password for update auth (optional)
  httpUpdater.setup(&server, "/firmware"); // Unprotected update

  // --- Not Found Handler (try to serve files from SPIFFS) ---
  server.onNotFound(handleNotFound);

  // --- Start the server ---
  server.begin();
  Serial.println(F("HTTP server started"));
}

#ifdef ENABLE_MQTT
void setupMQTT() {
  // Configure the MQTT client with the server address and port
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  // Set the function to call when an MQTT message arrives
  mqttClient.setCallback(mqttCallback);
  Serial.println(F("MQTT Client configured. Will attempt connection in main loop via reconnectMQTT()."));
  // Note: The initial connection attempt happens in the loop via reconnectMQTT()
  // This avoids blocking setup if the MQTT broker is temporarily unavailable.
}
#endif // ENABLE_MQTT

#ifdef ENABLE_OTA
void setupOTA() {
  // --- Configure OTA ---
  ArduinoOTA.setPort(8266);             // Default port for ESP8266 OTA
  ArduinoOTA.setHostname(DEVICE_NAME);  // Set hostname for easier identification in Arduino IDE Ports list

  // Set password for OTA updates if one is defined and not the default "1234"
  if (strlen(devicePassword) > 0 && strcmp(devicePassword, "1234") != 0) {
    ArduinoOTA.setPassword(devicePassword);
     Serial.println(F("OTA Password SET. Use the configured device password for uploads."));
  } else {
    Serial.println(F("OTA Password NOT SET (using default or empty password). Updates are unprotected."));
  }

  // --- Define OTA Callbacks ---
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("OTA Start updating " + type);

    // Stop regular tasks to free up resources and prevent interference
    sensorTicker.detach();
    displayTicker.detach();
    #ifdef ENABLE_DATA_LOGGING
        dataLogTicker.detach();
    #endif
    if (blinkTicker.active()) blinkTicker.detach(); // Stop AP blink if it was happening
    digitalWrite(LED_PIN, HIGH); // Turn LED solid ON during OTA

    Serial.println(F("Tickers detached for OTA."));
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0, 0); display.println(F("OTA Update Start"));
    display.println(type); display.display();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("\nOTA End"));
    digitalWrite(LED_PIN, LOW); // Turn LED off
    display.setCursor(0, 30); display.println(F("Update Success!"));
    display.println(F("Rebooting...")); display.display();
    // Note: Reboot is handled automatically by ArduinoOTA library after onEnd callback finishes.
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentage = (progress / (total / 100));
    Serial.printf("OTA Progress: %u%%\r", percentage);
    digitalWrite(LED_PIN, (millis() / 100) % 2); // Blink LED quickly during progress

    // Update display less frequently to avoid slowing down transfer
    static unsigned long lastOtaDisplayUpdate = 0;
    if (millis() - lastOtaDisplayUpdate > 250) {
        display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0);
        display.println("OTA Update"); display.print(F("Progress: ")); display.print(percentage); display.println(F("%"));
        drawProgressBar(0, 32, SCREEN_WIDTH, 10, percentage); // Draw progress bar
        display.display();
        lastOtaDisplayUpdate = millis();
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    digitalWrite(LED_PIN, HIGH); // Turn LED solid ON for error
    String errorMsg;
    if (error == OTA_AUTH_ERROR) errorMsg = F("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) errorMsg = F("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) errorMsg = F("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) errorMsg = F("Receive Failed");
    else if (error == OTA_END_ERROR) errorMsg = F("End Failed");
    else errorMsg = F("Unknown Error");

    Serial.println(errorMsg);
    display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
    display.println(F("OTA Error!")); display.println(errorMsg);
    display.display();

    delay(3000);
    // Consider restarting tickers or the device after error. A restart is often safest.
    // ESP.restart();
  });

  // --- Start OTA service ---
  ArduinoOTA.begin();
  Serial.println(F("OTA ready"));
}
#endif // ENABLE_OTA


// ========================================================================
//   WORKER FUNCTIONS (Called from loop based on Ticker flags)
// ========================================================================

void performSensorUpdate() {
  // --- Read DHT Sensor ---
  float newTemp = dht.readTemperature();   // Read temperature as Celsius (the default)
  float newHumidity = dht.readHumidity(); // Read humidity

  // Check if reads were successful (will be NAN Not a Number if failed)
  if (!isnan(newTemp) && !isnan(newHumidity)) {
    // Basic sanity check (optional, depends on expected environment)
    // if (newTemp > -40 && newTemp < 80 && newHumidity >= 0 && newHumidity <= 100) {
        currentTemp = newTemp;
        currentHumidity = newHumidity;
        lastSensorUpdate = millis(); // Record time of successful read

        // Update simple rolling history for web chart (only 24 points)
        temperatureHistory[historyPointer] = currentTemp;
        humidityHistory[historyPointer] = currentHumidity;
        historyPointer = (historyPointer + 1) % 24; // Wrap around the buffer
    // } else {
        // Serial.printf("DHT Read Warning: Temp=%.1f, Humid=%.1f - Outside expected range.\n", newTemp, newHumidity);
        // // Optionally set to NAN if readings are crazy
        // currentTemp = NAN;
        // currentHumidity = NAN;
    // }
  } else {
    // Reading failed, don't update current values, print error
    Serial.println(F("Failed to read from DHT sensor!"));
    // Optionally, set values to NAN to indicate failure on display/MQTT
    // currentTemp = NAN;
    // currentHumidity = NAN;
  }

  // --- NTP Time Update Check ---
  // Check if it's time for a periodic NTP update or if the time has never been set (lastNtpUpdate == 0)
  if (WiFi.status() == WL_CONNECTED && (millis() - lastNtpUpdate > NTP_UPDATE_INTERVAL_MS || lastNtpUpdate == 0)) {

    // --- Diagnostic log before periodic update attempt ---
    Serial.println(F("Attempting periodic NTP update..."));
    Serial.print(F("  Diag - NTP Periodic: ms=")); Serial.print(millis());
    Serial.print(F(", WiFi Status=")); Serial.print(WiFi.status());
    Serial.print(F(", RSSI=")); Serial.print(WiFi.RSSI());
    Serial.print(F(", Heap=")); Serial.println(ESP.getFreeHeap());
    IPAddress ntpServerIP;
    if (WiFi.hostByName(NTP_SERVER, ntpServerIP)) {
        Serial.print(F("  Diag - NTP Periodic: Resolved '")); Serial.print(NTP_SERVER); Serial.print(F("' to ")); Serial.println(ntpServerIP);
    } else {
        Serial.print(F("  Diag - NTP Periodic: FAILED to resolve '")); Serial.print(NTP_SERVER); Serial.println("'");
    }
    // --- End Diagnostic ---

    if(timeClient.update()) { // Use standard update method from library
        lastNtpUpdate = millis(); // Record time of successful update
        Serial.print(F("NTP time updated: ")); Serial.println(timeClient.getFormattedTime());
        // Clear the fail log timer on success
        lastNtpFailLogTime = 0;
    } else {
        // Only log failure periodically to avoid spamming the serial console
        if (lastNtpFailLogTime == 0 || millis() - lastNtpFailLogTime > NTP_FAIL_LOG_INTERVAL) {
            Serial.println(F("Periodic NTP update failed. Will retry later."));
            lastNtpFailLogTime = millis(); // Update log time *after* printing
        }
        // Do NOT update lastNtpUpdate, so it will keep trying at the NTP_UPDATE_INTERVAL_MS rate
        // Or more frequently if lastNtpUpdate is still 0 (initial failure state)
    }
  }

  // --- Update local Time/Date Strings ---
  // Check if NTP has ever successfully updated (epoch time > roughly year 2001)
  if (timeClient.getEpochTime() > 1000000000) {
    time_t epochTime = timeClient.getEpochTime();
    // Use the POSIX standard localtime() function to convert epoch to broken-down time struct
    struct tm *timeInfo = localtime(&epochTime);
    // Check if localtime returned a valid pointer (it shouldn't be NULL)
    if (timeInfo) {
        snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
        snprintf(dateString, sizeof(dateString), "%02d/%02d/%04d", timeInfo->tm_mday, timeInfo->tm_mon + 1, timeInfo->tm_year + 1900); // tm_mon is 0-11, tm_year is years since 1900
    } else {
        // Should not happen if epochTime is valid, but handle defensively
        Serial.println(F("Error: localtime() returned NULL!"));
        strcpy(timeString, "TIME ERR");
        strcpy(dateString, "DATE ERR");
    }
  } else {
      // Keep default "--:--" strings if time hasn't been set yet
  }


  // --- MQTT Publish Check ---
  #ifdef ENABLE_MQTT
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected() && (millis() - lastMqttPublish > MQTT_PUBLISH_INTERVAL_MS)) {
        publishSensorData();
        lastMqttPublish = millis(); // Reset publish timer only on successful publish attempt
    }
  #endif
}


void performDisplayUpdate() {
  display.clearDisplay();
  animationFrame = (animationFrame + 1) % 3; // Cycle through 0, 1, 2 for animations

  // --- Select display content based on currentMode ---
  switch (currentMode) {
    case NORMAL:    drawNormal(); break;
    case SMILE:     drawAnimatedFace(smileFaces[animationFrame]); break;
    case SAD:       drawAnimatedFace(sadFaces[animationFrame]); break;
    case RAGE:      drawAnimatedFace(rageFaces[animationFrame]); break;
    case COOL:      drawAnimatedFace(coolFaces[animationFrame]); break;
    case SURPRISED: drawAnimatedFace(surprisedFaces[animationFrame]); break;
    case SLEEPY:    drawAnimatedFace(sleepyFaces[animationFrame]); break;
    case WEATHER:   drawWeather(); break;
    default: // Should not happen
      display.setTextSize(1); display.setCursor(0,0); display.println(F("Unknown Mode!"));
  }

  // --- Overlay Lockout Status (if active) ---
  if (lockout) {
    unsigned long remaining = (lockoutEndTime > millis()) ? (lockoutEndTime - millis()) / 1000 : 0;
    display.setTextSize(1);
    // Draw a filled rectangle to clear the area, then text
    display.fillRect(0, SCREEN_HEIGHT-9, SCREEN_WIDTH, 9, SSD1306_BLACK); // Clear bottom line area
    display.setTextColor(SSD1306_WHITE); // Ensure text is white
    display.setCursor(0, SCREEN_HEIGHT-8); // Position text on bottom line
    display.print(F("LOCKED: ")); display.print(remaining); display.print(F("s"));
  }

  // --- Push buffer to the display hardware ---
  display.display();
}


#ifdef ENABLE_DATA_LOGGING
void performDataLog() {
  Serial.println(F(">>> Performing Data Log"));

  // --- Prerequisites Check ---
  // Don't log if sensor data is invalid
  if (isnan(currentTemp) || isnan(currentHumidity)) {
    Serial.println(F("Skipping data log: Invalid sensor readings (NaN)."));
    return;
  }
  // Don't log if WiFi is not connected (can't get valid time)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Skipping data log: WiFi not connected."));
    return;
  }
   // Don't log if NTP time isn't valid yet
  time_t now = timeClient.getEpochTime();
  if (now < 1000000000) { // Check if time looks like a valid Unix timestamp
     Serial.println(F("Skipping data log: NTP time not synchronized yet."));
     return;
  }

  // --- Store Data in History Array ---
  history[historyIndex].temperature = currentTemp;
  history[historyIndex].humidity = currentHumidity;
  history[historyIndex].timestamp = now;

  Serial.printf("Logging: T=%.1fC, H=%.1f%%, TS=%lu at Index %d\n",
                history[historyIndex].temperature, history[historyIndex].humidity,
                (unsigned long)history[historyIndex].timestamp, historyIndex);

  // Move to the next index, wrap around if necessary
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  // Mark that the buffer has wrapped around at least once
  if (historyIndex == 0 && !historyWrapped) {
      historyWrapped = true;
      Serial.println(F("Data logging history buffer has wrapped."));
  }

  // --- Publish Logged Data via MQTT (Optional) ---
  #ifdef ENABLE_MQTT
  if (mqttClient.connected()) {
    // Create a JSON document to hold the logged data
    // Adjust size based on complexity. 128 should be enough for T/H/TS.
    DynamicJsonDocument doc(128);
    doc["temperature"] = currentTemp; // Use current temp, already stored
    doc["humidity"] = currentHumidity; // Use current humidity, already stored
    doc["timestamp"] = now;            // Use the timestamp just acquired

    // Serialize JSON to a string
    String jsonPayload;
    serializeJson(doc, jsonPayload);

    // Construct the MQTT topic
    String logTopic = String(DEVICE_NAME) + "/log";

    // Publish the JSON payload
    if (mqttClient.publish(logTopic.c_str(), jsonPayload.c_str())) {
        // Serial.println(F("Published logged data point to MQTT topic: ") + logTopic); // Verbose
    } else {
        Serial.println(F("ERROR: Failed to publish logged data to MQTT."));
    }
  } else {
    Serial.println(F("Could not publish logged data: MQTT not connected."));
  }
  #endif // ENABLE_MQTT

  Serial.println(F("<<< Finished Data Log"));
}
#endif // ENABLE_DATA_LOGGING


// ========================================================================
//   DISPLAY DRAWING FUNCTIONS
// ========================================================================

void drawNormal() {
  // --- Top Section: Time and Date ---
  display.setTextSize(2); // Larger font for time
  display.setCursor(0, 0);
  display.print(timeString);

  display.setTextSize(1); // Smaller font for date
  display.setCursor(0, 18); // Position below time
  display.print(dateString);

  // --- Middle Section: Sensor Readings ---
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print(F("T:"));
  if (!isnan(currentTemp)) display.print(currentTemp, 1); else display.print(F("--.-"));
  // Use degree symbol (char 247 in cp437 font if enabled, otherwise 'C')
  display.write((char)247); display.print(F("C")); // Requires display.cp437(true); in setupDisplay()

  display.setCursor(64, 30); // Position humidity on the right half
  display.print(F(" H:"));
  if (!isnan(currentHumidity)) display.print(currentHumidity, 1); else display.print(F("--.-"));
  display.print(F("%"));

  // --- Bottom Section: Network Status & Info ---
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print(F("IP: "));
  if (WiFi.status() == WL_CONNECTED) {
      display.print(WiFi.localIP().toString());
  } else {
      display.print(F("Offline"));
  }

  // Status line (Bottom row)
  display.setCursor(0, 54); // Bottom line
  #ifdef ENABLE_MQTT
    if (mqttConnected) display.print(F("MQTT OK"));
    else if (WiFi.status() == WL_CONNECTED) display.print(F("MQTT Con...")); // Trying to connect
    else display.print(F("Network Offline"));
  #else
     if (WiFi.status() == WL_CONNECTED) display.print(F("Network OK")); else display.print(F("Network Offline"));
  #endif

  // Optionally show password reveal prompt or state if button held
  if (digitalRead(SWITCH_PASS) == LOW) {
    // Clear the right part of the status line and show password
    display.fillRect(64, 54, SCREEN_WIDTH - 64, 8, SSD1306_BLACK);
    display.setCursor(64, 54);
    display.print(F(" connected"));
  }
}

void drawAnimatedFace(const char* face) {
  // --- Background: Sensor readings ---
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("T:"));
  if (!isnan(currentTemp)) display.print(currentTemp, 1); else display.print("--.-");
  display.write((char)247); display.print(F("C ")); // Degree symbol
  display.print(F("H:"));
  if (!isnan(currentHumidity)) display.print(currentHumidity, 1); else display.print("--.-");
  display.print(F("%"));

  // --- Foreground: Animated Face ---
  display.setTextSize(4); // Large font for the face
  // Center the face horizontally (approximate)
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(face, 0, 0, &x1, &y1, &w, &h);
  int cursorX = (SCREEN_WIDTH - w) / 2;
  // Center vertically (approximate)
  int cursorY = ((SCREEN_HEIGHT - 8 - h) / 2) + 8; // +8 to account for sensor reading line

  display.setCursor(cursorX, cursorY);
  display.print(face);
}

void drawWeather() {
  // --- Top text ---
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Weather Condition"));

  // --- Sensor Readings (Left side) ---
  display.setCursor(0, 16);
  display.print(F("T: "));
  if (!isnan(currentTemp)) display.print(currentTemp, 1); else display.print("--.-");
  display.write((char)247); display.print(F(" C")); // Degree symbol

  display.setCursor(0, 26);
  display.print(F("H: "));
  if (!isnan(currentHumidity)) display.print(currentHumidity, 1); else display.print("--.-");
  display.print(F("%"));

  // --- Condition Icon and Text (Right side) ---
  const uint8_t * bmp; // Pointer to the bitmap data in PROGMEM
  String conditionText;

  // Determine condition based on humidity (simple example)
  if (isnan(currentHumidity)) {
       bmp = cloud_bmp; // Default or placeholder if no data
       conditionText = F("N/A");
  } else if (currentHumidity > 85) {
       bmp = rainy_bmp;
       conditionText = F("Rainy");
  } else if (currentHumidity > 60) {
       bmp = cloud_bmp;
       conditionText = F("Cloudy");
  } else {
       bmp = sunny_bmp;
       conditionText = F("Clear");
  }

  // Draw condition text first
  display.setCursor(70, 16); // Position text next to sensor readings
  display.println(conditionText);

  // Draw bitmap icon below the condition text
  // Position: x=80, y=28, bitmap pointer, width=32, height=32, color=WHITE
  display.drawBitmap(80, 28, bmp, 32, 32, SSD1306_WHITE);
}

void drawGraph() {
  // This function draws a simple graph based on the `temperatureHistory` array
  // Currently, this is only called internally by one of the web handlers, not directly on the OLED.
  // If you want to draw a graph ON the OLED, call this from `performDisplayUpdate()`
  // and adjust coordinates/size.

  const int graphX = 0;                   // Top-left X of graph area
  const int graphY = SCREEN_HEIGHT - 16; // Top-left Y (leave space below)
  const int graphWidth = SCREEN_WIDTH;   // Full width
  const int graphHeight = 15;            // Height of graph area

  // Optional: Draw bounding box or X-axis line
  display.drawRect(graphX, graphY, graphWidth, graphHeight + 1, SSD1306_WHITE); // Box
  // display.drawLine(graphX, graphY + graphHeight, graphX + graphWidth, graphY + graphHeight, WHITE); // X-axis Line

  // --- Find Min/Max Temperature in the history buffer ---
  float minTemp = 1000.0;
  float maxTemp = -1000.0;
  int validPoints = 0;
  for (int i = 0; i < 24; i++) {
     // Check if the history point is valid (not NaN and maybe not zero if zero is impossible)
     if (!isnan(temperatureHistory[i]) && temperatureHistory[i] != 0 ) { // Adjust condition if 0 C is valid
        if(temperatureHistory[i] < minTemp) minTemp = temperatureHistory[i];
        if(temperatureHistory[i] > maxTemp) maxTemp = temperatureHistory[i];
        validPoints++;
     }
  }

  // Don't draw if insufficient data or range is too small
  if (validPoints < 2 || (maxTemp - minTemp < 1.0)) return; // Avoid division by zero or flat line

  // Add a little padding to min/max for better visualization
  minTemp -= 1.0;
  maxTemp += 1.0;
  float tempRange = maxTemp - minTemp;

  // --- Draw the Temperature Line ---
  int lastX = -1, lastY = -1; // Store previous point coords

  for (int i = 0; i < 24; i++) {
    // Calculate the actual index in the circular buffer, starting from the oldest point
    int index = (historyPointer + i) % 24;

    if (!isnan(temperatureHistory[index]) && temperatureHistory[index] != 0 ) {
        // Map the history index (0-23) to the screen X coordinate
        int x = graphX + map(i, 0, 23, 0, graphWidth - 1);
        // Map the temperature value to the screen Y coordinate
        // Invert Y because screen coords increase downwards
        // Constrain ensures the value stays within the graph height
        int y = graphY + graphHeight - constrain(map(temperatureHistory[index], minTemp, maxTemp, 0, graphHeight), 0, graphHeight);

        // Draw a line from the previous point to the current point
        if(lastX != -1) { // Don't draw a line for the very first point
             display.drawLine(lastX, lastY, x, y, SSD1306_WHITE);
        }
        // Store current point as the next 'last' point
        lastX = x;
        lastY = y;
    } else {
        // If data is invalid (NaN), break the line segment
        lastX = -1;
        lastY = -1;
    }
  }
}

void drawProgressBar(int x, int y, int width, int height, int progress) {
  progress = constrain(progress, 0, 100); // Ensure progress is 0-100

  // Draw the outline of the progress bar
  display.drawRect(x, y, width, height, SSD1306_WHITE);

  // Calculate the width of the filled portion
  // Map progress (0-100) to the inner width (width - 2 pixels for the border)
  int filledWidth = map(progress, 0, 100, 0, width - 2);

  // Draw the filled rectangle representing the progress
  // (x+1, y+1) to start inside the border
  // (height-2) to stay inside the border vertically
  display.fillRect(x + 1, y + 1, filledWidth, height - 2, SSD1306_WHITE);
}


// ========================================================================
//   MQTT FUNCTIONS
// ========================================================================

#ifdef ENABLE_MQTT

// --- MQTT Message Callback ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to String for easier processing
  // Important: Ensure payload is null-terminated *before* converting
  payload[length] = '\0'; // Add null terminator
  String message = String((char*)payload);
  String topicStr = String(topic);

  Serial.print(F("MQTT Message Received [")); Serial.print(topic); Serial.print(F("]: ")); Serial.println(message);

  // Construct the expected prefix for device-specific topics
  String prefix = String(DEVICE_NAME) + "/";

  // Ignore messages that don't start with our device prefix (unless subscribing to general topics)
  if (!topicStr.startsWith(prefix)) {
       Serial.println(F("Ignoring MQTT message: Topic doesn't match device prefix."));
       return;
  }

  // Extract the command part of the topic (after the prefix)
  String commandTopic = topicStr.substring(prefix.length());

  // --- Process known command topics ---
  if (commandTopic == "mode") {
      // Change display mode based on payload
      if (message == "normal") handleModeChange(NORMAL);
      else if (message == "smile") handleModeChange(SMILE);
      else if (message == "sad") handleModeChange(SAD);
      else if (message == "rage") handleModeChange(RAGE);
      else if (message == "cool") handleModeChange(COOL);
      else if (message == "surprised") handleModeChange(SURPRISED);
      else if (message == "sleepy") handleModeChange(SLEEPY);
      else if (message == "weather") handleModeChange(WEATHER);
      else Serial.println(F("Unknown mode requested via MQTT"));
      // Note: handleModeChange will update display and publish new state back
  }
  else if (commandTopic == "buzzer") {
      // Activate buzzer based on payload (e.g., frequency)
      int frequency = message.toInt();
      if (frequency >= 50 && frequency <= 5000) { // Basic sanity check for frequency
         playTone(frequency, 500); // Play tone for 500ms
         Serial.print(F("MQTT triggered buzzer: Freq=")); Serial.println(frequency);
         // Optionally publish an ack/status back
         // mqttClient.publish((prefix + "buzzer/status").c_str(), "triggered", false);
      } else {
         Serial.println(F("Invalid MQTT buzzer frequency received"));
      }
  }
  else if (commandTopic == "command") {
      // Execute specific device commands
      if (message == "reboot") {
         Serial.println(F("Reboot command received via MQTT. Rebooting..."));
         // Publish status before rebooting (optional, might not send if reboot is fast)
         mqttClient.publish((prefix+"status").c_str(), "rebooting", true); // Retained status
         delay(500); // Give MQTT time to publish
         rebootDevice();
      } else if (message == "reset") {
         Serial.println(F("Factory Reset command received via MQTT. Resetting..."));
         mqttClient.publish((prefix+"status").c_str(), "resetting", true);
         delay(500);
         factoryReset();
      } else if (message == "ping") {
          // Respond to a ping request (useful for checking connectivity)
          mqttClient.publish((prefix+"status").c_str(), "pong", false); // Not retained
          Serial.println(F("Responded to MQTT ping with pong."));
      } else if (message == "get_status") {
           // Force immediate publish of current sensor data
           publishSensorData();
      } else {
          Serial.println(F("Unknown command received via MQTT"));
      }
  } else {
      // Handle other potential subscribed topics or log unknown ones
      Serial.print(F("Received message on unhandled subscribed topic: ")); Serial.println(commandTopic);
  }
} // end mqttCallback

// --- MQTT Reconnect Logic ---
void reconnectMQTT() {
  // Exit immediately if WiFi isn't connected
  if (WiFi.status() != WL_CONNECTED) return;

  // Exit if it's too soon to retry connection based on interval
  if (millis() - lastMqttAttempt < MQTT_RECONNECT_INTERVAL) return;

  // Record the time of this connection attempt
  lastMqttAttempt = millis();

  // Check if the client is already connected or connection attempt in progress
  if (!mqttClient.connected()) {
    Serial.print(F("Attempting MQTT connection to ")); Serial.print(MQTT_SERVER); Serial.print(":"); Serial.println(MQTT_PORT);

    // --- Diagnostic log before connection attempt ---
    Serial.print(F("  Diag - MQTT Conn: ms=")); Serial.print(millis());
    Serial.print(F(", WiFi Status=")); Serial.print(WiFi.status());
    Serial.print(F(", RSSI=")); Serial.print(WiFi.RSSI());
    Serial.print(F(", Heap=")); Serial.println(ESP.getFreeHeap());
    IPAddress mqttServerIP;
    if (WiFi.hostByName(MQTT_SERVER, mqttServerIP)) {
        Serial.print(F("  Diag - MQTT Conn: Resolved '")); Serial.print(MQTT_SERVER); Serial.print(F("' to ")); Serial.println(mqttServerIP);
    } else {
        Serial.print(F("  Diag - MQTT Conn: FAILED to resolve '")); Serial.print(MQTT_SERVER); Serial.println("'");
        // Maybe abort connection attempt if DNS fails? Or let connect() try IP if previously resolved?
    }
    // --- End Diagnostic ---

    // Create a unique client ID for this connection attempt
    // Appending random hex ensures uniqueness if multiple devices with same name exist
    // Or if broker cleans up sessions slowly after disconnects
    String clientId = String(DEVICE_NAME) + "-";
    clientId += String(random(0xffff), HEX);
    clientId += String(random(0xffff), HEX); // Make it longer

    // --- Configure Last Will and Testament (LWT) ---
    // This message is published by the broker if the client disconnects unexpectedly
    String lwtTopic = String(DEVICE_NAME) + "/status";
    const char* lwtPayload = "offline";
    const int lwtQos = 1;        // QoS level for LWT (0, 1, or 2)
    const boolean lwtRetain = true; // Retain the LWT message

    // --- Attempt to Connect ---
    Serial.print(F("Connecting as Client ID: ")); Serial.println(clientId);
    // connect(clientId, [username, password], lwtTopic, lwtQos, lwtRetain, lwtMessage, [cleanSession], [protocolVersion])
    // Using NULL for username/password - Add credentials if your broker requires them
    // Default cleanSession=true means broker discards session info on disconnect
    if (mqttClient.connect(clientId.c_str(), nullptr, nullptr, lwtTopic.c_str(), lwtQos, lwtRetain, lwtPayload)) {
      Serial.println(F("MQTT connected!"));
      mqttConnected = true; // Update global state flag

      // Publish "online" status immediately after connection (also retained)
      mqttClient.publish(lwtTopic.c_str(), "online", true);

      // Subscribe to command topics specific to this device
      String prefix = String(DEVICE_NAME) + "/";
      mqttClient.subscribe((prefix + "mode").c_str());    // Topic for changing display mode
      mqttClient.subscribe((prefix + "buzzer").c_str());  // Topic for triggering buzzer
      mqttClient.subscribe((prefix + "command").c_str()); // Topic for general commands (reboot, reset, ping)
      Serial.println(F("Subscribed to command topics:"));
      Serial.println(F("  ") + prefix + F("mode"));
      Serial.println(F("  ") + prefix + F("buzzer"));
      Serial.println(F("  ") + prefix + F("command"));

      // Publish current state immediately on successful connection
      publishSensorData();
      lastMqttPublish = millis(); // Reset publish timer

    } else {
      // Connection failed
      mqttConnected = false; // Update global state flag
      Serial.print(F("MQTT connection failed, rc=")); Serial.print(mqttClient.state());
      Serial.print(F(" ("));
      // Print explanation for common PubSubClient return codes
      switch (mqttClient.state()) {
         case MQTT_CONNECTION_TIMEOUT:      Serial.print(F("Connection Timeout")); break; // (-4) MQTT_CONNECT_TIMEOUT - Added missing define in PubSubClient? Should be CONNECTION
         case MQTT_CONNECTION_LOST:         Serial.print(F("Connection Lost")); break;    // (-3)
         case MQTT_CONNECT_FAILED:          Serial.print(F("Connect Failed")); break;     // (-2) Network level
         case MQTT_DISCONNECTED:            Serial.print(F("Disconnected")); break;        // (-1)
         // case MQTT_CONNECTED:            // (0) Shouldn't happen here in 'else' block
         case MQTT_CONNECT_BAD_PROTOCOL:    Serial.print(F("Bad Protocol")); break;       // (1)
         case MQTT_CONNECT_BAD_CLIENT_ID:   Serial.print(F("Bad ClientID")); break;      // (2)
         case MQTT_CONNECT_UNAVAILABLE:     Serial.print(F("Server Unavailable")); break; // (3)
         case MQTT_CONNECT_BAD_CREDENTIALS: Serial.print(F("Bad Credentials")); break;   // (4)
         case MQTT_CONNECT_UNAUTHORIZED:    Serial.print(F("Unauthorized")); break;      // (5)
         default: Serial.print(F("Unknown Error")); break;
      }
      Serial.println(F("). Will try again later."));
      // Note: reconnectMQTT() will be called again automatically in the loop after MQTT_RECONNECT_INTERVAL
    }
  }
} // end reconnectMQTT


// --- MQTT Publish Sensor Data ---
void publishSensorData() {
  // Check prerequisites: MQTT must be connected
  if (!mqttClient.connected()) {
     Serial.println(F("Skipped MQTT publish: Client not connected."));
     return;
  }

  // Prepare topic prefix
  String prefix = String(DEVICE_NAME) + "/";

  // Prepare payload strings for temperature and humidity
  // dtostrf(floatVal, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf)
  char tempStr[8];  // Buffer for temperature string e.g., "-10.5" + null
  char humidStr[8]; // Buffer for humidity string e.g., "100.0" + null

  String tempPayload = F(""); // Default to empty string if NaN
  String humidPayload = F(""); // Default to empty string if NaN

  if (!isnan(currentTemp)) {
      dtostrf(currentTemp, 4, 1, tempStr); // Use width=4, 1 decimal place
      tempPayload = tempStr;
  }
  if (!isnan(currentHumidity)) {
       dtostrf(currentHumidity, 4, 1, humidStr);
       humidPayload = humidStr;
  }


  // --- Publish sensor data ---
  // Use retain=true for state values that new subscribers should receive immediately
  mqttClient.publish((prefix + "temperature").c_str(), tempPayload.c_str(), true);
  mqttClient.publish((prefix + "humidity").c_str(), humidPayload.c_str(), true);

  // --- Publish non-sensor state ---
  mqttClient.publish((prefix + "mode").c_str(), getModeName(currentMode).c_str(), true); // Publish current display mode

  // --- Publish system information (optional, less frequent, not retained?) ---
  mqttClient.publish((prefix + "ip").c_str(), (WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString().c_str() : "Offline"), true); // Retain IP
  // mqttClient.publish((prefix + "uptime").c_str(), formatUptime().c_str(), false); // Uptime probably shouldn't be retained
  // mqttClient.publish((prefix + "rssi").c_str(), String(WiFi.RSSI()).c_str(), false); // Signal strength varies

  // Consider publishing heap size periodically for debugging?
  // mqttClient.publish((prefix + "heap").c_str(), String(ESP.getFreeHeap()).c_str(), false);

  Serial.println(F("Published status & sensor data to MQTT."));

} // end publishSensorData

#endif // ENABLE_MQTT


// ========================================================================
//   WEB SERVER HANDLERS
// ========================================================================

void handleRoot() {
    // Serve the main HTML page dynamically (includes references to CSS/JS)
    // Uses PROGMEM strings for CSS/JS defined earlier
    // Dynamically inserts current values and states

    String html = F("<!DOCTYPE html><html><head>");
    html += F("<title>Smart Display Control</title>");
    html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    // Use link to CSS served separately
    html += F("<link rel='stylesheet' href='/main.css'>");
    html += F("<meta http-equiv='refresh' content='60'>"); // Optional: auto-refresh page every 60s
    html += F("</head><body><div class='container'>");
    html += F("<h1>") + String(DEVICE_NAME) + F(" Control Panel</h1>"); // Use device name

    // --- Current Readings Card ---
    html += F("<div class='card'><h2>Current Status</h2><div class='readings'>");
    // Temperature Reading
    html += F("<div class='reading temp'><h3>Temperature</h3><div class='value' id='temperature'>");
    html += isnan(currentTemp) ? F("--.-") : String(currentTemp, 1);
    html += F("&#176;C</div></div>"); // Degree symbol: &#176; or &deg;
    // Humidity Reading
    html += F("<div class='reading humid'><h3>Humidity</h3><div class='value' id='humidity'>");
    html += isnan(currentHumidity) ? F("--.-") : String(currentHumidity, 1);
    html += F("%</div></div>");
    html += F("</div><div class='readings'>"); // Second row of readings
    // Time Reading
    html += F("<div class='reading'><h3>Time</h3><div class='value' id='time'>");
    html += String(timeString);
    html += F("</div></div>");
    // Uptime Reading
    html += F("<div class='reading'><h3>Uptime</h3><div class='value' id='uptime'>");
    html += formatUptime();
    html += F("</div></div>");
    html += F("</div></div>"); // Close card

    // --- Display Mode Card ---
    html += F("<div class='card'><h2>Display Mode</h2><div class='controls'>");
    html += F("<div class='card'><h2>Device Control</h2><div class='controls'>");
    // --- Button for D0 LED ---
    html += F("<button id='led-d0-button' class='led-control") + String(ledD0State ? F(" active") : F("")) + F("' onclick=\"location.href='/toggleled0'\">LED D0 (GPIO16): ");
    html += String(ledD0State ? F("ON") : F("OFF"));
    html += F("</button>");
    // --- Button for D3 LED ---
    html += F("<button id='led-d3-button' class='led-control") + String(ledD3State ? F(" active") : F("")) + F("' onclick=\"location.href='/toggleled3'\">LED D3 (GPIO0): ");
    html += String(ledD3State ? F("ON") : F("OFF"));
    html += F("</button>");
    html += F("</div><p style='font-size:0.8em; color:#e74c3c;'>Warning: Toggling LED D3 (GPIO0) might interfere with device booting if reboot occurs while ON (depending on wiring).</p></div>"); // Close Device Control card with warning
    // Buttons for each mode - CSS class matches mode name, 'active' class added if current
    html += F("<button class='normal") + String(currentMode == NORMAL ? F(" active") : F("")) + F("' onclick=\"location.href='/normal'\">Normal</button>");
    html += F("<button class='smile") + String(currentMode == SMILE ? F(" active") : F("")) + F("' onclick=\"location.href='/smile'\">Smile</button>");
    html += F("<button class='sad")   + String(currentMode == SAD   ? F(" active") : F("")) + F("' onclick=\"location.href='/sad'\">Sad</button>");
    html += F("<button class='rage")  + String(currentMode == RAGE  ? F(" active") : F("")) + F("' onclick=\"location.href='/rage'\">Angry</button>");
    html += F("</div><div class='controls'>"); // New row of controls
    html += F("<button class='cool")  + String(currentMode == COOL  ? F(" active") : F("")) + F("' onclick=\"location.href='/cool'\">Cool</button>");
    html += F("<button class='surprised") + String(currentMode == SURPRISED ? F(" active") : F("")) + F("' onclick=\"location.href='/surprised'\">Surprised</button>");
    html += F("<button class='sleepy")+ String(currentMode == SLEEPY ? F(" active") : F("")) + F("' onclick=\"location.href='/sleepy'\">Sleepy</button>");
    html += F("<button class='weather") + String(currentMode == WEATHER ? F(" active") : F("")) + F("' onclick=\"location.href='/weather'\">Weather</button>");
    html += F("</div></div>"); // Close card

    // --- Security Access Card ---
    html += F("<div class='card'><h2>Security Access</h2>");
    if (lockout) {
       unsigned long remaining = (lockoutEndTime > millis())?(lockoutEndTime - millis()) / 1000:0;
       html += F("<p style='color: red;'>Device Locked! Try again in ") + String(remaining) + F(" seconds.</p>");
    } else {
       html += F("<form id='password-form' onsubmit='return false;'>"); // Prevent default form submission
       html += F("<input type='password' id='password' placeholder='Enter password (if set)'>");
       html += F("<button type='submit'>Submit</button></form>"); // JS handles the submit action
       html += F("<div id='password-message'></div>"); // Feedback message area for JS
    }
    html += F("</div>"); // Close card

    // --- Historical Data Card (using Chart.js) ---
    html += F("<div class='card'><h2>Historical Data (Last ~24 Intervals)</h2>");
    html += F("<div class='graph-container'><canvas id='history-chart'></canvas></div></div>"); // Canvas for JS chart

    // --- Advanced Options Card ---
    html += F("<div class='card'><h2>Advanced Options</h2><div class='controls'>");
    html += F("<button onclick=\"location.href='/report'\" target='_blank'>Generate System Report</button>");
    html += F("<button onclick=\"location.href='/firmware'\">Update Firmware</button>");
    html += F("</div><div class='controls' style='margin-top: 10px;'>");
    // Reboot Button with confirmation
    html += F("<button style='background:#ff9800' onclick=\"if(confirm('Are you sure you want to reboot the device?')) location.href='/reset?action=reboot'\">Reboot Device</button>");
    // Factory Reset Button with STRONG warning and confirmation
    html += F("<button style='background:#f44336' onclick=\"if(confirm('WARNING: This will erase ALL settings (WiFi, Password) and REBOOT!\\nAre you absolutely sure?')) location.href='/reset?action=factory'\">Factory Reset</button>");
    html += F("</div></div>"); // Close card

    // --- Footer and MQTT Status ---
    #ifdef ENABLE_MQTT
    // Fixed MQTT status bubble (updated by JS)
    html += F("<div id='mqtt-status' class='mqtt-status ") + String(mqttConnected ? F("connected") : F("disconnected")) + F("'>");
    html += F("MQTT: ") + String(mqttConnected ? F("Connected") : (WiFi.status()==WL_CONNECTED ? F("Connecting...") : F("Offline")));
    html += F("</div>");
    #endif
    // Footer with dynamic info (also updated by JS)
    html += F("<div class='footer'><p>Smart Display v") + String(FIRMWARE_VERSION);
    html += F(" | IP: ") + (WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString(): F("Offline"));
    html += F("</p></div>");

    html += F("</div>"); // Close container

    // --- Include JavaScript libraries and our main script ---
    // Include Chart.js from CDN
    html += F("<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>");
    // Include our main JS served separately
    html += F("<script src='/main.js'></script>");
    html += F("</body></html>");

    server.send(200, "text/html", html);
} // end handleRoot


void handleAPIStatus() {
  // Create a JSON document to hold the current status
  // Adjust size based on number of fields. 512 should be safe.
  DynamicJsonDocument doc(512);

  // Use ArduinoJson v6 syntax for adding members
  // Assign value directly, or use nullptr for JSON null if data is invalid/unavailable

  if (isnan(currentTemp)) doc["temperature"] = nullptr; else doc["temperature"] = currentTemp;
  if (isnan(currentHumidity)) doc["humidity"] = nullptr; else doc["humidity"] = currentHumidity;

  doc["time"] = timeString;
  doc["date"] = dateString;
  doc["uptime_ms"] = millis();       // Raw uptime in milliseconds
  doc["uptime"] = formatUptime();   // Formatted uptime string
  doc["mode"] = getModeName(currentMode); // Current display mode name
  doc["lockout_active"] = lockout;      // Security lockout status
  doc["ip_address"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : F("Offline");
  doc["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0; // WiFi signal strength
  doc["heap_free"] = ESP.getFreeHeap();    // Available memory
  doc["version"] = FIRMWARE_VERSION;      // Firmware version
  doc["last_sensor_update_ms"] = lastSensorUpdate; // Millis() time of last successful sensor read

  #ifdef ENABLE_MQTT
    doc["mqtt_connected"] = mqttConnected; // MQTT connection status
    doc["mqtt_server"] = MQTT_SERVER;      // Configured MQTT server
  #else
    doc["mqtt_connected"] = false; // Explicitly false if MQTT is disabled
  #endif

  // Serialize JSON to a string
  String jsonResponse;
  serializeJson(doc, jsonResponse);

  // Send the JSON response
  server.send(200, "application/json", jsonResponse);
} // end handleAPIStatus


void handleAPIHistory() {
  // Create JSON document for historical data (likely needs more space than status)
  DynamicJsonDocument doc(2048); // Increased size for 24*2 data points + labels

  // Create nested arrays for labels and data series
  JsonArray labels = doc.createNestedArray("labels");
  JsonArray tempData = doc.createNestedArray("temperature");
  JsonArray humidData = doc.createNestedArray("humidity");

  // Add labels for the X-axis. Could be timestamps, "minutes ago", or just index.
  // Using simple relative index (-23 to 0) for demonstration.
  for (int i = 0; i < 24; i++) {
       // This label assumes the points represent the last 24 intervals, with 0 being the most recent.
       // It reads chronologically if graph starts from left (-23).
       labels.add(String(i - 23));
       // Alternative: Use timestamps if available and format them?
       // Alternative: Simple index "P"+String(i)
  }

  // Add temperature and humidity data from the circular buffer
  for (int i = 0; i < 24; i++) {
    // Calculate the actual index in the history array, starting from the oldest point
    int index = (historyPointer + i) % 24;

    // Add temperature data: Add the value, or JSON 'null' if data is invalid
    if (!isnan(temperatureHistory[index]) && temperatureHistory[index] != 0) { // Check for validity
        tempData.add(temperatureHistory[index]);
    } else {
        tempData.add(nullptr); // Use JSON null for invalid/missing points
    }

    // Add humidity data: Add the value, or JSON 'null' if data is invalid
    if (!isnan(humidityHistory[index]) && humidityHistory[index] != 0) { // Check for validity
        humidData.add(humidityHistory[index]);
    } else {
        humidData.add(nullptr); // Use JSON null for invalid/missing points
    }
  }

  // Serialize JSON to string
  String jsonResponse;
  serializeJson(doc, jsonResponse);

  // Send the JSON response
  server.send(200, "application/json", jsonResponse);
} // end handleAPIHistory

void handleToggleLedD0() {
    ledD0State = !ledD0State; // Toggle the state variable
    digitalWrite(LED_D0_PIN, ledD0State ? HIGH : LOW); // Update the physical LED state (HIGH=ON assumed)
    Serial.print(F("LED D0 (GPIO16) toggled via Web to: ")); Serial.println(ledD0State ? "ON" : "OFF");
  
    // Optional: Publish state to MQTT if enabled
    #ifdef ENABLE_MQTT
      if (mqttClient.connected()) {
          String topic = String(DEVICE_NAME) + "/led/d0/state";
          String payload = ledD0State ? "ON" : "OFF";
          mqttClient.publish(topic.c_str(), payload.c_str(), true); // Retain state
      }
    #endif
  
    // Redirect back to the root page to show the updated button state
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", ""); // 302 Redirect
  }
  
  void handleToggleLedD3() {
    ledD3State = !ledD3State; // Toggle the state variable
    digitalWrite(LED_D3_PIN, ledD3State ? HIGH : LOW); // Update the physical LED state (HIGH=ON assumed)
    Serial.print(F("LED D3 (GPIO0) toggled via Web to: ")); Serial.println(ledD3State ? "ON" : "OFF");
  
    // Optional: Publish state to MQTT if enabled
    #ifdef ENABLE_MQTT
      if (mqttClient.connected()) {
          String topic = String(DEVICE_NAME) + "/led/d3/state";
          String payload = ledD3State ? "ON" : "OFF";
          mqttClient.publish(topic.c_str(), payload.c_str(), true); // Retain state
      }
    #endif
  
    // Redirect back to the root page to show the updated button state
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", ""); // 302 Redirect
  }


void handleSubmit() {
   // --- Security Check: Lockout ---
   if (lockout) {
        unsigned long remaining = (lockoutEndTime > millis())?(lockoutEndTime - millis()) / 1000:0;
        String msg = "Locked Out! Try again in " + String(remaining) + " sec.";
        server.send(429, "text/plain", msg); // HTTP 429 Too Many Requests
        return;
   }

   // --- Input Validation ---
   if (!server.hasArg("pwd")) {
       server.send(400, "text/plain", "Bad Request: Missing password field ('pwd').");
       return;
   }

   // --- Password Check ---
   String inputPassword = server.arg("pwd");
   if (checkPassword(inputPassword)) {
       // --- Access Granted ---
       failedAttempts = 0; // Reset failed attempts counter
       Serial.println(F("Security access granted via web UI."));
       playMelody(UNLOCK_MELODY, UNLOCK_DURATIONS, 5); // Play success melody
       server.send(200, "text/plain", "Access Granted");

       #ifdef ENABLE_MQTT
         if (mqttClient.connected()) {
             mqttClient.publish((String(DEVICE_NAME) + "/access").c_str(), "granted", false); // Publish non-retained event
         }
       #endif
   } else {
       // --- Access Denied ---
       failedAttempts++;
       Serial.print(F("Web security access FAILED - Attempt ")); Serial.println(failedAttempts);
       playMelody(LOCK_MELODY, LOCK_DURATIONS, 5); // Play failure melody

       String msg = "Access Denied - Attempt " + String(failedAttempts) + "/" + String(MAX_FAILED_ATTEMPTS);

       // Check if lockout should be triggered
       if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
           lockout = true;
           lockoutEndTime = millis() + LOCKOUT_DURATION_MS;
           msg += ". Device LOCKED for " + String(LOCKOUT_DURATION_MS / 1000) + " sec!";
           Serial.print(F("Too many failed attempts. Locking device until ~")); Serial.println(lockoutEndTime);
           #ifdef ENABLE_MQTT
             if (mqttClient.connected()) { mqttClient.publish((String(DEVICE_NAME) + "/access").c_str(), "lockout", false); }
           #endif
       } else {
            #ifdef ENABLE_MQTT
                if (mqttClient.connected()) { mqttClient.publish((String(DEVICE_NAME) + "/access").c_str(), "denied", false); }
            #endif
       }
       server.send(401, "text/plain", msg); // HTTP 401 Unauthorized
   }
} // end handleSubmit


void handleModeChange(DisplayMode newMode) {
  // --- Update State ---
  currentMode = newMode;
  String modeName = getModeName(newMode); // Get the string name of the mode
  Serial.print(F("Display mode changed via Web to: ")); Serial.println(modeName);

  // --- Immediate Feedback (Optional) ---
  // Trigger an immediate display update instead of waiting for the next ticker interval
  // timeToUpdateDisplay = true; // Set flag for loop() to handle - less immediate but non-blocking
   performDisplayUpdate(); // Force immediate update - potentially blocks slightly

  // --- MQTT Update (If enabled and connected) ---
  #ifdef ENABLE_MQTT
    if (mqttClient.connected()) {
        // Publish the new mode state, retain it so new clients get the current mode
        mqttClient.publish((String(DEVICE_NAME) + "/mode").c_str(), modeName.c_str(), true);
    }
  #endif

  // --- HTTP Response ---
  // Redirect the browser back to the root page ("/") after changing the mode
  server.sendHeader("Location", "/"); // Standard way to trigger redirect
  server.send(302, "text/plain", ""); // HTTP 302 Found (Redirect)
} // end handleModeChange


void handleReset() {
  // --- Determine Action ---
  String action = "";
  if (server.hasArg("action")) {
     action = server.arg("action");
  }

  String responseHTMLStart = F("<html><head><title>Action Result</title><meta http-equiv='refresh' content='5;url=/'></head><body style='font-family: sans-serif;'><h1>");
  String responseHTMLEnd = F("</h1><p>You will be redirected back to the main page shortly.</p></body></html>");

  // --- Process Action ---
  if (action == "reboot") {
      Serial.println(F("Reboot requested via web interface."));
      server.send(200, "text/html", responseHTMLStart + F("Rebooting Device...") + responseHTMLEnd);
      delay(1000); // Short delay to allow HTTP response to send
      rebootDevice(); // Function to actually perform the reboot
  }
  else if (action == "factory") {
      Serial.println(F("Factory Reset requested via web interface."));
      server.send(200, "text/html", responseHTMLStart + F("Performing Factory Reset...") + responseHTMLEnd);
      delay(1000); // Short delay for response
      factoryReset(); // Function to perform factory reset
  }
  else {
      // --- Invalid Action ---
      Serial.print(F("Invalid action received in /reset: ")); Serial.println(action);
      server.send(400, "text/plain", "Bad Request: Invalid 'action' parameter provided.");
  }
} // end handleReset


void handleUpdate() {
  // Simple redirect to the /firmware page served by ESP8266HTTPUpdateServer
  server.sendHeader("Location", "/firmware");
  server.send(302, "text/plain", ""); // HTTP 302 Found (Redirect)
} // end handleUpdate


void handleNotFound() {
  // Try to handle the request as a file request from SPIFFS first
  if (handleFileRead(server.uri())) {
    return; // File was served successfully
  }

  // If it wasn't a file, send a standard 404 response
  String message = F("404 Not Found\n\n");
  message += F("URI: "); message += server.uri();
  message += F("\nMethod: "); message += (server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: "); message += server.args(); message += F("\n");
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  Serial.println(F("Sent 404 for URI: ") + server.uri());
} // end handleNotFound


// ========================================================================
//   UTILITY AND HELPER FUNCTIONS
// ========================================================================

void loadConfig() {
  Serial.println(F("Loading configuration from /config.json..."));
  if (SPIFFS.exists("/config.json")) {
    File configFile = SPIFFS.open("/config.json", "r");
    if (configFile) {
      // Allocate JSON document. Adjust size if config grows.
      DynamicJsonDocument doc(512);
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(doc, configFile);
      configFile.close(); // Close the file ASAP

      if (!error) {
        Serial.println(F("Successfully parsed config.json"));
        // Safely read the password string
        if (doc.containsKey("password") && doc["password"].is<const char*>()) {
           // Use strlcpy for safe copying into the char array
           strlcpy(devicePassword, doc["password"], sizeof(devicePassword));
           Serial.println(F("Loaded custom device password from config."));
        } else {
           Serial.println(F("No valid 'password' string found in config, using default or previous value."));
           // Keep the default "1234" or whatever was previously loaded
        }
        // Add more configuration loading here (e.g., MQTT server/port, if desired)
        // Example:
        // if (doc.containsKey("mqtt_user") && doc["mqtt_user"].is<const char*>()) {
        //    strlcpy(mqttUser, doc["mqtt_user"], sizeof(mqttUser));
        // }
      } else {
        // Parsing failed
        Serial.print(F("ERROR: Failed to parse config.json: ")); Serial.println(error.c_str());
        // Maybe delete the corrupted file? Or leave it?
      }
    } else {
        Serial.println(F("ERROR: Failed to open config.json for reading, although it exists."));
    }
  } else {
    // Config file doesn't exist
    Serial.println(F("config.json not found. Using default values. Saving new config file."));
    saveConfig(); // Create a new config file with current defaults
  }
   // Print current password (masked) for confirmation
   Serial.print(F("Effective Password: ")); for(size_t i=0; i<strlen(devicePassword); i++) Serial.print('*'); Serial.println();
}

void saveConfig() {
  Serial.println(F("Saving current configuration to /config.json..."));
  // Allocate JSON document
  DynamicJsonDocument doc(512);

  // Add current settings to the JSON document
  doc["password"] = devicePassword;
  // Add other settings here if they should be saved
  // doc["mqtt_server"] = MQTT_SERVER; // Only if loading from config is also implemented

  // Open the config file for writing (this will create it if it doesn't exist)
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
      Serial.println(F("ERROR: Failed to open config.json for writing!"));
      return;
  }

  // Serialize the JSON document to the file
  size_t bytesWritten = serializeJson(doc, configFile);
  if (bytesWritten == 0) {
       Serial.println(F("ERROR: Failed to write configuration to config.json (maybe file system full?)."));
  } else {
       Serial.print(bytesWritten); Serial.println(F(" bytes written to config.json."));
       Serial.println(F("Configuration saved successfully."));
  }

  // Close the file
  configFile.close();
}

void checkButtons() {
  // --- Ring Button Logic ---
  static bool lastRingState = HIGH;          // Keep track of the previous state
  static unsigned long ringPressTime = 0;    // When was the button pressed?
  bool currentRingState = digitalRead(SWITCH_RING); // Read current state

  // Rising edge (Press): HIGH -> LOW (because of INPUT_PULLUP)
  if (lastRingState == HIGH && currentRingState == LOW) {
      ringPressTime = millis();
      playTone(3000, 100); // Short beep on press
      Serial.println(F("Ring button PRESSED"));
      ringActive = true; // Set global flag (if used elsewhere)
      // Send MQTT message for doorbell ring start
      #ifdef ENABLE_MQTT
      if (mqttClient.connected()) {
          mqttClient.publish((String(DEVICE_NAME) + "/doorbell").c_str(), "ring_start", false);
      }
      #endif
  }
  // Falling edge (Release): LOW -> HIGH
  else if (lastRingState == LOW && currentRingState == HIGH) {
      unsigned long pressDuration = millis() - ringPressTime;
      Serial.print(F("Ring button RELEASED (Pressed for ")); Serial.print(pressDuration); Serial.println(F("ms)"));
      noTone(BUZZER_PIN); // Ensure tone stops if it was continuous
      ringActive = false;
       // Send MQTT message for doorbell ring end
       #ifdef ENABLE_MQTT
       if (mqttClient.connected()) {
           mqttClient.publish((String(DEVICE_NAME) + "/doorbell").c_str(), "ring_end", false);
       }
       #endif
  }
  lastRingState = currentRingState; // Store current state for next iteration

  // --- Password Reveal/Action Button Logic ---
  static bool lastPassState = HIGH;         // Track previous state
  static unsigned long passPressTime = 0;   // When pressed
  bool currentPassState = digitalRead(SWITCH_PASS); // Read current state

  // Press: HIGH -> LOW
  if(lastPassState == HIGH && currentPassState == LOW) {
      passPressTime = millis();
      Serial.println(F("Password button PRESSED"));
      // Action on short press could go here, but currently just used for hold reveal/combo reset
  }
  // Release: LOW -> HIGH
  else if (lastPassState == LOW && currentPassState == HIGH) {
       unsigned long pressDuration = millis() - passPressTime;
       Serial.print(F("Password button RELEASED (Pressed for ")); Serial.print(pressDuration); Serial.println(F("ms)"));
       passPressTime = 0; // Reset press time
       // Trigger immediate display update to remove password if it was shown
       timeToUpdateDisplay = true;
  }
  lastPassState = currentPassState; // Store current state

  // --- Combined Button Hold for Factory Reset ---
  static unsigned long resetPressStartTime = 0; // When did the combo press start?
  const unsigned long RESET_HOLD_DURATION = 5000; // 5 seconds

  // Check if BOTH buttons are currently pressed (LOW)
  if (currentRingState == LOW && currentPassState == LOW) {
     // If the reset timer hasn't started yet, start it now
     if (resetPressStartTime == 0) {
          resetPressStartTime = millis();
          Serial.println(F("Reset Condition Detected: Both buttons pressed... Holding..."));
          // Visual/Audible Feedback:
          display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0);
          display.println("Hold Both Buttons"); display.println("for 5s to"); display.println("FACTORY RESET!");
          display.display();
          playTone(1000, 50); delay(50); playTone(1000,50); // Double beep indicator
     }
     // Else, check if the hold duration has been met
     else if (millis() - resetPressStartTime >= RESET_HOLD_DURATION) {
          Serial.println(F("FACTORY RESET triggered by button hold!"));
          display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
          display.println(F("!! FACTORY RESET !!"));
          display.println(F("Release Buttons NOW"));
          display.display();
          playMelody(ALERT_MELODY, ALERT_DURATIONS, 4); // Play alert sound

          // IMPORTANT: Wait here until buttons are released to avoid accidental repeated resets
          Serial.println(F("Waiting for button release before proceeding with reset..."));
          while (digitalRead(SWITCH_RING) == LOW || digitalRead(SWITCH_PASS) == LOW) {
              playTone(2000, 20); // Short repeating tone while waiting
              delay(200);
              yield(); // Allow background tasks while waiting
          }
          noTone(BUZZER_PIN);
          Serial.println(F("Buttons released, performing factory reset NOW..."));
          factoryReset(); // This function will reboot the device
          // Code below factoryReset() will likely not execute
     }
     // If duration not met yet, do nothing, just continue holding
  } else {
      // If either button is released, cancel the reset timer/process
      if (resetPressStartTime != 0) { // Only print cancel message if timer was running
          Serial.println(F("Reset Condition: Canceled (Button released too soon)."));
          resetPressStartTime = 0; // Reset the start time
          // Restore normal display
          timeToUpdateDisplay = true; // Request display refresh in the next loop
      }
  }
} // end checkButtons

bool checkPassword(String inputPassword) {
    // Simple string comparison against the stored password
    // Ensures comparison is case-sensitive.
    return inputPassword.equals(devicePassword);
}

// --- Filesystem Helpers ---
String getContentType(String filename) {
  // Basic MIME type mapping based on file extension
  if (filename.endsWith(".htm") || filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".json")) return "application/json";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg") || filename.endsWith(".jpeg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/pdf";
  else if (filename.endsWith(".zip")) return "application/zip";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain"; // Default MIME type
}

bool handleFileRead(String path) {
  // Ensure path starts with "/"
  if (!path.startsWith("/")) { path = "/" + path; }

  // If path ends with "/", append "index.html"
  if (path.endsWith("/")) { path += "index.html"; }

  // Check if the file exists (first try .gz, then original)
  String pathWithGz = path + ".gz";
  bool found = false;

  if (SPIFFS.exists(pathWithGz)) {
      path = pathWithGz; // Use the gzipped version
      found = true;
      Serial.print(F("Serving gzipped file: ")); Serial.println(path);
  } else if (SPIFFS.exists(path)) {
      found = true;
      Serial.print(F("Serving file: ")); Serial.println(path);
  }

  if (found) {
      File file = SPIFFS.open(path, "r");
      if (file) {
          String contentType = getContentType(path); // Determine content type (even for .gz)
          // If serving gzipped file, add Content-Encoding header
          if (path.endsWith(".gz")) {
              server.sendHeader("Content-Encoding", "gzip");
              // Override content type if it ended up as gzip - browser needs original type
              contentType = getContentType(path.substring(0, path.length() - 3));
          }
          server.streamFile(file, contentType); // Stream the file content
          file.close();
          return true; // Indicate success
      } else {
          Serial.println(F("Error opening existing file: ") + path);
          // Fall through to return false
      }
  }

  // File not found in SPIFFS
  // Serial.println(F("File not found in SPIFFS: ") + path); // Verbose
  return false; // Indicate file not found or couldn't be served
}


// --- Device Control Functions ---

void goToSleep(int sleepTimeSeconds) {
  #ifdef ENABLE_DEEP_SLEEP
    if (sleepTimeSeconds <= 0) {
        Serial.println("Deep sleep requested with invalid duration. Aborting sleep.");
        return;
    }
    Serial.print(F("Entering deep sleep for ")); Serial.print(sleepTimeSeconds); Serial.println(F(" seconds..."));
    display.clearDisplay();
    display.setTextSize(1); display.setCursor(0, 0);
    display.println(F("Going to sleep..."));
    display.print(F("Wake in ")); display.print(sleepTimeSeconds); display.println(F("s"));
    display.display();
    delay(500); // Allow display to update, message to potentially send

    // ESP8266 Deep Sleep takes time in microseconds (uS)
    // ESP.deepSleep(microseconds, mode)
    // WAKE_RF_DEFAULT wakes with RF calibration (recommended)
    // WAKE_RF_DISABLED wakes faster but WiFi might be less reliable immediately
    // WAKE_NO_RFCAL seems similar to DISABLED
    ESP.deepSleep(sleepTimeSeconds * 1000000ULL, WAKE_RF_DEFAULT);
    delay(100); // Code below deepSleep() will not normally execute
    Serial.println("This should not print if deep sleep worked.");
  #else
    Serial.println(F("Deep sleep requested but ENABLE_DEEP_SLEEP feature is not defined/enabled. Doing nothing."));
  #endif
}

void rebootDevice() {
   Serial.println(F("Rebooting device NOW..."));
   // Optional: Cleanly disconnect MQTT, close files, etc.
   #ifdef ENABLE_MQTT
     if(mqttClient.connected()) {
        mqttClient.publish((String(DEVICE_NAME)+"/status").c_str(), "rebooting_cmd", true); // Let others know why it's going down
        mqttClient.disconnect();
     }
   #endif
   delay(500); // Short delay for messages to potentially send
   yield(); // Yield processor
   ESP.restart();
}

void factoryReset() {
  Serial.println(F("--- Starting Factory Reset ---"));
  // Provide Visual/Audible Feedback
  digitalWrite(LED_PIN, HIGH);
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.println(F("!!! Factory Reset !!!"));
  display.println(F("Erasing Data..."));
  display.display();
  playMelody(ALERT_MELODY, ALERT_DURATIONS, 4); // Alert sound

  // 1. Format Filesystem (SPIFFS)
  Serial.println(F("Formatting SPIFFS... This may take a moment."));
  display.setCursor(0, 20); display.println(F("Formatting FS...")); display.display();
  bool formatOK = SPIFFS.format();
  if (formatOK) {
      Serial.println(F("SPIFFS Format Successful."));
      display.println(F("FS Format OK"));
  } else {
      Serial.println(F("ERROR: SPIFFS Format Failed!"));
      display.println(F("FS Format FAIL!"));
  }
   display.display();
   delay(1000);

  // 2. Reset WiFi Credentials stored by WiFiManager
  Serial.println(F("Resetting WiFi Credentials..."));
  display.setCursor(0, 30); display.println(F("Resetting WiFi...")); display.display();
  // WiFi.disconnect(true); // Also erases SDK config (might include more than just STA creds)
  // WiFiManager provides a specific reset function:
  WiFiManager wifiManager;
  wifiManager.resetSettings(); // Clears stored SSID/Password
  Serial.println(F("WiFi Credentials Reset (via WiFiManager)."));
  display.println(F("WiFi Reset OK")); display.display();
  delay(1000);

  // 3. Clear EEPROM (Optional - if you use EEPROM for custom settings)
  // Only needed if EEPROM stores config *other* than what WiFiManager uses.
  // Serial.println(F("Clearing EEPROM..."));
  // display.setCursor(0, 40); display.println(F("Clearing EEPROM...")); display.display();
  // for (int i = 0; i < EEPROM.length(); i++) { // Use EEPROM.length()
  //   EEPROM.write(i, 0); // Write 0 to each byte
  // }
  // bool commitOK = EEPROM.commit(); // Commit changes
  // if (commitOK) {
  //      Serial.println(F("EEPROM Clear & Commit OK."));
  //      display.println(F("EEPROM Clear OK"));
  // } else {
  //      Serial.println(F("ERROR: EEPROM commit failed."));
  //       display.println(F("EEPROM Commit FAIL"));
  // }
  // display.display();
  // delay(1000);

  // 4. Final actions & Reboot
  Serial.println(F("--- Factory Reset Complete. Rebooting NOW... ---"));
  display.setCursor(0, 50); display.println(F("Reset Complete!")); display.display();
  digitalWrite(LED_PIN, LOW); // Turn off LED
  playTone(3000, 500); // Final beep
  delay(3000); // Wait a bit for user to see message
  ESP.restart(); // Reboot into fresh state
}


// --- Formatting Helpers ---
String formatUptime() {
  unsigned long totalSeconds = millis() / 1000;
  int days = totalSeconds / 86400;        // 86400 seconds in a day
  int hours = (totalSeconds % 86400) / 3600; // Seconds remaining after days / seconds in hour
  int minutes = (totalSeconds % 3600) / 60;  // Seconds remaining after hours / seconds in minute
  int seconds = totalSeconds % 60;           // Seconds remaining

  char buf[30]; // Buffer to hold formatted string (e.g., "123d 23:59:59")
  snprintf(buf, sizeof(buf), "%dd %02d:%02d:%02d", days, hours, minutes, seconds);
  return String(buf);
}

String getModeName(DisplayMode mode) {
  // Simple mapping from enum to human-readable string
  switch(mode){
      case NORMAL: return F("Normal");
      case SMILE: return F("Smile");
      case SAD: return F("Sad");
      case RAGE: return F("Rage");
      case COOL: return F("Cool");
      case SURPRISED: return F("Surprised");
      case SLEEPY: return F("Sleepy");
      case WEATHER: return F("Weather");
      default: return F("Unknown"); // Fallback
  }
}


// --- Sound and Light ---
void playTone(int frequency, int duration) {
    if (frequency <= 0 || duration <= 0) return; // Basic validation
    tone(BUZZER_PIN, frequency, duration);
    // Note: tone() with duration is non-blocking on ESP8266.
    // If you need blocking behaviour, add a delay(duration); after the tone() call.
}

void playMelody(const int melody[], const int durations[], int size) {
   for (int i = 0; i < size; i++) {
     if (melody[i] > 0 && durations[i] > 0) { // Check for valid note/duration
        tone(BUZZER_PIN, melody[i], durations[i]);
        delay(durations[i] * 1.3); // Wait for the note duration plus a small pause
     } else {
        // If frequency or duration is 0, treat as a rest/pause
        delay(durations[i]); // Just wait for the specified duration
     }
   }
   noTone(BUZZER_PIN); // Ensure buzzer is off after melody
}

void blinkLED(int times) {
   int currentState = digitalRead(LED_PIN); // Get current state
   for (int i = 0; i < times * 2; i++) { // Loop twice per blink (ON/OFF)
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle state
        delay(150); // Blink interval
   }
   digitalWrite(LED_PIN, currentState); // Restore original state (usually OFF after setup/error)
}


// --- System Report Generator ---
String generateReport() {
    String html = F("<!DOCTYPE html><html><head>");
    html += F("<title>") + String(DEVICE_NAME) + F(" System Report</title>");
    html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    // Embed CSS directly for self-contained report
    html += F("<style>") + String(MAIN_CSS) + F(" body { background: white; } table td:first-child { font-weight: bold; width: 150px;} </style>");
    html += F("</head><body><div class='container' style='max-width: 700px;'>");
    html += F("<h1>") + String(DEVICE_NAME) + F(" System Report</h1>");
    html += F("<p>Report generated on ") + String(dateString) + F(" at ") + String(timeString) + F("</p>");

    // --- Device Information Card ---
    html += F("<div class='card'><h2>Device Information</h2><table>");
    html += F("<tr><td>Device Name:</td><td>") + String(DEVICE_NAME) + F("</td></tr>");
    html += F("<tr><td>Firmware Version:</td><td>") + String(FIRMWARE_VERSION) + F("</td></tr>");
    html += F("<tr><td>Chip ID (Hex):</td><td>") + String(ESP.getChipId(), HEX) + F("</td></tr>");
    html += F("<tr><td>SDK Version:</td><td>") + String(ESP.getSdkVersion()) + F("</td></tr>");
    html += F("<tr><td>Core Version:</td><td>") + String(ESP.getCoreVersion()) + F("</td></tr>");
    html += F("<tr><td>CPU Frequency:</td><td>") + String(ESP.getCpuFreqMHz()) + F(" MHz</td></tr>");
    html += F("<tr><td>Uptime:</td><td>") + formatUptime() + F(" (") + String(millis()) + F(" ms)</td></tr>");
    html += F("<tr><td>Last Reset Reason:</td><td>") + ESP.getResetReason() + F("</td></tr>");
    html += F("<tr><td>Last Reset Info:</td><td>") + ESP.getResetInfo() + F("</td></tr>");
    html += F("</table></div>");

    // --- Network Information Card ---
    html += F("<div class='card'><h2>Network Status</h2><table>");
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    html += F("<tr><td>WiFi Status:</td><td style='color:") + String(isConnected ? F("#27ae60") : F("#e74c3c")) + F(";'>") + String(isConnected ? F("Connected") : F("Disconnected")) + F("</td></tr>");
    if (isConnected) {
        html += F("<tr><td>SSID:</td><td>") + WiFi.SSID() + F("</td></tr>");
        html += F("<tr><td>IP Address:</td><td>") + WiFi.localIP().toString() + F("</td></tr>");
        html += F("<tr><td>Subnet Mask:</td><td>") + WiFi.subnetMask().toString() + F("</td></tr>");
        html += F("<tr><td>Gateway IP:</td><td>") + WiFi.gatewayIP().toString() + F("</td></tr>");
        html += F("<tr><td>DNS Server IP:</td><td>") + WiFi.dnsIP().toString() + F("</td></tr>");
        html += F("<tr><td>Hostname:</td><td>") + WiFi.hostname() + F("</td></tr>");
        html += F("<tr><td>MAC Address:</td><td>") + WiFi.macAddress() + F("</td></tr>");
        html += F("<tr><td>Signal Strength:</td><td>") + String(WiFi.RSSI()) + F(" dBm</td></tr>");
    }
     html += F("<tr><td>NTP Server:</td><td>") + String(NTP_SERVER) + F("</td></tr>");
     html += F("<tr><td>NTP Last Sync:</td><td>") + String(lastNtpUpdate == 0 ? "Never" : String((millis()-lastNtpUpdate)/1000) + "s ago") + F("</td></tr>");
    html += F("</table></div>");

    // --- MQTT Status Card ---
    #ifdef ENABLE_MQTT
    html += F("<div class='card'><h2>MQTT Status</h2><table>");
    html += F("<tr><td>MQTT Status:</td><td style='color:") + String(mqttConnected ? F("#27ae60") : F("#e74c3c")) + F(";'>") + String(mqttConnected ? F("Connected") : F("Disconnected")) + F("</td></tr>");
    html += F("<tr><td>Broker Address:</td><td>") + String(MQTT_SERVER) + F(":") + String(MQTT_PORT) + F("</td></tr>");
    if (mqttConnected) {
        // html += "<tr><td>Client ID:</td><td>" + String(mqttClient.getClientId()) + "</td></tr>"; // ClientID changes on reconnect
        html += F("<tr><td>Last Publish:</td><td>") + String(lastMqttPublish == 0 ? "Never" : String((millis()-lastMqttPublish)/1000) + "s ago") + F("</td></tr>");
    } else {
        html += F("<tr><td>Last Attempt:</td><td>") + String(lastMqttAttempt == 0 ? "Never" : String((millis()-lastMqttAttempt)/1000) + "s ago") + F("</td></tr>");
    }
     html += F("<tr><td>PubSubClient State:</td><td>") + String(mqttClient.state()) + F("</td></tr>");
    html += F("</table></div>");
    #endif // ENABLE_MQTT

    // --- Current Readings Card ---
    html += F("<div class='card'><h2>Current Readings</h2><table>");
    html += F("<tr><td>Temperature:</td><td>") + (isnan(currentTemp) ? F("--.-") : String(currentTemp, 1)) + F(" &#176;C</td></tr>");
    html += F("<tr><td>Humidity:</td><td>") + (isnan(currentHumidity) ? F("--.-") : String(currentHumidity, 1)) + F(" %</td></tr>");
    html += F("<tr><td>Last Sensor Read:</td><td>") + String(lastSensorUpdate == 0 ? "Never" : String((millis()-lastSensorUpdate)/1000) + "s ago") + F("</td></tr>");
    html += F("<tr><td>Display Mode:</td><td>") + getModeName(currentMode) + F("</td></tr>");
    html += F("<tr><td>Security Locked:</td><td>") + String(lockout ? F("Yes") : F("No")) + (lockout ? F(" (") + String((lockoutEndTime > millis()?(lockoutEndTime - millis()) / 1000:0)) + F("s left)") : "") + F("</td></tr>");
    html += F("<tr><td>Failed Attempts:</td><td>") + String(failedAttempts) + F("</td></tr>");
    html += F("</table></div>");

    // --- System Resources Card ---
    html += F("<div class='card'><h2>System Resources</h2><table>");
    html += F("<tr><td>Free Heap:</td><td>") + String(ESP.getFreeHeap()) + F(" bytes</td></tr>");
    uint32_t maxHeap = ESP.getMaxFreeBlockSize(); // Not total heap, largest contiguous
    html += F("<tr><td>Max Alloc Heap:</td><td>") + String(maxHeap) + F(" bytes</td></tr>");
    uint8_t frag = ESP.getHeapFragmentation(); // Percentage
    html += F("<tr><td>Heap Fragmentation:</td><td>") + String(frag) + F(" %</td></tr>");

    FSInfo fs_info;
    if (SPIFFS.info(fs_info)) {
      html += F("<tr><td>Filesystem Size:</td><td>") + String(fs_info.totalBytes) + F(" bytes</td></tr>");
      html += F("<tr><td>Filesystem Used:</td><td>") + String(fs_info.usedBytes) + F(" bytes (") + String((fs_info.usedBytes*100)/fs_info.totalBytes) + F("%)</td></tr>");
      html += F("<tr><td>FS Block Size:</td><td>") + String(fs_info.blockSize) + F("</td></tr>");
      html += F("<tr><td>FS Page Size:</td><td>") + String(fs_info.pageSize) + F("</td></tr>");
      html += F("<tr><td>FS Max Open Files:</td><td>") + String(fs_info.maxOpenFiles) + F("</td></tr>");
      html += F("<tr><td>FS Max Path Length:</td><td>") + String(fs_info.maxPathLength) + F("</td></tr>");
    } else {
      html += F("<tr><td>Filesystem Info:</td><td>Unavailable</td></tr>");
    }
    html += F("</table></div>");

    // --- Data Logging History Card (Optional) ---
    #ifdef ENABLE_DATA_LOGGING
      html += F("<div class='card'><h2>Logged Data History (Latest ") + String(HISTORY_SIZE) + F(" Points)</h2>");
      html += F("<table border='1' style='width:100%; font-size: 0.8em; border-collapse: collapse;'><thead><tr><th>#</th><th>Timestamp</th><th>Date/Time</th><th>Temp (°C)</th><th>Humid (%)</th></tr></thead><tbody>");
      int loggedCount = 0;
      // Iterate through the buffer starting from the oldest valid entry
      int startIdx = historyWrapped ? historyIndex : 0; // Index where the loop starts (current index if wrapped, 0 otherwise)
      int numEntries = historyWrapped ? HISTORY_SIZE : historyIndex; // How many entries to iterate through

      for (int i = 0; i < numEntries; i++) {
           int currentIdx = (startIdx + i) % HISTORY_SIZE; // Actual index in the circular buffer

            // Only process if the timestamp seems valid (basic check)
            if (history[currentIdx].timestamp > 1000000000) {
               loggedCount++;
               time_t ts = history[currentIdx].timestamp;
               struct tm * tminfo = localtime(&ts);
               char timebuf[25];
               // Format timestamp nicely if localtime() worked
               if (tminfo) {
                   snprintf(timebuf, sizeof(timebuf),"%04d-%02d-%02d %02d:%02d:%02d", tminfo->tm_year + 1900, tminfo->tm_mon + 1, tminfo->tm_mday, tminfo->tm_hour, tminfo->tm_min, tminfo->tm_sec );
               } else {
                   snprintf(timebuf, sizeof(timebuf), "Time Format Error"); // Fallback
               }
               html += F("<tr><td>") + String(numEntries - i) + F("</td>"); // Display as "N minutes ago" style index
               html += F("<td>") + String((unsigned long)ts) + F("</td>");
               html += F("<td>") + String(timebuf) + F("</td>");
               html += F("<td>") + String(history[currentIdx].temperature, 1) + F("</td>");
               html += F("<td>") + String(history[currentIdx].humidity, 1) + F("</td>");
               html += F("</tr>");
           }
      } // end for loop
      if(loggedCount == 0) html += F("<tr><td colspan='5' style='text-align:center;'>No valid data logged yet.</td></tr>");
      html += F("</tbody></table>");
      html += F("<p>History buffer index: ") + String(historyIndex) + F(", Wrapped: ") + String(historyWrapped ? "Yes" : "No") + F("</p>");
      html += F("</div>"); // Close card
    #endif // ENABLE_DATA_LOGGING

    // --- Footer ---
    html += F("<div class='footer'><p>End of Report</p></div>");
    html += F("</div></body></html>"); // Close main container, body, html

    return html;
}