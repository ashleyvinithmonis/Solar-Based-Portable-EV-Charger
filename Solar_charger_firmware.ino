#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// ============================================================
// LCD / WIFI
// ============================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences prefs;
WebServer server(80);

String ap_ssid;
String ap_password;
String welcomeMsg;

// ============================================================
// ESP32 PINS
// ============================================================

#define SOURCE_VOLT_PIN 34
#define LOAD_VOLT_PIN   35
#define CURRENT_PIN     32

// ============================================================
// BLE UUIDs
// KEEP THESE EXACTLY THE SAME
// ============================================================

#define SERVICE_UUID \
"4fa4c201-1fb2-45fe-a22c-a54179531c9b"

#define CHARACTERISTIC_UUID \
"beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ============================================================
// BLE VARIABLES
// ============================================================

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

bool deviceConnected = false;

// ============================================================
// SENSOR VALUES
// ============================================================

float sourceVoltage = 0.0;
float loadVoltage = 0.0;
float currentDraw = 0.0;
float wattage = 0.0;
float wattHours = 0.0;

// ============================================================
// CHARGING TIME
// ============================================================

unsigned long chargingStartMillis = 0;
unsigned long lastEnergyCalcTime = 0;

unsigned long elapsedSeconds = 0;

bool isCharging = false;

// ============================================================
// PEAK VALUES
// ============================================================

float peakSourceVolt = 0.0;
float peakLoadVolt = 0.0;
float peakCurrent = 0.0;
float peakWattage = 0.0;

// ============================================================
// LCD
// ============================================================

unsigned long lastLcdUpdate = 0;

int lcdDisplayScreen = 0;

// ============================================================
// VOLTAGE SENSOR SETTINGS
// ============================================================

// Common 0-25 V voltage sensor module
// Nominal ratio = 5:1
const float VOLTAGE_SENSOR_RATIO = 5.0;

// ------------------------------------------------------------
// ZERO OFFSET
// ------------------------------------------------------------
//
// With source/load disconnected and sensor negative connected
// to ESP32 GND, you measured:
//
// Solar = 0.71 V
// Load  = 0.71 V
//
// Therefore subtract 0.71 V from each calculated reading.
// ------------------------------------------------------------

const float SOURCE_ZERO_OFFSET = 0.71;
const float LOAD_ZERO_OFFSET   = 0.71;

// Calibration multipliers.
// Keep at 1.000 initially.
// We can calibrate these later against your multimeter.
const float SOURCE_CALIBRATION = 1.000;
const float LOAD_CALIBRATION   = 1.000;

// ============================================================
// ACS712-5A
// ============================================================

// ACS712-5A sensitivity = 185 mV/A
const float ACS712_SENSITIVITY = 0.185;

// Minimum current considered as actual charging current.
const float CURRENT_THRESHOLD = 0.05;

// Current zero voltage is measured during startup.
float currentZeroVoltage = 2.50;

// ============================================================
// ADC
// ============================================================

const int VOLTAGE_SAMPLES = 30;
const int CURRENT_SAMPLES = 100;

// ============================================================
// BLE CALLBACKS
// ============================================================

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer* pServer)
  {
    deviceConnected = true;

    Serial.println();
    Serial.println("================================");
    Serial.println("BLE: CONNECTED");
    Serial.println("================================");
  }

  void onDisconnect(BLEServer* pServer)
  {
    deviceConnected = false;

    Serial.println();
    Serial.println("BLE: DISCONNECTED");

    delay(100);

    pServer->getAdvertising()->start();

    Serial.println("BLE: ADVERTISING RESTARTED");
  }
};

// ============================================================
// LCD TIME FORMAT
// ============================================================

String formatLCDTime(unsigned long totalSec)
{
  int hrs = totalSec / 3600;
  int mins = (totalSec % 3600) / 60;
  int secs = totalSec % 60;

  char buf[9];

  snprintf(
    buf,
    sizeof(buf),
    "%02d:%02d:%02d",
    hrs,
    mins,
    secs
  );

  return String(buf);
}

// ============================================================
// AVERAGE ADC MILLIVOLTS
// ============================================================

float readAverageMilliVolts(
  int pin,
  int samples
)
{
  uint32_t total = 0;

  for (int i = 0; i < samples; i++)
  {
    total += analogReadMilliVolts(pin);

    delayMicroseconds(150);
  }

  return
    (float)total /
    (float)samples;
}

// ============================================================
// AVERAGE RAW ADC
// ============================================================

float readAverageADC(
  int pin,
  int samples
)
{
  uint32_t total = 0;

  for (int i = 0; i < samples; i++)
  {
    total += analogRead(pin);

    delayMicroseconds(150);
  }

  return
    (float)total /
    (float)samples;
}

// ============================================================
// ACS712 ZERO CALIBRATION
// ============================================================

void calibrateCurrent()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("ACS712 ZERO CALIBRATION");
  Serial.println("================================");

  Serial.println(
    "Make sure NO charging current is flowing."
  );

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ACS712 Setup");

  lcd.setCursor(0, 1);
  lcd.print("No Current");

  delay(2000);

  float totalVoltage = 0.0;

  for (int i = 0; i < 200; i++)
  {
    totalVoltage +=
      analogReadMilliVolts(
        CURRENT_PIN
      );

    delay(5);
  }

  currentZeroVoltage =
    (totalVoltage / 200.0) / 1000.0;

  Serial.print(
    "ACS712 zero voltage = "
  );

  Serial.print(
    currentZeroVoltage,
    4
  );

  Serial.println(" V");

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ACS Zero:");

  lcd.setCursor(0, 1);
  lcd.print(
    currentZeroVoltage,
    2
  );

  lcd.print("V");

  delay(1500);
}

// ============================================================
// SOURCE VOLTAGE
// ============================================================

float readSourceVoltage()
{
  float sensorVoltage =
    readAverageMilliVolts(
      SOURCE_VOLT_PIN,
      VOLTAGE_SAMPLES
    ) / 1000.0;

  // Convert sensor output to actual voltage
  float actualVoltage =
    sensorVoltage *
    VOLTAGE_SENSOR_RATIO;

  // Apply calibration
  actualVoltage *=
    SOURCE_CALIBRATION;

  // Remove measured zero offset
  actualVoltage -=
    SOURCE_ZERO_OFFSET;

  // Remove negative/very small readings
  if (actualVoltage <= 0.10)
  {
    return 0.0;
  }

  // Protect against invalid readings
  if (!isfinite(actualVoltage))
  {
    return 0.0;
  }

  // Voltage sensor is intended for approximately 0-25 V.
  if (actualVoltage > 25.0)
  {
    return 0.0;
  }

  return actualVoltage;
}

// ============================================================
// LOAD VOLTAGE
// ============================================================

float readLoadVoltage()
{
  float sensorVoltage =
    readAverageMilliVolts(
      LOAD_VOLT_PIN,
      VOLTAGE_SAMPLES
    ) / 1000.0;

  // Convert sensor output to actual voltage
  float actualVoltage =
    sensorVoltage *
    VOLTAGE_SENSOR_RATIO;

  // Apply calibration
  actualVoltage *=
    LOAD_CALIBRATION;

  // Remove measured zero offset
  actualVoltage -=
    LOAD_ZERO_OFFSET;

  // Remove negative/very small readings
  if (actualVoltage <= 0.10)
  {
    return 0.0;
  }

  if (!isfinite(actualVoltage))
  {
    return 0.0;
  }

  if (actualVoltage > 25.0)
  {
    return 0.0;
  }

  return actualVoltage;
}

// ============================================================
// ACS712 CURRENT
// ============================================================

float readCurrent()
{
  float sensorVoltage =
    readAverageMilliVolts(
      CURRENT_PIN,
      CURRENT_SAMPLES
    ) / 1000.0;

  float difference =
    sensorVoltage -
    currentZeroVoltage;

  float current =
    difference /
    ACS712_SENSITIVITY;

  // Remove small noise
  if (fabs(current) < CURRENT_THRESHOLD)
  {
    return 0.0;
  }

  // This project measures charging current.
  // Negative values are treated as zero.
  if (current < 0.0)
  {
    current = 0.0;
  }

  // Prevent invalid values
  if (!isfinite(current))
  {
    return 0.0;
  }

  // ACS712-5A nominal maximum
  if (current > 5.5)
  {
    return 0.0;
  }

  return current;
}

// ============================================================
// WIFI JSON
// ============================================================

void handleJsonData()
{
  String json =
    "{\"source\":" +
    String(sourceVoltage, 2) +

    ",\"load\":" +
    String(loadVoltage, 2) +

    ",\"current\":" +
    String(currentDraw, 2) +

    ",\"wattage\":" +
    String(wattage, 2) +

    ",\"wh\":" +
    String(wattHours, 3) +

    ",\"time\":\"" +
    formatLCDTime(elapsedSeconds) +

    "\",\"isCharging\":" +
    String(
      isCharging
      ? "true"
      : "false"
    ) +

    ",\"pkSolar\":" +
    String(peakSourceVolt, 1) +

    ",\"pkCurr\":" +
    String(peakCurrent, 1) +

    ",\"pkWatt\":" +
    String(peakWattage, 1) +

    "}";

  server.send(
    200,
    "application/json",
    json
  );
}

// ============================================================
// SAVE SETTINGS
// ============================================================

void handleSaveSettings()
{
  if (
    server.hasArg("ssid") &&
    server.arg("ssid").length() > 0
  )
  {
    ap_ssid =
      server.arg("ssid");

    prefs.putString(
      "ssid",
      ap_ssid
    );
  }

  if (
    server.hasArg("pass") &&
    server.arg("pass").length() >= 8
  )
  {
    ap_password =
      server.arg("pass");

    prefs.putString(
      "pass",
      ap_password
    );
  }

  if (server.hasArg("welcome"))
  {
    welcomeMsg =
      server.arg("welcome");

    prefs.putString(
      "welcome",
      welcomeMsg
    );
  }

  server.send(
    200,
    "text/html",
    "<html><body style='background:#0F172A;color:#FFF;text-align:center;padding-top:50px;'><h2>Saved! Rebooting...</h2></body></html>"
  );

  delay(2000);

  ESP.restart();
}

// ============================================================
// WIFI WEB PAGE
// ============================================================

void handleRoot()
{
  String html;

  html.reserve(2500);

  html += F(
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1.0'>"
    "<title>Solar Charger</title>"

    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0;}"
    "body{font-family:sans-serif;background:#0F172A;color:#F8FAFC;padding:20px;text-align:center;}"
    ".card{background:#1E293B;border-radius:12px;padding:15px;margin-bottom:10px;text-align:left;border:1px solid #334155;}"
    ".lbl{font-size:11px;font-weight:700;color:#94A3B8;text-transform:uppercase;}"
    ".val{font-size:24px;font-weight:800;color:#22C55E;}"
    "input{width:100%;padding:10px;margin:6px 0 12px 0;background:#0F172A;border:1px solid #334155;border-radius:6px;color:#FFF;}"
    "button{width:100%;padding:12px;background:#22C55E;border:none;border-radius:6px;color:#FFF;font-weight:700;cursor:pointer;}"
    ".pass-wrap{position:relative;}"
    ".toggle{position:absolute;right:10px;top:14px;cursor:pointer;font-size:11px;color:#94A3B8;}"
    "</style>"

    "</head><body>"

    "<h2>Car Solar Charger</h2>"

    "<div id='st' style='color:#22C55E;font-size:12px;margin:8px 0;'>○ READY</div>"

    "<div class='card'>"
    "<div class='lbl'>Solar Input</div>"
    "<div class='val'><span id='vs'>0.00</span> V</div>"
    "</div>"

    "<div class='card'>"
    "<div class='lbl'>Load Output</div>"
    "<div class='val'><span id='vl'>0.00</span> V</div>"
    "</div>"

    "<div class='card'>"
    "<div class='lbl'>Current</div>"
    "<div class='val'><span id='i'>0.00</span> A</div>"
    "</div>"

    "<div class='card'>"
    "<div class='lbl'>Power</div>"
    "<div class='val'><span id='w'>0.00</span> W</div>"
    "<div style='font-size:12px;color:#CBD5E1;'>Total: <b id='wh'>0.000 Wh</b></div>"
    "</div>"

    "<div class='card'>"
    "<div class='lbl'>Charging Time</div>"
    "<div class='val' id='t'>00:00:00</div>"
    "</div>"

    "<div class='card'>"
    "<h3 style='font-size:13px;color:#94A3B8;margin-bottom:8px;'>SETTINGS</h3>"

    "<form action='/save' method='POST'>"

    "<label style='font-size:11px;'>Wi-Fi Name</label>"
    "<input type='text' name='ssid' value='"
  );

  html += ap_ssid;

  html += F(
    "'>"

    "<label style='font-size:11px;'>Wi-Fi Password</label>"

    "<div class='pass-wrap'>"

    "<input type='password' id='p' name='pass' value='"
  );

  html += ap_password;

  html += F(
    "'>"

    "<span class='toggle' onclick='tP()'>SHOW</span>"

    "</div>"

    "<label style='font-size:11px;'>LCD Welcome Message</label>"

    "<input type='text' name='welcome' maxlength='16' value='"
  );

  html += welcomeMsg;

  html += F(
    "'>"

    "<button type='submit'>Save & Restart</button>"

    "</form>"

    "</div>"

    "<script>"

    "function u(){"

    "fetch('/data')"
    ".then(r=>r.json())"
    ".then(d=>{"

    "document.getElementById('vs').innerText=d.source.toFixed(2);"

    "document.getElementById('vl').innerText=d.load.toFixed(2);"

    "document.getElementById('i').innerText=d.current.toFixed(2);"

    "document.getElementById('w').innerText=d.wattage.toFixed(2);"

    "document.getElementById('wh').innerText=d.wh.toFixed(3)+' Wh';"

    "document.getElementById('t').innerText=d.time;"

    "document.getElementById('st').innerText="
    "d.isCharging?'● CHARGING':'○ READY';"

    "});"

    "}"

    "setInterval(u,1500);"

    "function tP(){"

    "var p=document.getElementById('p');"

    "p.type=p.type==='password'?'text':'password';"

    "}"

    "</script>"

    "</body></html>"
  );

  server.send(
    200,
    "text/html",
    html
  );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "       SOLAR BASED EV CHARGER"
  );
  Serial.println(
    "========================================"
  );
  Serial.println();

  // ==========================================================
  // ADC
  // ==========================================================

  analogReadResolution(12);

  analogSetPinAttenuation(
    SOURCE_VOLT_PIN,
    ADC_11db
  );

  analogSetPinAttenuation(
    LOAD_VOLT_PIN,
    ADC_11db
  );

  analogSetPinAttenuation(
    CURRENT_PIN,
    ADC_11db
  );

  pinMode(
    SOURCE_VOLT_PIN,
    INPUT
  );

  pinMode(
    LOAD_VOLT_PIN,
    INPUT
  );

  pinMode(
    CURRENT_PIN,
    INPUT
  );

  // ==========================================================
  // PREFERENCES
  // ==========================================================

  prefs.begin(
    "charger_cfg",
    false
  );

  ap_ssid =
    prefs.getString(
      "ssid",
      "Car Solar Charger"
    );

  ap_password =
    prefs.getString(
      "pass",
      "12345678"
    );

  welcomeMsg =
    prefs.getString(
      "welcome",
      "Car Solar"
    );

  // ==========================================================
  // LCD
  // ==========================================================

  lcd.init();

  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(welcomeMsg);

  delay(2000);

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Initialising");

  lcd.setCursor(0, 1);
  lcd.print("System...");

  delay(1500);

  // ==========================================================
  // ACS712 CALIBRATION
  // ==========================================================

  calibrateCurrent();

  // ==========================================================
  // WIFI ACCESS POINT
  // ==========================================================

  WiFi.mode(WIFI_AP);

  bool wifiStarted =
    WiFi.softAP(
      ap_ssid.c_str(),
      ap_password.c_str()
    );

  if (wifiStarted)
  {
    Serial.println(
      "WiFi AP started."
    );

    Serial.print("SSID: ");
    Serial.println(ap_ssid);

    Serial.print("IP: ");
    Serial.println(
      WiFi.softAPIP()
    );
  }
  else
  {
    Serial.println(
      "WiFi AP FAILED"
    );
  }

  server.on(
    "/",
    handleRoot
  );

  server.on(
    "/data",
    handleJsonData
  );

  server.on(
    "/save",
    HTTP_POST,
    handleSaveSettings
  );

  server.begin();

  Serial.println(
    "Web server started."
  );

  // ==========================================================
  // BLE
  // ==========================================================

  Serial.println();
  Serial.println(
    "Starting BLE..."
  );

  BLEDevice::init(
    "Car Solar Charger"
  );

  pServer =
    BLEDevice::createServer();

  pServer->setCallbacks(
    new MyServerCallbacks()
  );

  BLEService* pService =
    pServer->createService(
      SERVICE_UUID
    );

  pCharacteristic =
    pService->createCharacteristic(
      CHARACTERISTIC_UUID,

      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );

  pCharacteristic->addDescriptor(
    new BLE2902()
  );

  // Initial BLE value
  pCharacteristic->setValue(
    "0.00,0.00,0.00,0.00,0.000,0"
  );

  pService->start();

  BLEAdvertising* pAdvertising =
    BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(
    SERVICE_UUID
  );

  pAdvertising->setScanResponse(
    true
  );

  pAdvertising->setMinPreferred(
    0x06
  );

  pAdvertising->setMinPreferred(
    0x12
  );

  pServer->
    getAdvertising()->
    start();

  Serial.println(
    "BLE started."
  );

  Serial.println(
    "BLE name: Car Solar Charger"
  );

  Serial.print(
    "Service UUID: "
  );

  Serial.println(
    SERVICE_UUID
  );

  Serial.print(
    "Characteristic UUID: "
  );

  Serial.println(
    CHARACTERISTIC_UUID
  );

  // ==========================================================
  // ENERGY
  // ==========================================================

  lastEnergyCalcTime =
    millis();

  // ==========================================================
  // INITIAL VALUES
  // ==========================================================

  sourceVoltage = 0.0;
  loadVoltage = 0.0;
  currentDraw = 0.0;
  wattage = 0.0;

  wattHours = 0.0;

  elapsedSeconds = 0;

  chargingStartMillis = 0;

  isCharging = false;

  // ==========================================================
  // READY SCREEN
  // ==========================================================

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  lcd.setCursor(0, 1);
  lcd.print("BLE + WiFi ON");

  delay(2000);

  lcd.clear();

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "SYSTEM READY"
  );
  Serial.println(
    "========================================"
  );
  Serial.println();

  Serial.println(
    "Source zero offset: 0.71 V"
  );

  Serial.println(
    "Load zero offset  : 0.71 V"
  );

  Serial.println(
    "Waiting for sensors..."
  );

  Serial.println(
    "Waiting for BLE connection..."
  );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // WIFI
  // ==========================================================

  server.handleClient();

  // ==========================================================
  // READ SENSORS
  // ==========================================================

  sourceVoltage =
    readSourceVoltage();

  loadVoltage =
    readLoadVoltage();

  currentDraw =
    readCurrent();

  // ==========================================================
  // CURRENT THRESHOLD
  // ==========================================================

  if (
    currentDraw <
    CURRENT_THRESHOLD
  )
  {
    currentDraw = 0.0;
  }

  // ==========================================================
  // POWER
  // ==========================================================

  if (
    currentDraw >=
      CURRENT_THRESHOLD
    &&
    loadVoltage > 0.10
  )
  {
    wattage =
      loadVoltage *
      currentDraw;
  }
  else
  {
    wattage = 0.0;
  }

  // ==========================================================
  // ENERGY
  // ==========================================================

  unsigned long now =
    millis();

  float hoursPassed =
    (
      now -
      lastEnergyCalcTime
    )
    /
    3600000.0;

  if (
    currentDraw >=
    CURRENT_THRESHOLD
  )
  {
    wattHours +=
      wattage *
      hoursPassed;
  }

  lastEnergyCalcTime =
    now;

  // ==========================================================
  // CHARGING TIME
  // ==========================================================

  if (
    currentDraw >=
    CURRENT_THRESHOLD
  )
  {
    if (!isCharging)
    {
      isCharging = true;

      // Resume from previous elapsed time.
      chargingStartMillis =
        millis() -
        (
          elapsedSeconds *
          1000UL
        );
    }

    elapsedSeconds =
      (
        millis() -
        chargingStartMillis
      )
      /
      1000UL;
  }
  else
  {
    isCharging = false;
  }

  // ==========================================================
  // PEAK SOURCE VOLTAGE
  // ==========================================================

  if (
    sourceVoltage >
    peakSourceVolt
  )
  {
    peakSourceVolt =
      sourceVoltage;
  }

  // ==========================================================
  // PEAK LOAD VOLTAGE
  // ==========================================================

  if (
    loadVoltage >
    peakLoadVolt
  )
  {
    peakLoadVolt =
      loadVoltage;
  }

  // ==========================================================
  // PEAK CURRENT
  // ==========================================================

  if (
    currentDraw >
    peakCurrent
  )
  {
    peakCurrent =
      currentDraw;
  }

  // ==========================================================
  // PEAK POWER
  // ==========================================================

  if (
    wattage >
    peakWattage
  )
  {
    peakWattage =
      wattage;
  }

  // ==========================================================
  // SERIAL DEBUG
  // ==========================================================

  static unsigned long lastSerial =
    0;

  if (
    millis() -
    lastSerial >=
    1000
  )
  {
    lastSerial =
      millis();

    Serial.println();
    Serial.println(
      "----------- SENSOR DATA -----------"
    );

    Serial.print(
      "Source Voltage : "
    );

    Serial.print(
      sourceVoltage,
      2
    );

    Serial.println(" V");

    Serial.print(
      "Load Voltage   : "
    );

    Serial.print(
      loadVoltage,
      2
    );

    Serial.println(" V");

    Serial.print(
      "Current        : "
    );

    Serial.print(
      currentDraw,
      2
    );

    Serial.println(" A");

    Serial.print(
      "Power          : "
    );

    Serial.print(
      wattage,
      2
    );

    Serial.println(" W");

    Serial.print(
      "Energy         : "
    );

    Serial.print(
      wattHours,
      3
    );

    Serial.println(" Wh");

    Serial.print(
      "Charging Time  : "
    );

    Serial.println(
      formatLCDTime(
        elapsedSeconds
      )
    );

    Serial.print(
      "BLE Connected  : "
    );

    Serial.println(
      deviceConnected
      ? "YES"
      : "NO"
    );

    Serial.println(
      "-----------------------------------"
    );
  }

  // ==========================================================
  // BLE TELEMETRY
  // ==========================================================

  if (
    deviceConnected &&
    pCharacteristic != NULL
  )
  {
    String telemetryCSV =
      String(
        sourceVoltage,
        2
      )
      + ","
      +
      String(
        loadVoltage,
        2
      )
      + ","
      +
      String(
        currentDraw,
        2
      )
      + ","
      +
      String(
        wattage,
        2
      )
      + ","
      +
      String(
        wattHours,
        3
      )
      + ","
      +
      String(
        elapsedSeconds
      );

    pCharacteristic->setValue(
      telemetryCSV.c_str()
    );

    pCharacteristic->notify();

    Serial.print(
      "BLE TX: "
    );

    Serial.println(
      telemetryCSV
    );
  }

  // ==========================================================
  // LCD
  // ==========================================================

  if (
    millis() -
    lastLcdUpdate >=
    2500
  )
  {
    lastLcdUpdate =
      millis();

    lcd.clear();

    switch (
      lcdDisplayScreen
    )
    {
      // ------------------------------------------------------
      // SCREEN 0
      // ------------------------------------------------------

      case 0:

        lcd.setCursor(0, 0);

        lcd.print("Solar:");

        lcd.print(
          sourceVoltage,
          2
        );

        lcd.print("V");

        lcd.setCursor(0, 1);

        lcd.print("Load :");

        lcd.print(
          loadVoltage,
          2
        );

        lcd.print("V");

        break;

      // ------------------------------------------------------
      // SCREEN 1
      // ------------------------------------------------------

      case 1:

        lcd.setCursor(0, 0);

        lcd.print("Curr:");

        lcd.print(
          currentDraw,
          2
        );

        lcd.print("A");

        lcd.setCursor(0, 1);

        lcd.print("Power:");

        lcd.print(
          wattage,
          2
        );

        lcd.print("W");

        break;

      // ------------------------------------------------------
      // SCREEN 2
      // ------------------------------------------------------

      case 2:

        lcd.setCursor(0, 0);

        lcd.print("Energy:");

        lcd.print(
          wattHours,
          2
        );

        lcd.print("Wh");

        lcd.setCursor(0, 1);

        lcd.print("Time:");

        lcd.print(
          formatLCDTime(
            elapsedSeconds
          )
        );

        break;

      // ------------------------------------------------------
      // SCREEN 3
      // ------------------------------------------------------

      case 3:

        lcd.setCursor(0, 0);

        lcd.print("PkP:");

        lcd.print(
          peakWattage,
          1
        );

        lcd.print("W");

        lcd.setCursor(0, 1);

        lcd.print("PkI:");

        lcd.print(
          peakCurrent,
          1
        );

        lcd.print("A");

        break;

      // ------------------------------------------------------
      // SCREEN 4
      // ------------------------------------------------------

      case 4:

        lcd.setCursor(0, 0);

        lcd.print("BLE:");

        if (deviceConnected)
        {
          lcd.print("CONNECTED");
        }
        else
        {
          lcd.print("WAITING");
        }

        lcd.setCursor(0, 1);

        if (deviceConnected)
        {
          lcd.print("Phone Connected");
        }
        else
        {
          lcd.print("Searching...");
        }

        break;
    }

    lcdDisplayScreen =
      (
        lcdDisplayScreen +
        1
      )
      %
      5;
  }

  delay(100);
}