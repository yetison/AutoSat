/*
  Vollständiger, stabiler Code für AutoSat.
  Kompass-Logik auf Adafruit_QMC5883P umgestellt, Handler-Logik vollständig wiederhergestellt.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <ESP8266WiFiGratuitous.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <TinyMPU6050.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h> // Bibliothek aus dem Test-Sketch

#include "index.h"

#define datapin D5
#define motor1 D6
#define motor2 D0
#define UP 1
#define DOWN 2
#define Az_PCB_Correction 180

const char* ssid = "ssid";
const char* password = "ssid-pw";

float Astra_Az = 167, Astra_El = 30.19, El_Offset = 15, Az_Offset = -10.0;
int motorSpeed = 700;
float Azimut = 0, Elevation = 0;
float sAzimut = 0, sElevation = 0;
float dAzimut = 0, dElevation = 0;
float IsRotor = 0, RotorPos = 0;
float compassOffsetX = 0;
float compassOffsetY = 0;
volatile bool isCalibrating = false;
volatile bool startMPUCal = false;
unsigned long calStartTime = 0;
int16_t minX = 32767, maxX = -32768, minY = 32767, maxY = -32768;

bool auto_on = false, update_rotor = false, rotor_changed = false, rotor_off = false;
unsigned long comp_on_time = 0;
int motor_error = 0;
int LED_level = 0;
unsigned long lastMotorAction = 0; // Sperrzeit für Auto-Loop

MPU6050 mpu (Wire);
Adafruit_QMC5883P qmc; // QMC Instanz
ESP8266WebServer server(80);

void runCalibration() {
  if (startMPUCal) {
    isCalibrating = true;
    mpu.Calibrate();
    delay(2000); // KÜNSTLICHE VERZÖGERUNG: Damit das Web-UI den Status "Kalibriert..." sicher sieht
    isCalibrating = false;
    startMPUCal = false;
    Serial.println("MPU Calibrating finished.");
  }
  
  if (!isCalibrating) return;
  
  int16_t x, y, z;
  qmc.getRawMagnetic(&x, &y, &z);
  
  if (x < minX) minX = x;
  if (x > maxX) maxX = x;
  if (y < minY) minY = y;
  if (y > maxY) maxY = y;
  
  delay(10); // Reduce CPU load
  
  if (millis() - calStartTime > 30000) { // 30 Sekunden sammeln
    isCalibrating = false;
    compassOffsetX = (minX + maxX) / 2.0;
    compassOffsetY = (minY + maxY) / 2.0;
    Serial.print("Calib Done. Offsets: "); Serial.print(compassOffsetX); Serial.print(", "); Serial.println(compassOffsetY);
    EEPROM_Write(&compassOffsetX, sizeof(float) * 5);
    EEPROM_Write(&compassOffsetY, sizeof(float) * 6);
    EEPROM.commit();
    Serial.println("Compass Calibrating finished.");
  }
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin(); 
  delay(1000);
  Serial.println("Initialise Compass and MPU...");

  // --- INITIALISIERUNG MIT ADAFRUIT BIBLIOTHEK ---
  if (!qmc.begin()) {
    Serial.println("Failed to find QMC5883P chip");
  } else {
    Serial.println("QMC5883P Found!");
    qmc.setMode(QMC5883P_MODE_NORMAL);
    qmc.setODR(QMC5883P_ODR_50HZ);
    qmc.setOSR(QMC5883P_OSR_4);
    qmc.setDSR(QMC5883P_DSR_2);
    qmc.setRange(QMC5883P_RANGE_8G);
    qmc.setSetResetMode(QMC5883P_SETRESET_ON);
  }
  // ----------------------------------------------

  mpu.Initialize();

  pinMode(datapin, OUTPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, HIGH);
  pinMode(D4, OUTPUT);
  digitalWrite(D4, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.hostname("SatFinder");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP

  
  ArduinoOTA.begin();
  
  server.on("/", handleRoot);
  server.on("/get_data", handleGetData);
  server.on("/on", handleOn);
  server.on("/off", handleOff);
  server.on("/r_off", handleRotorOff);
  server.on("/az_up", handleAzUp);
  server.on("/az_down", handleAzDown);
  server.on("/el_up", handleElUp);
  server.on("/el_down", handleElDown);
  server.on("/rotor_up", handleRotorUp);
  server.on("/rotor_down", handleRotorDown);
  server.on("/rotor_up_step", handleRotorUpStep);
  server.on("/rotor_down_step", handleRotorDownStep);
  server.on("/cal", handleCal);
  server.on("/cal_compass", handleCalCompass);
  server.on("/reset_cal", handleResetCal);
  server.on("/slider1", handleSlider1);
  server.on("/slider2", handleSlider2);
  server.on("/slider3", handleSlider3);
  server.on("/settings", handleSettings);
  server.on("/get_settings", handleGetSettings);
  server.on("/set_settings", handleSetSettings);
  server.onNotFound(handleNotFound);

  server.begin();
  
  EEPROM.begin(128);
  
  float tmp = 0;
  EEPROM_Read(&tmp, 0);
  if ((tmp >= 90) && (tmp <= 270)) Astra_Az = tmp;
  EEPROM_Read(&tmp, sizeof(float));
  if ( (tmp >= 10) && (tmp < 50) ) Astra_El = tmp;
  EEPROM_Read(&tmp, sizeof(float) * 2);
  if (abs(tmp) < 90) El_Offset = tmp;
  EEPROM_Read(&tmp, sizeof(float) * 3);
  if (abs(tmp) < 90) Az_Offset = tmp;
  EEPROM_Read(&tmp, sizeof(float) * 4);
  if ((tmp >= 500) && (tmp < 1024)) motorSpeed = trunc(tmp);
  
  EEPROM_Read(&compassOffsetX, sizeof(float) * 5);
  EEPROM_Read(&compassOffsetY, sizeof(float) * 6);
  
  sAzimut = Astra_Az;
  sElevation = Astra_El;
}

void checkCompass() {
  static unsigned long lastCheck = 0;
  static int16_t lastX = 0, lastY = 0;
  static unsigned long lastChangeTime = millis();

  if (millis() - lastCheck > 2000) { // Check every 2 seconds
    lastCheck = millis();
    
    int16_t x, y, z;
    // Try to read directly
    if (qmc.getRawMagnetic(&x, &y, &z)) {
      // Check if data is frozen
      if (x == lastX && y == lastY && (x != 0 || y != 0)) {
        if (millis() - lastChangeTime > 10000) { // Frozen for 10 seconds
          Serial.println("Compass frozen, forcing re-init...");
          Wire.begin(); // Reset I2C
          qmc.begin();
          qmc.setMode(QMC5883P_MODE_NORMAL);
          qmc.setODR(QMC5883P_ODR_50HZ);
          qmc.setOSR(QMC5883P_OSR_4);
          qmc.setDSR(QMC5883P_DSR_2);
          qmc.setRange(QMC5883P_RANGE_8G);
          qmc.setSetResetMode(QMC5883P_SETRESET_ON);
          lastChangeTime = millis();
        }
      } else {
        lastX = x;
        lastY = y;
        lastChangeTime = millis();
      }
    } else {
      // Sensor not responding at all
      Serial.println("Compass error, re-initializing...");
      Wire.begin();
      qmc.begin();
      // ... (re-init settings) ...
      qmc.setMode(QMC5883P_MODE_NORMAL);
      qmc.setODR(QMC5883P_ODR_50HZ);
      qmc.setOSR(QMC5883P_OSR_4);
      qmc.setDSR(QMC5883P_DSR_2);
      qmc.setRange(QMC5883P_RANGE_8G);
      qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    }
  }
}

void loop(void) {
  server.handleClient();
  ArduinoOTA.handle();
  mpu.Execute();
  runCalibration();

  // --- KOMPASS LESEN MIT BIBLIOTHEK ---
  checkCompass(); // Check compass health
  int16_t x = 0, y = 0, z = 0;
  qmc.getRawMagnetic(&x, &y, &z); // Read directly
  
  sAzimut = Astra_Az + dAzimut;
  sElevation = Astra_El + dElevation;
  
  dAzimut = round(dAzimut * 10) / 10;
  dElevation = round(dElevation * 10) / 10;
  
  sAzimut = round(sAzimut * 10) / 10;
  sElevation = round(sElevation * 10) / 10;
  
  if (rotor_changed && (millis() > comp_on_time )) {
    rotor_changed = false;
    Serial.println("Compass On");
  }
  
  if (!rotor_changed)  {
    static float lastValidAzimut = 0;
    // Only update if we have a valid reading (not all zeros)
    if (x != 0 || y != 0) {
      float x_cal = x - compassOffsetX;
      float y_cal = y - compassOffsetY;
      // Invertieren der Y-Achse für geänderte Kompass-Orientierung
      float currentAzimut = atan2(-y_cal, x_cal) * 180 / PI;
      if (currentAzimut < 0) currentAzimut += 360;
      lastValidAzimut = currentAzimut;
    }
    
    Azimut = lastValidAzimut - Az_PCB_Correction - Az_Offset;
    if (Azimut < 0) Azimut += 360;
    if (Azimut > 360) Azimut -= 360;
  }
  
  Elevation_Calc();

  // Elevation Motor Control (dependent on auto_on)
  if (auto_on && motor_error < 20 && (millis() - lastMotorAction > 500)) {
    float diff = sElevation - Elevation;
    if (abs(diff) > 0.5) { // Deadband auf 0.5° erhöht
      Serial.print("Elevation diff: "); Serial.println(diff);
      if (diff > 0.5) {
        Serial.println("Motor UP called");
        motor(UP);
      }
      else if (diff < -0.5) {
        Serial.println("Motor DOWN called");
        motor(DOWN);
      }
    }
  } else if (motor_error >= 20) {
    static unsigned long lastErrorLog = 0;
    if (millis() - lastErrorLog > 5000) {
      Serial.print("Motor error high: "); Serial.println(motor_error);
      lastErrorLog = millis();
    }
  }

  // Azimuth Motor Control (dependent on auto_on)
  if (auto_on) {
    RotorPos = sAzimut - Azimut;
    if (RotorPos > 70) RotorPos = 70;
    if (RotorPos < -70) RotorPos = -70;
  }

  if ((!rotor_off && abs(RotorPos - IsRotor) > 3) || update_rotor) {
    comp_on_time = millis() + (abs(RotorPos - IsRotor) * 700);
    rotor_changed = true;

    goto_angle(-RotorPos);
    Serial.print("RotorPos: ");
    Serial.println(RotorPos);
    IsRotor = RotorPos;
    update_rotor = false;
  }
  delay(10);
} // End of loop()

void Elevation_Calc() {
  mpu.Execute(); // MPU Daten aktualisieren
  Elevation = -round((mpu.GetAngX() + El_Offset) * 10) / 10;
  if (Elevation < 0 ) Elevation = 0;
}

// --- Handler Funktionen ---
void handleRoot() { server.send(200, "text/html", MAIN_page); }
void handleSettings() { server.send(200, "text/html", Settings_page); }
void handleGetSettings() {
  String Text;
  StaticJsonDocument<300> root;
  root["azimut"] = Astra_Az;
  root["elevation"] = Astra_El;
  root["el_offset"] = El_Offset;
  root["az_offset"] = Az_Offset;
  root["motor_speed"] = motorSpeed;
  serializeJsonPretty(root, Text);
  server.send(200, "text/plain", Text);
}
void handleSetSettings() {
  float tmp;
  if (server.args() > 3) {
    tmp = server.arg(0).toFloat();
    if ((tmp >= 90) && (tmp <= 270)) Astra_Az = tmp;
    tmp = server.arg(1).toFloat();
    if ( (tmp >= 10) && (tmp <= 50) ) Astra_El = tmp;
    tmp = server.arg(2).toFloat();
    if (abs(tmp) <= 90) El_Offset = tmp;
    tmp = server.arg(3).toFloat();
    if (abs(tmp) <= 90) Az_Offset = tmp;
    tmp = server.arg(4).toFloat();
    if ((tmp >= 500) && (tmp < 1024)) motorSpeed = trunc(tmp);
  }
  EEPROM_Write(&Astra_Az, 0);
  EEPROM_Write(&Astra_El, sizeof(float));
  EEPROM_Write(&El_Offset, sizeof(float) * 2);
  EEPROM_Write(&Az_Offset, sizeof(float) * 3);
  EEPROM_Write(&tmp, sizeof(float) * 4);
  EEPROM.commit();
  server.send(200, "text/html");
}
void handleGetData() {
  String Text;
  StaticJsonDocument<512> root;
  root["azimut"] = Azimut;
  root["elevation"] = Elevation;
  root["s_azimut"] = sAzimut;
  root["s_elevation"] = sElevation;
  root["d_azimut"] = dAzimut;
  root["d_elevation"] = dElevation;
  root["rotor"] = IsRotor;
  root["led_level"] = LED_level;
  root["isCalibrating"] = isCalibrating;
  root["el_offset"] = El_Offset;
  root["az_offset"] = Az_Offset;
  root["motor_speed"] = motorSpeed;
  root["astra_el"] = Astra_El;
  if (auto_on) root["state"] = "On"; else root["state"] = "Off";
  if (auto_on && rotor_off) root["state"] = "R-Off";
  serializeJsonPretty(root, Text);
  server.send(200, "text/plain", Text);
}
void handleCal() { 
  Serial.println("MPU Calibrating started...");
  isCalibrating = true; // Status für Web-UI setzen
  
  mpu.Calibrate(); 
  
  isCalibrating = false; // Status zurücksetzen
  Serial.println("MPU Calibrating finished.");
  server.send(200, "text/plain", "Done!"); 
}
void handleCalCompass() {
  Serial.println("Compass Calibrating started...");
  isCalibrating = true;
  calStartTime = millis();
  minX = 32767; maxX = -32768; minY = 32767; maxY = -32768;
  server.send(200, "text/plain", "Calibrating...");
}
void handleResetCal() {
  Serial.println("Resetting Calibration...");
  compassOffsetX = 0;
  compassOffsetY = 0;
  EEPROM_Write(&compassOffsetX, sizeof(float) * 5);
  EEPROM_Write(&compassOffsetY, sizeof(float) * 6);
  EEPROM.commit();
  server.send(200, "text/plain", "Reset Done!");
}
void handleAzDown() { dAzimut -= 0.5; update_rotor = true; server.send(200, "text/html"); }
void handleAzUp() { dAzimut += 0.5; update_rotor = true; server.send(200, "text/html"); }
void handleElDown() { 
  motor_error = 0;
  dElevation -= 0.1; // Manuelle Korrektur anpassen
  motor(DOWN); 
  server.send(200, "text/html"); 
}
void handleElUp() { 
  motor_error = 0;
  dElevation += 0.1; // Manuelle Korrektur anpassen
  motor(UP);
  server.send(200, "text/html"); 
}
void handleRotorDown() { RotorPos -= 1; update_rotor = true; server.send(200, "text/html"); }
void handleRotorUp() { RotorPos += 1; update_rotor = true; server.send(200, "text/html"); }
void handleRotorDownStep() {
  noInterrupts();
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x69);
  write_byte_with_parity(0xfe);
  interrupts();
  server.send(200, "text/html");
  comp_on_time = millis() + 1000;
  rotor_changed = true;
  IsRotor = IsRotor + 1.0 / 8.0;
}
void handleRotorUpStep() {
  noInterrupts();
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x68);
  write_byte_with_parity(0xfe);
  interrupts();
  server.send(200, "text/html");
  comp_on_time = millis() + 1000;
  rotor_changed = true;
  IsRotor = IsRotor - 1.0 / 8.0;
}
void handleOn() { digitalWrite(D4, LOW); server.send(200, "text/html"); auto_on = true; rotor_off = false; motor_error = 0; }
void handleOff() {
  auto_on = false; rotor_off = true; digitalWrite(D4, HIGH); server.send(200, "text/html");
  noInterrupts();
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x60);
  interrupts();
  comp_on_time = 0;
  digitalWrite(motor2, HIGH);
  digitalWrite(motor1, HIGH);
}
void handleRotorOff() {
  rotor_off = true;
  auto_on = false;
  server.send(200, "text/html");
  noInterrupts();
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x60);
  interrupts();
  comp_on_time = 0;
  digitalWrite(motor2, HIGH);
  digitalWrite(motor1, HIGH);
}
void handleSlider1() { if (server.args() > 0) { dAzimut = server.arg(0).toFloat(); } server.send(200, "text/html"); }
void handleSlider2() { 
  if (server.hasArg("level")) { 
    dElevation = server.arg("level").toFloat(); 
    Serial.print("Slider2 updated: ");
    Serial.println(dElevation);
  } else {
    Serial.println("Slider2 update failed: no level argument");
  }                
  server.send(200, "text/html"); 
}
void handleSlider3() { if (server.args() > 0) { RotorPos = server.arg(0).toFloat(); update_rotor = true; } server.send(200, "text/html"); }
void handleNotFound() { server.send(404, "text/plain", "File Not Found\n\n"); }
void EEPROM_Write(float *num, int MemPos) {
  byte ByteArray[4];
  memcpy(ByteArray, num, 4);
  for (int x = 0; x < 4; x++) { EEPROM.write((MemPos * 4) + x, ByteArray[x]); }
}
void EEPROM_Read(float *num, int MemPos) {
  byte ByteArray[4];
  for (int x = 0; x < 4; x++) { ByteArray[x] = EEPROM.read((MemPos * 4) + x); }
  memcpy(num, ByteArray, 4);
}
void write0() {
  for (int i = 1; i <= 22; i++) { digitalWrite(datapin, HIGH); delayMicroseconds(21); digitalWrite(datapin, LOW); delayMicroseconds(21); }
  delayMicroseconds(500);
}
void write1() {
  for (int i = 1; i <= 11; i++) { digitalWrite(datapin, HIGH); delayMicroseconds(21); digitalWrite(datapin, LOW); delayMicroseconds(21); }
  delayMicroseconds(1000);
}
bool parity_even_bit(byte x) {
  unsigned int count = 0, i, b = 1;
  for (i = 0; i < 8; i++) { if ( x & (b << i) ) { count++; } }
  return (count % 2) == 0;
}
void write_parity(byte x) { if (parity_even_bit(x)) write0(); else write1(); }
void write_byte(byte x) { for (int j = 7; j >= 0; j--) { if (x & (1 << j)) write1(); else write0(); } }
void write_byte_with_parity(byte x) { write_byte(x); write_parity(x); }
void goto_angle(float a) {
  byte n1, d1, d2;
  int a16;
  if (a < -75.0) a = -75;
  if (a > 75.0) a = 75;
  
  // set the sign nibble in n1 to D0 (East) or E0 (West).
  if (a < 0) {
    n1 = 0xE0; // West
  } else {
    n1 = 0xD0; // East
  }
  
  a16 = (int) (16.0 * abs(a) + 0.5);
  d2 = a16 & 0xFF;
  d1 = n1 | ((a16 & 0xF00) >> 8);
  
  noInterrupts();
  // Clear Limits command to clear potential software locks
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x63);
  write_byte_with_parity(0x00);
  interrupts();
  delay(200); // Short delay for the rotor to process
  
  noInterrupts();
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x6E); // GoToAngle
  write_byte_with_parity(d1);
  write_byte_with_parity(d2);
  interrupts();
}

void motor(int direction) {
  if (direction == UP) {
    digitalWrite(motor2, LOW);
    analogWrite(motor1, motorSpeed);
  } else if (direction == DOWN) {
    digitalWrite(motor1, LOW);
    analogWrite(motor2, motorSpeed - 500);
  }
  delay(150); // Motor kurz bewegen
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, HIGH);
  lastMotorAction = millis(); // Sperre setzen
}
