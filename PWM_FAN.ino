// Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SimpleTimer.h>
#include <PID_v1.h>

// Webserver
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>

// FAN
#define tachInputPIN 5  //D1             
#define pwmOutputPIN 14 //D5   
#define RPMMax 2000
#define RPM_UPDATE 2500

const long minTemp = 20;
const long maxTemp = 45;

unsigned long TimeStamp=0;
volatile unsigned int NumberOfPulses; 
int LastReading, rpm, rpm_percent;

// GPIO where the DS18B20 is connected to
#define ONE_WIRE_BUS D6  
DeviceAddress DS18B20;
bool debug = 0;
int deviceCount = 0; 

float usedTemp, sensor1Temp, sensor2Temp; 
int fanSpeedPercent;

double PID_SP, PID_In, PID_Out;
PID myPID(&PID_In, &PID_Out, &PID_SP,2,5,1, REVERSE);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

SimpleTimer timer;

// Replace with your network credentials
const char* ssid = "xxxxx";
const char* password = "xxxxxx";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

//Sensor 1 : 0x28, 0xAC, 0x40, 0xF1, 0x16, 0x13, 0x01, 0x54
//Sensor 2 : 0x28, 0x64, 0x58, 0xBA, 0x16, 0x13, 0x01, 0x5C

DeviceAddress  sensor1 = { 0x28, 0xAC, 0x40, 0xF1, 0x16, 0x13, 0x01, 0x54 };
DeviceAddress  sensor2 = { 0x28, 0x64, 0x58, 0xBA, 0x16, 0x13, 0x01, 0x5C };

void ICACHE_RAM_ATTR PulseCount();

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);
  
  // Start the DS18B20 sensors
  sensors.begin();

  // Enable debug to find the DS18B20 addresses in the console
  debug = 0;
  if (debug == true) {
    deviceCount = sensors.getDeviceCount();
    for (int i = 0;  i < deviceCount;  i++)
    {
      Serial.print("Sensor ");
      Serial.print(i+1);
      Serial.print(" : ");
      sensors.getAddress(DS18B20, i);
      printAddress(DS18B20);
    }
  }

  // Change the sensors resolution
  sensors.setResolution(sensor1, 12);
  sensors.setResolution(sensor2, 12);
  sensors.setWaitForConversion(true);

  // Configure input pin to read tach value
  pinMode(tachInputPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tachInputPIN),PulseCount,FALLING);

  // Configure output pin to send fan PWM
  pinMode(pwmOutputPIN, OUTPUT);
  analogWriteRange(100);          // to have a range 1 - 100 for the fan
  analogWriteFreq(25000);         // Noctua Fan PWM --> check Noctua White paper if needed
  analogWrite(pwmOutputPIN, 100); // SET Output to 100% to be sure that fans are running while connecting to WI-FI

  // Set default value for PID
  usedTemp = 0;
  PID_In = 40;
  PID_SP = 40;
  PID_Out = 200;            // 80% speed on startup
  myPID.SetMode(AUTOMATIC);

  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/favicon.ico", "image/png");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(usedTemp).c_str());
  });
  server.on("/fanSP", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(fanSpeedPercent).c_str());
  });
  server.on("/fanPV", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(rpm_percent).c_str());
  });

  // Start server
  server.begin();

  // Start Evaluation timer
  timer.setInterval(10000, evaluate);
}

void loop() {

  // Handle PWM reading --> read RPM data from Fans
  if(millis()-TimeStamp>RPM_UPDATE)
  { 
    detachInterrupt(digitalPinToInterrupt(tachInputPIN));                                      // Detach interrupt while we are calculating
    if(NumberOfPulses<3)NumberOfPulses=0;                                                      // Prevent stopped fans

    rpm = (int)((double)(NumberOfPulses+LastReading)/2*30000.00/(double)(millis()-TimeStamp)); // Average RPM value * 30000 (--> 1000ms * 60sec / 2 (hall sensors twice per revolution)) / by difference time between two readings
    rpm_percent = rpm *100 / RPMMax;                                                           // RPMMax can be different based on the fan used
        
    LastReading=NumberOfPulses;
    NumberOfPulses=0;
    
    attachInterrupt(digitalPinToInterrupt(tachInputPIN),PulseCount,FALLING);
    TimeStamp=millis();
  }

  // Deal with overflow
  if(millis()<TimeStamp)TimeStamp=millis();
  
  timer.run(); 
}

void evaluate() {
  readSensorTemp();
  writeFanSpeed();
  Serial.print("RPM: ");
  Serial.print(rpm_percent);
  Serial.println(" %");
}

void readSensorTemp() {
  sensors.requestTemperatures();              // Send the command to get temperatures
  sensor1Temp = sensors.getTempC(sensor1);    // Read sensor 1
  sensor2Temp = sensors.getTempC(sensor2);    // Read sensor 2
  Serial.print("S1: ");  
  Serial.print(sensor1Temp);
  Serial.print(" °C / S2: ");  
  Serial.print(sensor2Temp);
  Serial.println(" °C");
}

void writeFanSpeed () {

  // Take sensor 1 as default, backup on sensor 2 if 1 is failing
  if(sensor1Temp >= minTemp || sensor1Temp <= maxTemp) {
    usedTemp = sensor1Temp;
  }
  else {
    usedTemp = sensor2Temp;
  }
  
  // PID 
  PID_In = usedTemp;                                // take temperature as Process Value (PV)
  myPID.Compute();                                  // Compute based on parmeter and Setpoint (SP)
  fanSpeedPercent = map(PID_Out, 0, 255, 25, 100);  // Map the Control Value (CV) to get a % 0 - 100
  Serial.print("PID out: ");
  Serial.print(fanSpeedPercent);
  Serial.println(" %");  

  // Safety check here
  if (usedTemp <= minTemp || usedTemp >= maxTemp) { // if all sensors fails --> go to 100 % PWM
    fanSpeedPercent = 100;
  } 
  analogWrite(pwmOutputPIN, fanSpeedPercent);       // Write fan speed
}

void printAddress(DeviceAddress deviceAddress)
{ 
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

void ICACHE_RAM_ATTR PulseCount()
{
  NumberOfPulses++; // Just increment count
  return;
}
