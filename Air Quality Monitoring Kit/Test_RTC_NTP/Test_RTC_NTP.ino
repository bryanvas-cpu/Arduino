/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-ds1307-real-time-clock-rtc-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>  // Add this line
#include <time.h>
#include <RTClib.h>

#define WIFI_BUTTON_PIN D5

// Enter your Wi-Fi credentials
const char* ssid = "bryan";
const char* password = "ioioioio";

// NTP server details
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // Offset for GMT in seconds
const int daylightOffset_sec = 0;  // Daylight savings time in seconds

// RTC object (for DS1307 or DS3231)
RTC_DS3231 rtc;  // Change to RTC_DS1307 for DS1307 module

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Global timeinfo struct and last sync timestamp
struct tm timeinfo;
unsigned long lastSyncMillis = 0;  // Last sync time in milliseconds

void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow Serial to initialize

  pinMode(WIFI_BUTTON_PIN, INPUT_PULLUP);

  // Check button at boot
  bool buttonPressed = (digitalRead(WIFI_BUTTON_PIN) == LOW);

  // Call initWiFi with the decision
  initWiFi(buttonPressed);

  // RTC initialization
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Sync RTC at startup
  syncTime();
}
void loop() {
  checkTimeAndSync();  // Check if 1 hour has passed and sync if necessary

  // Get current time from RTC
  DateTime now = rtc.now();

  // Getting each time field in individual variables
  String yearStr = String(now.year(), DEC);
  String monthStr = (now.month() < 10 ? "0" : "") + String(now.month(), DEC);
  String dayStr = (now.day() < 10 ? "0" : "") + String(now.day(), DEC);
  String hourStr = (now.hour() < 10 ? "0" : "") + String(now.hour(), DEC);
  String minuteStr = (now.minute() < 10 ? "0" : "") + String(now.minute(), DEC);
  String secondStr = (now.second() < 10 ? "0" : "") + String(now.second(), DEC);
  String dayOfWeek = daysOfTheWeek[now.dayOfTheWeek()];

  // Complete time string
  String formattedTime = dayOfWeek + ", " + yearStr + "-" + monthStr + "-" + dayStr + " " + hourStr + ":" + minuteStr + ":" + secondStr;

  // Print the complete formatted time
  Serial.println(formattedTime);

  Serial.println();
  delay(10000);
}

void initWiFi(bool startPortal) {
  WiFiManager wm;

  if (startPortal) {
    Serial.println("Starting WiFiManager portal...");
    wm.resetSettings();                
    wm.startConfigPortal("ESP8266-Setup");  // Blocking portal
    Serial.println("Wi-Fi configured!");
  } else {
    if (!wm.autoConnect("ESP8266-Setup")) {
      Serial.println("Failed to connect, restarting...");
      ESP.restart();
    }
    Serial.println("Wi-Fi connected to saved network!");
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}


void syncTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  // Configure time with NTP server
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println("\nESP8266 Time synchronized with NTP server.");

  // Format the current time as a string
  char timeStr[64];
  strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.print("Current time: ");
  Serial.println(timeStr);

  // Sync the RTC with the NTP time
  rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));

  lastSyncMillis = millis();  // Record the last sync time in milliseconds
}

void checkTimeAndSync() {
  // Check if 1 hour has passed since the last sync (1 hour = 3600000 milliseconds)
  if (millis() - lastSyncMillis >= 3600000) {
    Serial.println("Synchronizing time with NTP...");
    syncTime();
  }
}