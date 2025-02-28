#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include "time.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h> 
#include <credentialss.h>


#define Button1  25
#define Button2  26
#define Button3  14

// ------------------------------------------------------------------------------------------------------------------------------

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Setting SDA and SCL pins
#define SDA_PIN 21   
#define SCL_PIN 22

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Variables for timing
const unsigned long BUTTON_DEBOUNCE_INTERVAL = 200; // 200 ms for debounce
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // 1 seconds
                                                                                                                                             
// Variables for page view and debounce
int currentPage = 1;  // Start on Page 1 (Time & Date)
unsigned long lastButtonPressTime = 0;
const unsigned long debounceTime = 300;  // Debounce time in ms

void updateDisplay(); // Function prototype

// ------------------------------------------------------------------------------------------------------------------------------
// DHT sensor configuration for temperature and humidity
#define DHTPIN 13
#define DHTTYPE DHT22
unsigned long lastTempMillis = 0;
const long TEMP_INTERVAL = 900000; // 15min measurement
float temp;
float hum;                                                                                                                                                                          

DHT dht(DHTPIN, DHTTYPE);

// ------------------------------------------------------------------------------------------------------------------------------
// Heart Rate PIN
int pulsePin = 35;

// ------------------------------------------------------------------------------------------------------------------------------
// GPS PIN and Configuration:
#define TX_PIN 16
#define RX_PIN 17

unsigned long gpsStartTime = 0; // Variable to keep track of GPS timer start time
const unsigned long GPS_DELAY = 50; // Delay for GPS timer in milliseconds

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;  // TinyGPS++ object to process GPS data.
unsigned long last = 0UL; // Variable for tracking the latest time of the GPS update

float latitude = 0.0;
float longitude = 0.0;  // Variables for latitude and longitude

// ------------------------------------------------------------------------------------------------------------------------------
// NTP = Network Time Protocol. This is for date and time
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 0; //19800;
const int   DAYLIGHT_OFFSET_SEC = 0;

// Function to set the timezone
void setTimezone(String timezone){
  Serial.printf("Setting Timezone to %s\n", timezone.c_str());
  setenv("TZ", timezone.c_str(), 1);  //  Setting the time zone
  tzset();  // Configuration timezone
}

String getCurrentDateAndTime(); 

// ------------------------------------------------------------------------------------------------------------------------------

void setup() {
  
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  pinMode(Button3, INPUT);



  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // Begin serial communication with the GPS module
  dht.begin();
  
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(10, 30);
  u8g2.print("Starting ESP32...");
  u8g2.sendBuffer();
  

  // ---------------------------------------------------------------------------

  // Connecting to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.println("Connecting to Wi-Fi");
  }
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP adres: ");
  Serial.println(WiFi.localIP());




  // Init NTP and set timezone in CET (Central European Time)
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  setTimezone("CET-1CEST,M3.5.0,M10.5.0/3"); 

  // Print the current date and time for once
  Serial.print("Current Time and Date: ");
  Serial.println(getCurrentDateAndTime());

  // ---------------------------------------------------------------------------
  
  // Timer configuration for ESP32
  timer = timerBegin(0, 80, true);          // Timer 0, prescaler 80 (1 tick = 1us)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2000, true);       // 2000us = 2ms
  timerAlarmEnable(timer);                  // Start the timer

  // Showing for the first time
  updateDisplay();
}


// ------------------------------------------------------------------------------------------------------------------------------
// Function to get Time and Date informatioon
String getCurrentDateAndTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }
  
  char timeStringBuff[50]; 
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  
  String asString(timeStringBuff);
  asString.replace(" ", "-");

  return asString;

}

// ------------------------------------------------------------------------------------------------------------------------------
// Function to measure Temperature and Humidity
void measureTemperatureAndHumidity() {
    temp = dht.readTemperature();
    hum = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
        Serial.println("ERROR: Couldn't get data from DHT sensor");
        return;
    }
    

    // Serial Print debug information Temperature and Humidity
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.println(" %");
}


// ------------------------------------------------------------------------------------------------------------------------------

// Function to measure Heart Rate
void measureHeartRate(){

  // COMING SOON


}
// ------------------------------------------------------------------------------------------------------------------------------

void measureGPS(){
  // Starting GPS-timer
  gpsStartTime = millis();

  while (gpsSerial.available() > 0 || millis() - gpsStartTime < GPS_DELAY) {
      if (gps.encode(gpsSerial.read())) {
          if (gps.location.isUpdated() || gps.date.isUpdated() || gps.time.isUpdated()) {
              // Getting information per detection
              latitude = gps.location.lat(); // Getting the Latitude information
              longitude = gps.location.lng(); // Getting the Longitude information

              // Serial Print debug information GPS Latitude and Longitude
              Serial.print(F("Location: "));
              Serial.print(F("Latitude: "));
              Serial.print(gps.location.lat(), 6);
              Serial.print(F(", Longitude: "));
              Serial.println(gps.location.lng(), 6);
              break;
            }
        }
    }

}

// ------------------------------------------------------------------------------------------------------------------------------


void updateDisplay() {
  u8g2.clearBuffer();
    if (currentPage == 1) {
        // Display Time & Date on main page 1
        u8g2.setFont(u8g2_font_helvB10_tf);
        u8g2.setCursor(20, 20);
        u8g2.print("Time & Date");
        
        u8g2.setFont(u8g2_font_helvB08_tf);
        u8g2.setCursor(10, 40);
        u8g2.print(getCurrentDateAndTime());
    } 

    	else if (currentPage == 2) {
      // Display Temperature and Humidity on page 2
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(5, 15);
        u8g2.print("Temperature: ");
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(5, 30);
        u8g2.print(temp); u8g2.print(" C");
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(5, 45);
        u8g2.print("Humidity: ");
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(5, 60);
        u8g2.print(hum); u8g2.print(" %");
        
  }
      else if (currentPage == 3) {
      // Display Heart Rate on page 3
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(5, 15);
        u8g2.print("Heart Rate: "); 
        u8g2.setCursor(5, 45);
        u8g2.print(BPM); 
        u8g2.print(" BPM");
  
    }
    u8g2.sendBuffer();
}
void loop() {

void updateDisplay();

}

