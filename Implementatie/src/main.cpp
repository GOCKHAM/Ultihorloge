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
#include "DFRobot_Heartrate.h"



// ------------------------------------------------------------------------------------------------------------------------------

#define Button1  25
#define Button2  26
#define Button3  14

unsigned long lastButton1Millis = 0;
unsigned long lastButton2Millis = 0;
unsigned long lastButton3Millis = 0;
const unsigned long buttonDebounceInterval = 50; 

int buttonState = 0;
int lastButtonState = 0; 


// ------------------------------------------------------------------------------------------------------------------------------

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

bool SwitchToGPS = false;


unsigned long A_B_StartMillis = 0;
const unsigned long TEMP_HUM_DISPLAY_TIME = 10000; // 10 seconds Temp/Hum


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
#define heartratePin 34
DFRobot_Heartrate heartrate(DIGITAL_MODE); ///< ANALOG_MODE or DIGITAL_MODE
uint8_t BPM = 0; // Variable to store heart rate value
unsigned long lastHeartRateMillis = 0;
const long HEART_RATE_INTERVAL = 100; // Measure heart rate every second when on heart rate page

// ------------------------------------------------------------------------------------------------------------------------------

// GPS PIN and Configuration:
#define RX_PIN 17
#define TX_PIN 16

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

WiFiClientSecure espClient;

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
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to Wi-Fi");
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) { 
    Serial.print(".");
    delay(500); 
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi not connected, but continuing...");
  }




  // Init NTP and set timezone in CET (Central European Time)
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  setTimezone("CET-1CEST,M3.5.0,M10.5.0/3"); 

  // ---------------------------------------------------------------------------
  

  // Showing for the first time
  updateDisplay();
}

// ------------------------------------------------------------------------------------------------------------------------------

// Function to check te WiFi status and reconnect
void checkWiFiReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost, reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }
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
  // Reading Heart Rate sensor value
  uint8_t rateValue;
  heartrate.getValue(heartratePin);   
  rateValue = heartrate.getRate();   // Get heart rate value 
  
  if(rateValue) {
    BPM = rateValue;
    Serial.print("Heart Rate: ");
    Serial.print(BPM);
    Serial.println(" BPM");
  }
  delay(20);
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
        // Display Temperature and Humidity on page 2
      if (!SwitchToGPS) {
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

    } else {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(5, 20);
        u8g2.print("Latitude:");
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(5, 35);
        u8g2.print(latitude, 6);
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(5, 50);
        u8g2.print("Longitude:");
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(5, 60);
        u8g2.print(longitude, 6);
    }
  }
  else if (currentPage == 2) {
    // Display Heart Rate
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setCursor(5, 20);
    u8g2.print("Heart Rate");
    
    u8g2.setFont(u8g2_font_logisoso28_tr);
    u8g2.setCursor(30, 55);
    u8g2.print(BPM);
    
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.setCursor(90, 55);
    u8g2.print("BPM");
  }


  if (currentPage == 3) {
    // Time & Date display
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

    char timeStringBuff[6];  // Format: HH:MM
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo);

    char dateStringBuff[12];  // Format: ex. Mon, Mar 09
    strftime(dateStringBuff, sizeof(dateStringBuff), "%a, %b %d", &timeinfo);

    u8g2.setFont(u8g2_font_logisoso32_tf); 
    u8g2.setCursor(5, 40);
    u8g2.print(timeStringBuff); // Time display

    u8g2.setFont(u8g2_font_helvB08_tf); 
    u8g2.setCursor(10, 60);
    u8g2.print(dateStringBuff); // Date display

    drawIcons();

}
  u8g2.sendBuffer();
}

// ------------------------------------------------------------------------------------------------------------------------------

void automaticMeasurement(){

  // Check if it is time for automatic readings
  if (millis() - lastTempMillis >= TEMP_INTERVAL) {
  lastTempMillis = millis();


  measureTemperatureAndHumidity(); // Measure temperature and humidity
  measureGPS();  // Measure GPS latitude and logitude
  updateDisplay();


  currentPage = 1; // Going to page 2 to show temperature and humidity
  SwitchToGPS = false;

  A_B_StartMillis = millis(); // Start the timer
  updateDisplay();
  }

  // Switching to GPS after 10 seconds
  if (!SwitchToGPS && millis() - A_B_StartMillis >= TEMP_HUM_DISPLAY_TIME) {
    SwitchToGPS = true;
    updateDisplay();
  }


}

// ------------------------------------------------------------------------------------------------------------------------------

// 🟢 Function for Button 1 - Measures Temperature, Humidity & GPS
void handleButton1() {

  // Button 1 for manual measurements and Peltier activation
  unsigned long currentMillis = millis();
  static bool lastButton1State = LOW;
  bool button1State = digitalRead(Button1);

  if (button1State == HIGH && lastButton1State == LOW && (currentMillis - lastButton1Millis >= buttonDebounceInterval)) {
  lastButton1Millis = currentMillis; // Update the timer

  Serial.println("Button 1 pressed: Manual measurement started!");
  measureTemperatureAndHumidity(); // Measure temperature and humidity
  measureGPS();  // Measure GPS latitude and logitude

  currentPage = 1; // Go to page 2
  SwitchToGPS = false;

  A_B_StartMillis = millis(); // Start the timer
  updateDisplay();
  }
  lastButton1State = button1State;

  // Switching to GPS after 10 seconds
  if (!SwitchToGPS && millis() - A_B_StartMillis >= TEMP_HUM_DISPLAY_TIME) {
    SwitchToGPS = true;
    updateDisplay();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------

// 🟢 Function for Button 2 - Measure Heart Rate
void handleButton2() {
  unsigned long currentMillis = millis();
  static bool lastButton2State = LOW;
  bool button2State = digitalRead(Button2);

  if (button2State == HIGH && lastButton2State == LOW && (currentMillis - lastButton2Millis >= buttonDebounceInterval)) {
    lastButton2Millis = currentMillis;
    
    Serial.println("Button 2 pressed: Measuring heart rate!");
    measureHeartRate(); // Start heart rate measurement
  
    currentPage = 2; // Go to Heart Rate page
    updateDisplay();
}
lastButton2State = button2State;

 // When on heart rate page, continuously measure heart rate
 if (currentPage == 2) {
  if (millis() - lastHeartRateMillis >= HEART_RATE_INTERVAL) {
    lastHeartRateMillis = millis();
    measureHeartRate();
    updateDisplay();
  }
}
}

// ------------------------------------------------------------------------------------------------------------------------------

// 🟢 Function for Button 3 - Show Time & Date
void handleButton3() {

  // Press button 3 (Go back to Page 1 - Time & Date).
  unsigned long currentMillis = millis();
  static bool lastButton3State = LOW;
  bool button3State = digitalRead(Button3);
  if (button3State == HIGH && lastButton3State == LOW && (currentMillis - lastButton3Millis >= buttonDebounceInterval)) {
  lastButton3Millis = currentMillis;
  Serial.println("Button 3 pressed: Back to Time page.");
  currentPage = 3; // Return to page 1 (Time & Date)
  updateDisplay();
  }
  lastButton3State = button3State;


  // keep display Time and date every seconds, updated
  if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
  lastDisplayUpdate = currentMillis;
  updateDisplay();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
void loop() {


  // keep display Time and date every seconds, updated
  unsigned long currentMillis = millis();
  if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }


  automaticMeasurement();
  handleButton1();
  handleButton2();
  handleButton3();


  checkWiFiReconnect();

}

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
