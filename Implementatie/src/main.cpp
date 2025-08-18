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
#include <ArduinoJson.h>
#include <HTTPClient.h>



// ------------------------------------------------------------------------------------------------------------------------------

#define Button1  25 // GPIO 25
#define Button2  26 // GPIO 26
#define Button3  14 // GPIO 14

// ------------------------------------------------------------------------------------------------------------------------------

#define peltierPin 32   // GPIO 35

// Peltier status
int peltierPwmValue = 0;   // Default off
bool peltierActive = false; 

// ------------------------------------------------------------------------------------------------------------------------------

unsigned long autoMeasurementDisplayMillis = 0;
bool autoMeasurementActive = false;

unsigned long lastButton1Millis = 0;
unsigned long lastButton2Millis = 0;
unsigned long lastButton3Millis = 0;
const unsigned long buttonDebounceInterval = 50; 
String sendStatus;

int buttonState = 0;
int lastButtonState = 0; 

// ------------------------------------------------------------------------------------------------------------------------------

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Setting SDA and SCL pins
#define SDA_PIN 21    // yellow
#define SCL_PIN 22    //ORange

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
bool SwitchToHome = false;  
unsigned long A_B_StartMillis = 0;
const unsigned long TEMP_HUM_DISPLAY_TIME = 10000; // 10 seconds Temp/Hum
const unsigned long GPS_DISPLAY_TIME = 10000; // 10 seconds GPS


bool displayActive = false;
unsigned long screenTimeoutMillis = 0;   // Time of last page change
const unsigned long SCREEN_TIMEOUT = 60000; // 1 minutes (in milliseconds)

// ------------------------------------------------------------------------------------------------------------------------------
// DHT sensor configuration for temperature and humidity
#define DHTPIN 13 // GPIO 13
#define DHTTYPE DHT22
unsigned long lastTempMillis = 0;
const long TEMP_INTERVAL = 900000; // 15min measurement
float temp;
float hum;                                                                                                                                                                        

DHT dht(DHTPIN, DHTTYPE);

// ------------------------------------------------------------------------------------------------------------------------------
// ===  OpenWeather  ===
const char* openWeatherApiKey = OPENWEATHER_KEY;   // staat in credentialss.h
const char* openWeatherHost   = "api.openweathermap.org";

void getWeather(float lat, float lon);
void displayWeather(float t, const char* desc);
float weatherTemp = NAN; 
char weatherDesc[32]=""; // OpenWeather
String city;


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

// ------------------------------------------------------------------------------------------------------------------------------

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ------------------------------------------------------------------------------------------------------------------------------

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String message = String((char*)payload);

  if (String(topic) == "peltier/control") {
    int pwmValue = message.toInt();
    Serial.print("New PWM value: ");
    Serial.println(pwmValue);

    if (pwmValue >= 0 && pwmValue <= 255) {
      ledcWrite(0, pwmValue);
      Serial.println("Peltier PWM updated.");
    } else {
      Serial.println("Invalid PWM value received.");
    }
  }

     
}

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------

// WiFi Logo Bitmap on display
const uint8_t wifi_icon[] U8X8_PROGMEM = {
	0b00000000, 0b00000000, //                 
	0b11100000, 0b00000111, //      ######     
	0b11111000, 0b00011111, //    ##########   
	0b11111100, 0b00111111, //   ############  
	0b00001110, 0b01110000, //  ###        ### 
	0b11100110, 0b01100111, //  ##  ######  ## 
	0b11110000, 0b00001111, //     ########    
	0b00011000, 0b00011000, //    ##      ##   
	0b11000000, 0b00000011, //       ####      
	0b11100000, 0b00000111, //      ######     
	0b00100000, 0b00000100, //      #    #     
	0b10000000, 0b00000001, //        ##       
	0b10000000, 0b00000001, //        ##       
	0b00000000, 0b00000000, //                 
	0b00000000, 0b00000000, //                 
	0b00000000, 0b00000000, //                 
};

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------

void setup() {
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  pinMode(Button3, INPUT);
  
  // LEDC (PWM) Initialize
  ledcSetup(0, 5000, 8);      // Channel 0, 5kHz frequency, 8-bit resolution (0-255)
  ledcAttachPin(peltierPin, 0);  // Link GPIO4 to channel 0

  // ---------------------------------------------------------------------------

  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, TX_PIN, RX_PIN);  // Begin serial communication with the GPS module
  dht.begin();

  // ---------------------------------------------------------------------------

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(10, 30);
  u8g2.print("Starting ESP32...");
  u8g2.sendBuffer();
  u8g2.setBitmapMode(1); // Ensures icons are displayed correctly
  

  // ---------------------------------------------------------------------------

  // Connecting to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
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

  // ---------------------------------------------------------------------------

  espClient.setInsecure(); // Uses TLS without a certificate
  // MQTT setup
  client.setServer(mqtt_server, 8883);
  client.setCallback(mqttCallback);

  // ---------------------------------------------------------------------------
  
  // Init NTP and set timezone in CET (Central European Time)
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  setTimezone("CET-1CEST,M3.5.0,M10.5.0/3"); 

  // ---------------------------------------------------------------------------

  // Showing for the first time
  updateDisplay();
}

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------

void checkWiFiReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost, reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
void reconnect() {
    static unsigned long previousMillis_reconnect = 0; // Variable to save the previous time
    const unsigned long interval_reconnect = 5000;    // Interval of 5 seconds

    if (!client.connected()) {
        unsigned long currentMillis = millis();

        // Check if the interval has expired
        if (currentMillis - previousMillis_reconnect >= interval_reconnect) {
            previousMillis_reconnect = currentMillis; // Update the previous time

            // Connecting to MQTT
            Serial.println("Attempting MQTT connection...");
        if (client.connect("ESP32TestClient", mqtt_username, mqtt_password)) {
            Serial.println("Connected");

            // Abonneer op meerdere topics
            client.subscribe("sensor/temperature");
            client.subscribe("sensor/humidity");
            client.subscribe("sensor/gps");
            client.subscribe("sensor/weather");
            client.subscribe("peltier/control");
            client.subscribe("button1/control");
            client.subscribe("button2/control");
            


        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");  // If failed, try connecting in 5 seconds again

            }
        }
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

void getWeather(float lat, float lon) {
  Serial.print("Fetching weather for lat: ");
  Serial.print(lat, 6);
  Serial.print(", lon: ");
  Serial.println(lon, 6);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi niet verbonden!");
    return;
  }

  HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/weather?lat=" + String(lat, 6) +
               "&lon=" + String(lon, 6) + "&units=metric&appid=" + openWeatherApiKey;

  Serial.print("Request URL: ");
  Serial.println(url);

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != 200) {
    Serial.print("HTTP error: ");
    Serial.println(httpCode);
    http.end();
    return;
  }

  String payload = http.getString();
  Serial.println("RAW JSON response:");
  Serial.println(payload);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print(F("JSON error: "));
    Serial.println(error.f_str());
    http.end();
    return;
  }

  weatherTemp = doc["main"]["temp"].as<float>();
  city = doc["name"].as<String>(); 

  const char* desc = doc["weather"][0]["description"];
  strncpy(weatherDesc, desc, sizeof(weatherDesc));
  weatherDesc[sizeof(weatherDesc) - 1] = '\0'; // null-terminator

  Serial.print("City: ");
  Serial.println(city);
  Serial.print("Temperature: ");
  Serial.println(weatherTemp);
  Serial.print("Description: ");
  Serial.println(weatherDesc);

  http.end();
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
void drawIcons() {
  int xPos = 110;  // X position for both icons (right side of screen)
  int wifiY = 5;   // Y position for Wi-Fi icon

  // Checking Wi-Fi status, if connected
  if (WiFi.status() == WL_CONNECTED) {
      u8g2.drawXBMP(xPos, wifiY, 16, 16, wifi_icon);  // Show Wi-Fi icon
  } else {
      // No Wi-Fi, draw an "X"
      u8g2.drawLine(xPos, wifiY, xPos + 10, wifiY + 10);
      u8g2.drawLine(xPos + 10, wifiY, xPos, wifiY + 10);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
void checkScreenTimeout() {
  if (currentPage == 3 && millis() - screenTimeoutMillis >= SCREEN_TIMEOUT) {
      displayActive = false;  // Turn off the display
      updateDisplay();        // Update the screen to disable it
  }
}


// ------------------------------------------------------------------------------------------------------------------------------
void updateDisplay() {
  u8g2.clearBuffer();

  if (!displayActive) {
    u8g2.sendBuffer(); // Turn off display
    return;
  }

    	else if (currentPage == 1) {
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

        u8g2.setFont(u8g2_font_helvB10_tr);
        u8g2.setCursor(0, 15); u8g2.print(city);

        u8g2.setFont(u8g2_font_helvB14_tr);
        u8g2.setCursor(0,35); u8g2.print(weatherTemp,1); u8g2.print(" C");

        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.setCursor(0,55); u8g2.print(weatherDesc);
        
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
    if (!isnan(latitude) && !isnan(longitude) && (latitude != 0.0 || longitude != 0.0)) {
      getWeather(latitude, longitude);}
    updateDisplay();

  // Set status as 'Automatic'
  sendStatus = "Automatisch";

  // Publish temperature to Note Red
  String tempPayload = "{\"temperature\":" + String(temp) + ",\"status\":\"Automatic\"}";
  client.publish("sensor/temperature", tempPayload.c_str());

  // Publish humidity to Note Red
  String humPayload = "{\"humidity\":" + String(hum) + ",\"status\":\"Automatic\"}";
  client.publish("sensor/humidity", humPayload.c_str());

  // Publish GPS latitude and longitude to Note Red
  String gpsPayload = "{\"latitude\":" + String(latitude, 6) + ",\"longitude\":" + String(longitude, 6) + ",\"status\":\"Automatic\"}";
  client.publish("sensor/gps", gpsPayload.c_str());

        String wPayload = "{\"outsideT\":" + String(weatherTemp,1) +
                  ",\"desc\":\"" + String(weatherDesc) + "\"" + 
                  ",\"status\":\"Automatic\"}";

  client.publish("sensor/weather", wPayload.c_str());



    displayActive = true;
    currentPage = 1; // Going to page 2 to show temperature and humidity
    SwitchToGPS = false;
    SwitchToHome = false;
    A_B_StartMillis = millis(); // Start the timer
    updateDisplay();
  }

  // Switching to GPS after 10 seconds
  if (!SwitchToGPS && millis() - A_B_StartMillis >= TEMP_HUM_DISPLAY_TIME) {
    SwitchToGPS = true;
    updateDisplay();
  }

  // After another 10 seconds go to home page
  if (SwitchToGPS && !SwitchToHome && millis() - A_B_StartMillis >= (TEMP_HUM_DISPLAY_TIME + GPS_DISPLAY_TIME)) {
    SwitchToHome = true;
    currentPage = 3; // Going back to homepage
    screenTimeoutMillis = millis(); // Reset the timer for 3 minutes
    updateDisplay();
  }
}
// ------------------------------------------------------------------------------------------------------------------------------
// Function for Button 1 - Measures Temperature, Humidity & GPS
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

    String button1Payload = "{\"status\":\"Manual\",\"source\":\"button1\",\"temperature\":" + String(temp) + ",\"humidity\":" + String(hum) + ",\"latitude\":" + String(latitude, 6) + ",\"longitude\":" + String(longitude, 6) + "}";
    client.publish("button1/control", button1Payload.c_str());

    displayActive = true;
    currentPage = 1; // Go to page 2
    SwitchToGPS = false;
    SwitchToHome = false;
    A_B_StartMillis = millis(); // Start the timer
    updateDisplay();
  }
  lastButton1State = button1State;

  // Switching to GPS after 10 seconds
  if (!SwitchToGPS && millis() - A_B_StartMillis >= TEMP_HUM_DISPLAY_TIME) {
    SwitchToGPS = true;
    updateDisplay();
  }

  // After another 10 seconds go to home page
  if (SwitchToGPS && !SwitchToHome && millis() - A_B_StartMillis >= (TEMP_HUM_DISPLAY_TIME + GPS_DISPLAY_TIME)) {
    SwitchToHome = true;
    currentPage = 3; // Going back to homepage
    screenTimeoutMillis = millis(); // Reset the timer for 3 minutes
    updateDisplay();
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
// Function for Button 2 - Measure outside temperature
void handleButton2() {
  unsigned long currentMillis = millis();
  static bool lastButton2State = LOW;
  bool button2State = digitalRead(Button2);
  
  if (button2State == HIGH && lastButton2State == LOW && (currentMillis - lastButton2Millis >= buttonDebounceInterval)) {
    lastButton2Millis = currentMillis;
    
    Serial.println("Button 2 pressed: Measuring Oudside temperature!");

    measureGPS();
    if (!isnan(latitude) && !isnan(longitude) && (latitude != 0.0 || longitude != 0.0)) {
      getWeather(latitude, longitude);

      String button2Payload = "{\"outsideT\":" + String(weatherTemp,1) +
                  ",\"desc\":\"" + String(weatherDesc) + "\"" + 
                  ",\"status\":\"" + sendStatus + "\",\"source\":\"button2\"}";

    client.publish("button2/control", button2Payload.c_str());
    
    } else {
    Serial.println("Geen GPS‑fix: kan geen weer ophalen.");
    }
    sendStatus = "Manueel";

    displayActive = true;
    currentPage = 2; // Go to Outside Temperature page
    updateDisplay();
  }
  lastButton2State = button2State;
  
}

// -----------------------------------------------------------------------------------------------------------------------------
// Function for Button 3 - Show Time & Date
void handleButton3() {

  // Press button 3 (Go back to Page 1 - Time & Date).
  unsigned long currentMillis = millis();
  static bool lastButton3State = LOW;
  bool button3State = digitalRead(Button3);
  if (button3State == HIGH && lastButton3State == LOW && (currentMillis - lastButton3Millis >= buttonDebounceInterval)) {
    lastButton3Millis = currentMillis;
    Serial.println("Button 3 pressed: Back to Time page.");
    displayActive = true;
    currentPage = 3; // Return to page 1 (Time & Date)
    screenTimeoutMillis = millis(); // Reset the timer for 3 minutes
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
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

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
  checkScreenTimeout();
  checkWiFiReconnect();

}

// ------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------