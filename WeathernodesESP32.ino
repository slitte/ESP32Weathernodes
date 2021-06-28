





/////////////////////// ArduinoJson Setup

#define ARDUINOJSON_ENABLE_STD_STREAM 0
#define ARDUINOJSON_ENABLE_STD_STRING 1
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1

/////////////////////// Libraries
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "settings.h"
#include <Adafruit_Sensor.h>


/////////////////////// Node Config Hostname
const char* espName = "ESPxTest";
const char* mqtt_clientId = "ESPxTest";

// Available Topics for mqtt

const char* topic_data = "/data/nodes";


/////////////////////// Sensor Setup

#define DHTPIN 19     // Digital pin connected to the DHT sensor 

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);


Adafruit_BME280 bme; // I2C


/////////////////////// WiFi and Definitions
WiFiClient wifiClient;
PubSubClient client(wifiClient);

#define SERIAL_BAUD 115200


/////////////////////// Deep Sleep Definitions
#define FORCE_DEEPSLEEP
#define uS_TO_mS_FACTOR 1000  /* Conversion factor for micro seconds to miliseconds */
#define mS_TO_S_FACTOR 1000  /* Conversion factor for milliseconds to seconds */
#define M_TO_S_FACTOR 60  /* Conversion factor for seconds to minutes */
#define TIME_TO_SLEEP  1        /* Time ESP32 will go to sleep (in minutes) */

// Variables for Sensor Data

float tempdht(NAN);
float humidht(NAN);
float presbme(NAN);
float tempbme(NAN);
float humibme(NAN);


/**
 * Le Setup
 */
void setup() {
    Serial.begin(SERIAL_BAUD);
    print_wakeup_reason(); // Reason for ESP to Wake UP
    setupdeepsleep(); // Setup for Deepsleep
    Wire.begin();
    pinMode(DHTPIN, INPUT);
    dht.begin();
    if (!bme.begin(0x76)) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        while (1);
    }

    splashScreen();
    delay(2000); //reprogramming

    startWIFI();

}

void setupdeepsleep() {

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR * mS_TO_S_FACTOR * M_TO_S_FACTOR); //enable Timer Wakeup with conversion from minutes to u seconds
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
        " Minutes");
}


/**
 *  get Data from Sensors
 */
void getSensorData() {


    Serial.println("---");
    Serial.println("");
    Serial.println("Measuring Temperature DHT");
    int tryCnttd = 0;
    while (isnan(tempdht)) {
        delay(2500);
        tempdht = dht.readTemperature(); // Gets the values of the temperature
        Serial.print(".");
        tryCnttd++;

        if (tryCnttd > 5) {
            Serial.println("");
            Serial.println("Could not read Temperature.");
            goToBed(5);
        }

    }
    Serial.println("");


    Serial.println("Measuring Humidity DHT");

    int tryCnthd = 0;
    while (isnan(humidht)) {
        delay(2500);
        humidht = dht.readHumidity(); // Gets the values of the humidity 
        Serial.print(".");
        tryCnthd++;

        if (tryCnthd > 5) {
            Serial.println("");
            Serial.println("Could not read Humidity.");
            goToBed(5);
        }

    }
    Serial.println("");
    Serial.println("Measuring Temperature BME");

    int tryCntbt = 0;
    while (isnan(tempbme)) {
        delay(2500);
        tempbme = bme.readTemperature(); // Gets the values of the Temperature
        Serial.print(".");
        tryCntbt++;

        if (tryCntbt > 5) {
            Serial.println("");
            Serial.println("Could not read Temperature.");
            goToBed(5);
        }

    }


    Serial.println("");
    Serial.println("Measuring Pressure BME");

    int tryCntbp = 0;
    while (isnan(presbme)) {
        delay(2500);
        presbme = bme.readPressure(); // Gets the values of the Pressure
        Serial.print(".");
        tryCntbp++;

        if (tryCntbp > 5) {
            Serial.println("");
            Serial.println("Could not read Pressure.");
            goToBed(5);
        }

    }

    presbme = presbme / 100; // Pressure in hPa
    Serial.println("");
    Serial.println("Measuring Humidity BME");

    int tryCntbh = 0;
    while (isnan(humibme)) {
        delay(2500);
        humibme = bme.readHumidity(); // Gets the values of the Humiidty
        Serial.print(".");
        tryCntbh++;

        if (tryCntbh > 5) {
            Serial.println("");
            Serial.println("Could not read Humidity");
            goToBed(5);
        }

    }

    Serial.println("");


    Serial.println("---");
    Serial.print("Temperature DHT = ");
    Serial.print(tempdht);
    Serial.println(" °C");
    Serial.print("Humidity DHT = ");
    Serial.print(humidht);
    Serial.println(" %");
    Serial.print("Temperature BME = ");
    Serial.print(tempbme);
    Serial.println(" °C");
    Serial.print("Pressure BME = ");
    Serial.print(presbme);
    Serial.println(" hPa");
    Serial.print("Humidity BME = ");
    Serial.print(humibme);
    Serial.println(" %");
    Serial.println("---");



}
/**
 *  send data to mqtt via ArduinoJson
 */
void sendtomqtt() {
    Serial.println("Sending Sensordata to MQTT");


    tempdht = tempdht * 100;
    tempdht = roundf(tempdht);
    tempdht = tempdht / 100;

    humidht = humidht * 100;
    humidht = roundf(humidht);
    humidht = humidht / 100;

    tempbme = tempbme * 100;
    tempbme = roundf(tempbme);
    tempbme = tempbme / 100;


    presbme = presbme * 100;
    presbme = roundf(presbme);
    presbme = presbme / 100;


    humibme = humibme * 100;
    humibme = roundf(humibme);
    humibme = humibme / 100;


    DynamicJsonDocument doc(200);

    doc["ESPName"] = espName;
    doc["tempdht"] = tempdht;
    doc["humidht"] = humidht;
    doc["tempbme"] = tempbme;
    doc["presbme"] = presbme;
    doc["humibme"] = humibme;


    serializeJson(doc, Serial);
    char buffer[200];
    size_t n = serializeJson(doc, buffer);
    client.publish(topic_data, buffer, n);
    Serial.println("");
    Serial.println("Data sent to Server");
}




/**
 * Establish WiFi-Connection
 *
 * If connection times out (threshold 50 sec)
 * device will sleep for 5 minutes and will restart afterwards.
 */
void startWIFI() {
    Serial.println("---");
    WiFi.mode(WIFI_STA);
    Serial.println("(Re)Connecting to Wifi-Network with following credentials:");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Device-Name: ");
    Serial.println(espName);

    WiFi.hostname(espName);
    WiFi.begin(ssid, password);

    int tryCnt = 0;

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        tryCnt++;

        if (tryCnt > 100) {
            Serial.println("");
            Serial.println("Could not connect to WiFi. Sending device to bed.");
            goToBed(5);
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    delay(300);
}


/**
 * Establish MQTT-Connection
 *
 * If connection fails, device will sleep for 5 minutes and will restart afterwards.
 */
void runMQTT() {
    Serial.println("---");
    Serial.println("Starting MQTT-Client");
    Serial.print("Host: ");
    Serial.println(mqtt_server);
    Serial.print("ClientId: ");
    Serial.println(mqtt_clientId);

    client.setServer(mqtt_server, mqtt_port);

    while (!client.connected()) {
        Serial.print("Attempting connection... ");
        // Attempt to connect
        if (client.connect(mqtt_clientId, mqtt_user, mqtt_password)) {
            Serial.println("Success.");
            client.loop();
        }
        else {
            Serial.println("Failed.");
            Serial.println("Could not connect to MQTT-Server. Sending device to bed.");
            goToBed(5);
        }
    }
}


/**
 * Sending device into deep sleep
 */
void goToBed(int minutes) {
#ifdef FORCE_DEEPSLEEP
    Serial.println("going to sleep for " + String(TIME_TO_SLEEP) + " minutes");
    //Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
    delay(100);
#endif
}
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
    }
}

/**
 * Dump some information on startup.
 */
void splashScreen() {
    for (int i = 0; i <= 5; i++) Serial.println();
    Serial.println("#######################################");
    Serial.print("# ");
    Serial.println("# -----------");
    Serial.println("# -----------");
    Serial.print("# DeviceName: ");
    Serial.println(espName);
    Serial.print("# Configured Endpoint: ");
    Serial.println(mqtt_server);
    Serial.println("#######################################");
    for (int i = 0; i < 2; i++) Serial.println();
}

/**
 * Looping Louie
 */
void loop() {
    runMQTT();
    getSensorData();
    sendtomqtt();

    delay(500); //wait
    goToBed(TIME_TO_SLEEP); //sending into deep sleep
}