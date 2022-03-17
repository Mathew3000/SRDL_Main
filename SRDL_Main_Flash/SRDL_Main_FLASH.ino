#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>

#include <ESP32_C3_TimerInterrupt.h>
#include <TinyMPU6050.h>
#include <Adafruit_MPL3115A2.h>
#include <SPI.h>
#include "FS.h"
#include "SPIFFS.h"

#include "Wire.h"

#define MPU_ADDR 0x68
#define MPL_ADDR 0x60
#define TIMER_US 1000

const char* ssid = "srdl_main_wifi";
const char* password = "supersavepw";
const char* host = "srdl_main";
AsyncWebServer server(80);
File fsUploadFile;

MPU6050 mpu (Wire);
Adafruit_MPL3115A2 baro;
ESP32Timer ITimer0(0);

File logFile;
bool fsInit = false;
bool baroInit = false;

int redLed = 7;
int greenLed = 6;


bool IRAM_ATTR TimerHandler0(void * timerNo)
{
	mpu.Execute(); // cycle mpu
    logAllData();
    return true;
}

void setup()
{
    // Setup LEDs
    pinMode(redLed, OUTPUT);
    pinMode(greenLed, OUTPUT);
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW);

    // Alive
    digitalWrite(redLed, HIGH);
    delay(500);
    digitalWrite(redLed, LOW);
    delay(500);
    digitalWrite(redLed, HIGH);
    delay(500);
    digitalWrite(redLed, LOW);
    delay(500);
    digitalWrite(greenLed, HIGH);
    delay(500);
    digitalWrite(greenLed, LOW);
    delay(500);
    digitalWrite(greenLed, HIGH);
    delay(500);
    digitalWrite(greenLed, LOW);

    // Setup SPIFFS
    fsInit = SPIFFS.begin(4);

    // open logfile
    if(fsInit)
    {
        char buf[12];
        for(int i = 0; i < 100; i++)
        {
            snprintf(buf, 12, "/data%i.log", i);
            if(!SPIFFS.exists(buf))
            {
                logFile = SPIFFS.open(buf, FILE_WRITE);
            }
        }
    }
    else    digitalWrite(redLed, HIGH);

    // Setup Sensors
	Wire.begin();
    mpu.Initialize();
    mpu.Calibrate();
    baroInit = baro.begin();
    if(!baroInit)   digitalWrite(redLed, HIGH);
    baro.setSeaPressure(1013.26); // This can be read from file
    // Read pressure calibration from file "calib.txt" in SPIFFS root with pressure as float "1013.26"
    getPressureFromFile(); // If file does not exist ignore

    // setup timer
    bool timerSetup = ITimer0.attachInterruptInterval(TIMER_US, TimerHandler0);
    if(!timerSetup) digitalWrite(redLed, HIGH);
    // Check if error
    if(!fsInit || !baroInit || !timerSetup)
    {
        for(;;)
        {
            delay(100);
        }
    }
    else
    {
        digitalWrite(greenLed, HIGH);
    }

    server.begin();
}

void loop()
{
    delay(5);
}

void logAllData()
{
    int16_t mpuData[] =
    {
        0x5555,
        mpu.GetRawAccX(),
        mpu.GetRawAccY(),
        mpu.GetRawAccZ(),
        mpu.GetRawGyroX(),
        mpu.GetRawGyroY(),
        mpu.GetRawGyroZ()
    };
    float baroData[] =
    {
        baro.getPressure(),
        baro.getAltitude(),
        baro.getTemperature()
    };

    if(logFile)
    {
        logFile.write((uint8_t*)mpuData, sizeof(mpuData));
        logFile.write((uint8_t*)baroData, sizeof(baroData));
    }
}

void getPressureFromFile()
{
    File calibFile = SPIFFS.open("/calib.txt");
    if(calibFile)
    {
        float calibDat = calibFile.readString().toFloat();
        if(calibDat < 1200.0f && calibDat > 800.0f) baro.setSeaPressure(calibDat);
    }
}