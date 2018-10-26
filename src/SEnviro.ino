/*
* Project SEnviro for agriculture
* Description: A skecth to develop a SEnviro node
* Author: Sergi Trilles
* Date: 21/03/2018
*/

#include "spark-dallas-temperature.h"
#include "OneWire.h"

#include <MQTT.h>
#include <SparkFun_Photon_Weather_Shield_Library.h>
#include <OneWire.h>
#include <spark-dallas-temperature.h>

#include "cellular_hal.h"
#include "Particle.h"

STARTUP(cellular_credentials_set("orangeworld", "orange", "orange", NULL));
//STARTUP(cellular_credentials_set("moreinternet", "", "", NULL));
SYSTEM_MODE(AUTOMATIC);

static const String TIME_FORMAT = "%Y-%m-%d %k:%M:%S";

#define MQTT_USER "senvmq"
#define MQTT_PASSWORD "senviro.2018"
#define MQTT_SERVER "senviro.init.uji.es"
#define MQTT_CLIENTID "senviro"

#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 11
#define SOIL_MOIST A1
#define SOIL_MOIST_POWER D5
#define WDIR  A0
#define RAIN  D2
#define WSPEED  D3

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

enum State {
  RECOLLECT_STATE,
  PUBLISH_STATE,
  SLEEP_STATE,
  SLEEP_WAIT_STATE,
  ENERGY_STATE,
  UPDATE_STATE
};

enum Energy {
  NORMAL_MODE,
  RECOVERY_MODE,
  CRITICAL_MODE
};

struct storeObser{
  String sTime;
  float aData[9];
};

State state = RECOLLECT_STATE;
Energy energy = NORMAL_MODE;

int SLEEP_PERIOD = 530;
static unsigned long lastSleep =    0 ;

unsigned int timer;
Weather sensor;
FuelGauge fuel;

long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

String SEnviroID = System.deviceID();
String dateutc = "";

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]

float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
long lastWindCheck = 0;
volatile float dailyrainin = 0; // [rain inches so far today in local time]

float humidity = 0;
float tempf = 0;
float tempc = 0;
double InTempC = 0;//original temperature in C from DS18B20
float soiltempf = 0;//converted temperature in F from DS18B20
float pascals = 0;
int soilMoisture = 0;

bool bUpdate = FALSE;
int count = 599;

int ObsStored = 0;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile unsigned long raintime, rainlast, raininterval, rain;

void callback(char* topic, byte* payload, unsigned int length);
MQTT client(MQTT_SERVER, 1883, 30, callback, 512);
//MQTT client(MQTT_SERVER, 1883, callback);

void update18B20Temp(DeviceAddress deviceAddress, double &tempC);//predeclare to compile

//---------------------------------------------------------------
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event
  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {

    dailyrainin += 0.2794; //Each dump is 0.011" of water
    //rainHour[minutes] += 0.2794; //Increase this minute's amount of rain
    rainlast = raintime; // set up for next event

  }
}

//---------------------------------------------------------------
void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

//---------------------------------------------------------------
void setup()
{
  Serial.println("Setup");
  //Different connections
  Particle.keepAlive(120);
  mqtt_connect();
  Particle.syncTime();

  Serial.printf(SEnviroID);

  // DS18B20 initialization
  sensors.begin();

  if (SEnviroID.equals("4e0022000251353337353037")) {
    DeviceAddress dA1 = {0x40, 0x60};
    sensors.setResolution(dA1, TEMPERATURE_PRECISION);
  }
  else if (SEnviroID.equals("270043001951343334363036")) {
    DeviceAddress dA1 = {0x28, 0x1A, 0x6, 0xAA, 0x8, 0x0, 0x0, 0x81};
    sensors.setResolution(dA1, TEMPERATURE_PRECISION);
  }
  else if (SEnviroID.equals("380033001951343334363036")) {
    DeviceAddress dA1 = {0x28, 0x82, 0x19, 0xAA, 0x8, 0x0, 0x0, 0x82};
    sensors.setResolution(dA1, TEMPERATURE_PRECISION);
  }
  else if (SEnviroID.equals("46005a000351353337353037")) {
    DeviceAddress dA1 = {0x28, 0x70, 0x60, 0xA8, 0x8, 0x0, 0x0, 0x7};
    sensors.setResolution(dA1, TEMPERATURE_PRECISION);
  }
  else if (SEnviroID.equals("4e0031000251353337353037")) {
    DeviceAddress dA1 = {0x28, 0x54, 0x67, 0xA9, 0x8, 0x0, 0x0, 0xB3};
    sensors.setResolution(dA1, TEMPERATURE_PRECISION);
  }
  else if (SEnviroID.equals("200034001951343334363036")) {
    DeviceAddress dA1 = {0x60, 0x40};
    sensors.setResolution(dA1, TEMPERATURE_PRECISION);
  }

  //sensors.setResolution(inSoilThermometer, TEMPERATURE_PRECISION);

  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

  pinMode(SOIL_MOIST_POWER, OUTPUT); //power control for soil moisture
  digitalWrite(SOIL_MOIST_POWER, LOW); //Leave off by defualt

  Serial.begin(9600); // open serial over USB

  sensor.begin(); //Initialize the I2C sensors and ping them
  sensor.setModeBarometer(); //Set to Barometer Mode
  sensor.setOversampleRate(7); // Set Oversample rate
  sensor.enableEventFlags(); //Necessary register calls to enble temp, baro ansd alt

  seconds = 0;
  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(RAIN, rainIRQ, FALLING);
  attachInterrupt(WSPEED, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();
}

//---------------------------------------------------------------
void mqtt_connect() {

  if (!Cellular.ready()) {

    Serial.println("Cellular.connect()");
    timer = millis();
    Cellular.connect();
    Serial.printf("Connect complete in %d\n", millis()-timer);
    delay(1000);
  }

  client.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWORD);

  updateMode();
}

//---------------------------------------------------------------
void loop()
{

  bool mqConnected;
  mqConnected = client.loop();

  switch(state) {

    case ENERGY_STATE:
    {
      Serial.println("ENERGY");

      if (fuel.getSoC() > 15)
      {
        energy = NORMAL_MODE;
      }
      else if (fuel.getSoC() > 5)
      {
        energy = RECOVERY_MODE;
      }
      else
      {
        energy = CRITICAL_MODE;
      }

      state = RECOLLECT_STATE;
      break;
    }

    case UPDATE_STATE:
    {
      Serial.println("UPDATE");

      bUpdate = FALSE;
      delay(600000);

      state = ENERGY_STATE;
      break;
    }

    case RECOLLECT_STATE:
    {
      Serial.println("RECOLLECT");

      if (energy != CRITICAL_MODE){
        getWeather();
      }

      state = PUBLISH_STATE;
      break;
    }

    case PUBLISH_STATE:
    {
      Serial.println("PUBLISH");

      if (energy == NORMAL_MODE){
        int iAttempts = 0;
        while(!mqConnected && iAttempts < 5)
        {
          mqtt_connect();
          delay(50);
          iAttempts = iAttempts + 1;
          mqConnected = client.loop();
        }
        delay(10);
        if (mqConnected)
        {

          publish();

          while(ObsStored > 0)
          {
            publishLastStored();
          }
        }
      }
      else if (energy == RECOVERY_MODE)
      {
        storeEEPROM();
      }

      if (bUpdate)
      {
        state = UPDATE_STATE;
      }
      else
      {
        state = SLEEP_STATE;
      }

      break;
    }

    case SLEEP_STATE:
    {
      Serial.println("SLEEP");

      if (energy != CRITICAL_MODE){
        int sleepLeft = SLEEP_PERIOD;
        int startSleep = Time.now();
        while (sleepLeft > 0) {
          System.sleep(RAIN, FALLING, sleepLeft, SLEEP_NETWORK_STANDBY);
          sleepLeft = SLEEP_PERIOD - (Time.now() - startSleep);
          if (sleepLeft > 1) {
            unsigned long timeout = millis();
            while ( millis() - timeout < 10) delay(1);
          }
        }
        lastSleep = millis();
      }
      else
      {
        System.sleep(SLEEP_MODE_DEEP, SLEEP_PERIOD*3, SLEEP_NETWORK_STANDBY);
      }

      state = ENERGY_STATE;
      break;
    }
  }
}

//---------------------------------------------------------------
void publishLastStored(){

  storeObser sObser;
  EEPROM.get(ObsStored * sizeof(storeObser), sObser);

  publishObs("lost","AirTemperature",sObser.aData[0],sObser.sTime);
  publishObs("lost","Humidity",sObser.aData[1],sObser.sTime);
  publishObs("lost","Precipitation",sObser.aData[2],sObser.sTime);
  publishObs("lost","WindDirection",sObser.aData[3],sObser.sTime);
  publishObs("lost","WindSpeed",sObser.aData[4],sObser.sTime);
  publishObs("lost","AtmosphericPressure",sObser.aData[5],sObser.sTime);
  publishObs("lost","SoilTemperature",sObser.aData[6],sObser.sTime);
  publishObs("lost","SoilHumidity",sObser.aData[7],sObser.sTime);
  publishObs("lost","Battery",sObser.aData[8],sObser.sTime);

  ObsStored -= 1;
}

//---------------------------------------------------------------
void updateMode()
{
  //Serial.println("UpdateSubscribe");
  String sAux = "/update/" + SEnviroID ;
  //Serial.println(sAux);
  client.subscribe(sAux , MQTT::QOS1);
}

//---------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.println("callback");
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  if (message.equals("TRUE"))
  {
    Serial.println("UPDATE MODE");
    bUpdate = TRUE;
  }
}

//--------------------------------------------------------------
void storeEEPROM()
{
  storeObser observa = {dateutc, tempc, humidity, rainin, winddir, windspeedmph, pascals/100, soiltempf, soilMoisture, fuel.getSoC()};
  EEPROM.put(ObsStored * sizeof(storeObser), observa);
  ObsStored += 1;
}

//--------------------------------------------------------------
void publish()
{
  publishObs("current","AirTemperature",tempc,dateutc);
  publishObs("current","Humidity",humidity,dateutc);
  publishObs("current","Precipitation",rainin,dateutc);
  publishObs("current","WindDirection",winddir,dateutc);
  publishObs("current","WindSpeed",windspeedmph,dateutc);
  publishObs("current","AtmosphericPressure",pascals/100,dateutc);
  publishObs("current","SoilTemperature",soiltempf,dateutc);
  publishObs("current","SoilHumidity",soilMoisture,dateutc);
  publishObs("current","Battery",fuel.getSoC(),dateutc);
}

//--------------------------------------------------------------
void publishObs(String sMoment, String sTopic, float fObs, String sTime)
{
  String sObs = "";
  sObs += "{";
  sObs += "\"time\":\"" + sTime + "\",";
  sObs += "\"value\":\"" + String(fObs) + "\"";
  sObs += "}";

  String sAux = "/"+sMoment+"/" + SEnviroID + "/" + sTopic;

  Serial.println("Topic: ");
  Serial.println(sAux);
  Serial.println("Data: ");
  Serial.println(String(sObs));

  client.publish( sAux , String(sObs));
}

//---------------------------------------------------------------
void getSoilTemp()
{
  //get temp from DS18B20
  sensors.requestTemperatures();

  if (SEnviroID.equals("4e0022000251353337353037")) {
    DeviceAddress dA1 = {0x40, 0x60};
    update18B20Temp(dA1, InTempC);
  }
  else if (SEnviroID.equals("270043001951343334363036")) {
    DeviceAddress dA1 = {0x28, 0x1A, 0x6, 0xAA, 0x8, 0x0, 0x0, 0x81};
    update18B20Temp(dA1, InTempC);
  }
  else if (SEnviroID.equals("380033001951343334363036")) {
    DeviceAddress dA1 = {0x28, 0x82, 0x19, 0xAA, 0x8, 0x0, 0x0, 0x82};
    update18B20Temp(dA1, InTempC);
  }
  else if (SEnviroID.equals("46005a000351353337353037")) {
    DeviceAddress dA1 = {0x28, 0x70, 0x60, 0xA8, 0x8, 0x0, 0x0, 0x7};
    update18B20Temp(dA1, InTempC);
  }
  else if (SEnviroID.equals("4e0031000251353337353037")) {
    DeviceAddress dA1 = {0x28, 0x54, 0x67, 0xA9, 0x8, 0x0, 0x0, 0xB3};
    update18B20Temp(dA1, InTempC);
  }
  else if (SEnviroID.equals("200034001951343334363036")) {
    DeviceAddress dA1 = {0x60, 0x40};
    update18B20Temp(dA1, InTempC);
  }

  if(InTempC < -100)
    soiltempf = soiltempf;
  else
    soiltempf = InTempC;
}
//---------------------------------------------------------------
void getSoilMositure()
{
  digitalWrite(SOIL_MOIST_POWER, HIGH);
  delay(200);
  soilMoisture = analogRead(SOIL_MOIST);
  delay(100);
  digitalWrite(SOIL_MOIST_POWER, LOW);
}

//---------------------------------------------------------------
void update18B20Temp(DeviceAddress deviceAddress, double &tempC)
{
  tempC = sensors.getTempC(deviceAddress);
}

//---------------------------------------------------------------
int get_wind_direction()
{
  unsigned int adc;
  adc = analogRead(WDIR);

  if(adc > 2270 && adc < 2290) return (0);//North
  if(adc > 3220 && adc < 3299) return (1);//NE
  if(adc > 3890 && adc < 3999) return (2);//East
  if(adc > 3780 && adc < 3850) return (3);//SE
  if(adc > 3570 && adc < 3650) return (4);//South
  if(adc > 2790 && adc < 2850) return (5);//SW
  if(adc > 1580 && adc < 1610) return (6);//West
  if(adc > 1930 && adc < 1950) return (7);//NW

  return (-1); // error, disconnected?
}

//---------------------------------------------------------------
float get_wind_speed()
{
  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();
  delay(30000);
  float deltaTime = millis() - lastWindCheck; //750ms
  deltaTime /= 1000.0; //Covert to seconds
  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4
  //windSpeed *= 1.492; //4 * 1.492 = 5.968MPH
  windSpeed *= 2.4; //kmh

  return(windSpeed);
}

//---------------------------------------------------------------
void getWeather()
{
  dateutc = Time.format(Time.now(), TIME_FORMAT);
  humidity = sensor.getRH();
  tempf = sensor.getTempF();
  tempc = sensor.getTemp();

  pascals = sensor.readPressure();

  getSoilTemp();//Read the DS18B20 waterproof temp sensor
  getSoilMositure();//Read the soil moisture sensor

  winddir = get_wind_direction();
  windspeedmph = get_wind_speed();

  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 60 minutes
  rainin = dailyrainin;
  dailyrainin = 0;
}
