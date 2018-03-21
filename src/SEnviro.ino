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
 SYSTEM_MODE(AUTOMATIC);

 static const String TIME_FORMAT = "%Y-%m-%e %k:%M:%S";

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

 unsigned int timer;
 Weather sensor;
 FuelGauge fuel;

 //TODO
 //{0x28, 0x6F, 0xD1, 0x5E, 0x06, 0x00, 0x00, 0x76};//Waterproof temp sensor address
 /***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/
 DeviceAddress inSoilThermometer = {0x40, 0x60};

 //Global Variables
 //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

 long lastSecond; //The millis counter to see when a second rolls by
 byte seconds; //When it hits 60, increase the current minute
 byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
 byte minutes; //Keeps track of where we are in various arrays of data
 byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data


 String SEnviroID = System.deviceID();
 String dateutc = "";

 //We need to keep track of the following variables:
 //Wind speed/dir each update (no storage)
 //Wind gust/dir over the day (no storage)
 //Wind speed/dir, avg over 2 minutes (store 1 per second)
 //Wind gust/dir over last 10 minutes (store 1 per minute)
 //Rain over the past hour (store 1 per minute)
 //Total rain over date (store one per day)

 byte windspdavg[120]; //120 bytes to keep track of 2 minute average
 int winddiravg[120]; //120 ints to keep track of 2 minute average
 float windgust_10m[10]; //10 floats to keep track of 10 minute max
 int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
 volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

 //These are all the weather values that wunderground expects:
 int winddir = 0; // [0-360 instantaneous wind direction]
 float windspeedmph = 0; // [mph instantaneous wind speed]
 float windgustmph = 0; // [mph current wind gust, using software specific time period]
 int windgustdir = 0; // [0-360 using software specific time period]
 float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
 int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
 float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
 int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
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

 int count = 599;

 // volatiles are subject to modification by IRQs
 volatile long lastWindIRQ = 0;
 volatile byte windClicks = 0;
 volatile unsigned long raintime, rainlast, raininterval, rain;

 MQTT client(MQTT_SERVER, 1883, 30, callback, 512);

 void callback(char* topic, byte* payload, unsigned int length) {
   char p[length + 1];
   memcpy(p, payload, length);
   p[length] = NULL;
 }

 void update18B20Temp(DeviceAddress deviceAddress, double &tempC);//predeclare to compile

 //Interrupt routines (these are called by the hardware interrupts, not by the main code)
 //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

 void rainIRQ()
 // Count rain gauge bucket tips as they occur
 // Activated by the magnet and reed switch in the rain gauge, attached to input D2
 {
   raintime = millis(); // grab current time
   raininterval = raintime - rainlast; // calculate interval between this and last event

   if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
   {
     dailyrainin += 0.011; //Each dump is 0.011" of water
     rainHour[minutes] += 0.011; //Increase this minute's amount of rain
     rainlast = raintime; // set up for next event
   }
 }

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

     Particle.keepAlive(120);
     mqtt_connect();
     //Particle.variable("data", data);
     Particle.syncTime();

     // DS18B20 initialization
     sensors.begin();
     sensors.setResolution(inSoilThermometer, TEMPERATURE_PRECISION);

     pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
     pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

     pinMode(SOIL_MOIST_POWER, OUTPUT);//power control for soil moisture
     digitalWrite(SOIL_MOIST_POWER, LOW);//Leave off by defualt

     Serial.begin(9600);   // open serial over USB

     // Make sure your Serial Terminal app is closed before powering your device
     // Now open your Serial Terminal, and hit any key to continue!
     //Serial.println("Press any key to begin");
     //This line pauses the Serial port until a key is pressed
     //while(!Serial.available()) Spark.process();

     //Initialize the I2C sensors and ping them
     sensor.begin();

     /*You can only receive acurate barrometric readings or acurate altitiude
     readings at a given time, not both at the same time. The following two lines
     tell the sensor what mode to use. You could easily write a function that
     takes a reading in one made and then switches to the other mode to grab that
     reading, resulting in data that contains both acurate altitude and barrometric
     readings. For this example, we will only be using the barometer mode. Be sure
     to only uncomment one line at a time. */
     sensor.setModeBarometer();//Set to Barometer Mode
     //baro.setModeAltimeter();//Set to altimeter Mode

     //These are additional MPL3115A2 functions the MUST be called for the sensor to work.
     sensor.setOversampleRate(7); // Set Oversample rate
     //Call with a rate from 0 to 7. See page 33 for table of ratios.
     //Sets the over sample rate. Datasheet calls for 128 but you can set it
     //from 1 to 128 samples. The higher the oversample rate the greater
     //the time between data samples.

     sensor.enableEventFlags(); //Necessary register calls to enble temp, baro ansd alt

     seconds = 0;
     lastSecond = millis();

     // attach external interrupt pins to IRQ functions
     attachInterrupt(RAIN, rainIRQ, FALLING);
     attachInterrupt(WSPEED, wspeedIRQ, FALLING);

     // turn on interrupts
     interrupts();
 }


 void mqtt_connect() {

   if (!Cellular.ready()) {
       Serial.println("Cellular.on()");
       timer = millis();
       Cellular.on();
       Serial.printf("On complete in %d\n", millis()-timer);
       delay(1000);

       Serial.println("Cellular.connect()");
       timer = millis();
       Cellular.connect();
       Serial.printf("Connect complete in %d\n", millis()-timer);
       delay(1000);
   }

   client.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWORD);

 }


 //---------------------------------------------------------------
 void loop()
 {
   //Keep track of which minute it is
   if(millis() - lastSecond >= 1000)
   {

     lastSecond += 1000;
     //Take a speed and direction reading every second for 2 minute average
     if(++seconds_2m > 119) seconds_2m = 0;

     //Calc the wind speed and direction every second for 120 second to get 2 minute average
     float currentSpeed = get_wind_speed();
     //float currentSpeed = random(5); //For testing
     int currentDirection = get_wind_direction();
     windspdavg[seconds_2m] = (int)currentSpeed;
     winddiravg[seconds_2m] = currentDirection;
     //if(seconds_2m % 10 == 0) displayArrays(); //For testing

     //Check to see if this is a gust for the minute
     if(currentSpeed > windgust_10m[minutes_10m])
     {
       windgust_10m[minutes_10m] = currentSpeed;
       windgustdirection_10m[minutes_10m] = currentDirection;
     }

     //Check to see if this is a gust for the day
     if(currentSpeed > windgustmph)
     {
       windgustmph = currentSpeed;
       windgustdir = currentDirection;
     }

     if(++seconds > 59)
     {
       seconds = 0;

       if(++minutes > 59) minutes = 0;
       if(++minutes_10m > 9) minutes_10m = 0;

       rainHour[minutes] = 0; //Zero out this minute's rainfall amount
       windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
     }

     //Get readings from all sensors
     //getWeather();
     //Rather than use a delay, keeping track of a counter allows the photon to
     // still take readings and do work in between printing out data.
     count++;
     //alter this number to change the amount of time between each reading
     if(count == 600)
      {
        //Get readings from all sensors
         getWeather();

         int iAttempts = 0;

         while(!client.isConnected() && iAttempts < 5)
         {
             mqtt_connect();
             delay(50);
             iAttempts = iAttempts + 1;
         }

         publish();
         count = 0;

      }
   }
 }

 //--------------------------------------------------------------

 void publish() {

   publishObs("AirTemperature",tempc);
   publishObs("Humidity",humidity);
   publishObs("Precipitation",rainin);
   publishObs("WindDirection",winddir);
   publishObs("WindSpeed",windspeedmph);
   publishObs("AtmosphericPressure",pascals/100);
   publishObs("SoilTemperature",soiltempf);
   publishObs("SoilHumidity",soilMoisture);
   publishObs("Battery",fuel.getSoC());

   Serial.println("Cellular off");
   timer = millis();
   Cellular.off();
   Serial.printf("Off complete in %d\n", millis()-timer);
   delay(1000);

 }

 //--------------------------------------------------------------

 void publishObs( String sTopic, float fObs) {

   String sObs = "";
   sObs += "{";
   sObs += "\"time\":\"" + dateutc + "\",";
   sObs += "\"value\":\"" + String(fObs) + "\"";
   sObs += "}";

   String sAux = "/current/" + SEnviroID + "/" + sTopic;

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
     update18B20Temp(inSoilThermometer, InTempC);
     //Every so often there is an error that throws a -127.00, this compensates
     if(InTempC < -100)
       soiltempf = soiltempf;//push last value so data isn't out of scope
     else
       soiltempf = (InTempC * 9)/5 + 32;//else grab the newest, good data
 }
 //---------------------------------------------------------------
 void getSoilMositure()
 {
     /*We found through testing that leaving the soil moisture sensor powered
     all the time lead to corrosion of the probes. Thus, this port breaks out
     Digital Pin D5 as the power pin for the sensor, allowing the Photon to
     power the sensor, take a reading, and then disable power on the sensor,
     giving the sensor a longer lifespan.*/
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
 //Read the wind direction sensor, return heading in degrees
 int get_wind_direction()
 {
   unsigned int adc;

   adc = analogRead(WDIR); // get the current reading from the sensor

   // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
   // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
   // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

   //Wind Vains may vary in the values they return. To get exact wind direction,
   //it is recomended that you AnalogRead the Wind Vain to make sure the values
   //your wind vain output fall within the values listed below.
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
 //Returns the instataneous wind speed
 float get_wind_speed()
 {
   float deltaTime = millis() - lastWindCheck; //750ms

   deltaTime /= 1000.0; //Covert to seconds

   float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

   windClicks = 0; //Reset and start watching for new wind
   lastWindCheck = millis();

   //windSpeed *= 1.492; //4 * 1.492 = 5.968MPH
   windSpeed *= 2.4; //kmh

   return(windSpeed);
 }


 //---------------------------------------------------------------
 void getWeather()
 {
     dateutc = Time.format(Time.now(), TIME_FORMAT);
     // Measure Relative Humidity from the HTU21D or Si7021
     humidity = sensor.getRH();
     // Measure Temperature from the HTU21D or Si7021
     tempf = sensor.getTempF();
     tempc = sensor.getTemp();

     //Measure Pressure from the MPL3115A2
     pascals = sensor.readPressure();

     //If in altitude mode, you can get a reading in feet  with this line:

     getSoilTemp();//Read the DS18B20 waterproof temp sensor
     getSoilMositure();//Read the soil moisture sensor

     //Calc winddir
     winddir = get_wind_direction();

     //Calc windspeed
     windspeedmph = get_wind_speed();

     //Calc windgustmph
     //Calc windgustdir
     //Report the largest windgust today
     windgustmph = 0;
     windgustdir = 0;

     //Calc windspdmph_avg2m
     float temp = 0;
     for(int i = 0 ; i < 120 ; i++)
       temp += windspdavg[i];
     temp /= 120.0;
     windspdmph_avg2m = temp;

     //Calc winddir_avg2m
     temp = 0; //Can't use winddir_avg2m because it's an int
     for(int i = 0 ; i < 120 ; i++)
       temp += winddiravg[i];
     temp /= 120;
     winddir_avg2m = temp;

     //Calc windgustmph_10m
     //Calc windgustdir_10m
     //Find the largest windgust in the last 10 minutes
     windgustmph_10m = 0;
     windgustdir_10m = 0;
     //Step through the 10 minutes
     for(int i = 0; i < 10 ; i++)
     {
       if(windgust_10m[i] > windgustmph_10m)
       {
         windgustmph_10m = windgust_10m[i];
         windgustdir_10m = windgustdirection_10m[i];
       }
     }

     //Total rainfall for the day is calculated within the interrupt
     //Calculate amount of rainfall for the last 60 minutes
     rainin = 0;
     for(int i = 0 ; i < 60 ; i++)
       rainin += rainHour[i];
 }
