#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LSM303.h>
#include <ADXL345.h>
#include <SparkFun_MMA8452Q.h>
#include <ESP8266HTTPClient.h>
//#include <ArduinoJson.h>
#include <WifiLocation.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include "vector.h"
#include <iterator>
#include <algorithm>    // std::sort
#include <Filters.h>    // test signal frequency (Hz) pmcFilter
/*
ID 1 = "Pato"
ID 2 = "Isi"
ID 3 = "Chrysalis"
ID 4 = "Zeke I+D"
ID 5 = "Zeke Stgo"
*/
bool myfunction (long i,long j) { return (i<j); }

#define N_OFFSET 32
#define SAMPLES 200           // samples number
#define N_WIN 2               // Windwos

#define TRNM 1UL             // Parameter in between windwos time in hour
#define T_OFFSET 10          // offset recalculation in minutes
#define T_STATUS    2        // time of status hrs
#define TST 10000UL          // Start parameter windwos in secs
#define TRNS 40              // Sample windows time in millisecs
#define TASD 1000            // time after send data in msecs
#define TWC 1                // time Wifi check
#define Factor_IQR  5.0      // Interquarile reference factor
#define Factor_CAV  1.5      // Cumulative acelerator vector reference factor
#define Factor_ZC   1        // Zero Crossing rate reference factor
#define Factor_RSL  3.5
#define StartWindow 20
#define _SSID "zekefi-interno"  // network name
#define _NETPASS "JtXDF5jK79es" // network password
#define OFFSET true              // set median dc offset remove
#define REF_VAR  false          // indicate if reference are variables or fixed values

#define LSMACC   1          // LSM303 acceleromter ID
#define MMAACC   2          // MMA8452Q acceleromter ID
#define ADXLACC  3          // ADXL345 acceleromter ID
byte acc_id;                // accelerometer identification

int EventPin = D7;     // wifi status LED
int StatusPin = D6;      // accelerometer data readings LED
int ButtonPin = D3;   // On/Off wifi settings

/* Sampling freq */
#define FrecRPS  100                   // Tasa de muestreo definida por el usuario (100 a 200)
unsigned int INTERVALO = (1000 / FrecRPS); // s

/* time variables */
time_t tiempo;        // ?
time_t prevtime;      // time for check every one second
time_t t_offsetlap;   // time between offset recalculation
time_t t_mevent;      // when an event start && movement timer checker
time_t t_dpevent;      // when an event start && displacement timer checker
time_t t_temp;         // temporal for DP event
time_t wifiLap;       // counter time for wifi check
time_t ntpmicro;      // ntp microseconds
time_t eventAccion;   // time after event is on
time_t amaxm;         // millsecond of the max acceleration
time_t t_online;      //
time_t t_eventled;    // timer for led event display



/* accelerometer parameters and variables  */
char ID[15];                  // device number identification
char UBIDIR[200];             //UBIDOTS direccion


// status variables
bool ISCALC;                  // parameter calculation enable/disable
bool EVENT;                   // event detection on/off
bool REFERENCE;               // parameter calculation event
bool CSERVER;                 // check if central server is responding on/off
bool DATASEND;                // enable/disable send event to server
bool MCHECK;                  // movement event checker
bool DPCHECK;                  // displacement event checker
bool ISAFULL;                  // Check if array samples are full true = full
bool REPORT;                    // report status device
bool FILTER;                    // ENABLE DISABLE FILTER
bool WIFIRST;                  // wifi reset


// variabales counters per samples
int psample;                   // sample counter for parameters calculation
int s_cserver;                 // samples counter when server is not responding
int sample;                    // sample index
int c_mevent;                  // movement event counter
int c_pdevent;                 // prolongate seism and displacement for 0: counter / 1: time seims event counter
int eventsample;               // samples after first event detected

// timer counters
int tc_mevent;                 // cunter for an event start && movement checker
int tc_pdevent;                // for event start && displacement checker
int tc_temp1;                   // counter temporal for PD event
int tc_temp2;                   // counter temporal for PD event
int tc_wifirst;                // counter for  wifi rst button

//acelerometer objets
LSM303 compass;               // acelerometer objet
MMA8452Q accel;
ADXL345 accelerometer;
Vector sca;

// acceleromter variables

long offset_x, offset_y, offset_z;         // offset data for normalization
short old_x, old_y, old_z;                 // old acceleration data for normalization
long AccNetnow[SAMPLES];                   // net acceleration
short acc_x[SAMPLES];                      // xyz axis acceleration array
short acc_y[SAMPLES];                      // xyz axis acceleration array
short acc_z[SAMPLES];                      // xyz axis acceleration array
short acc_xMax, acc_yMax, acc_zMax;        // xyzaxis max acceleration
short mag_t;                               // sample number of aceleration magnitude

etl::vector<short,SAMPLES> X;
etl::vector<short,SAMPLES> Y;
etl::vector<short,SAMPLES> Z;
etl::vector<long,SAMPLES> XYZ;

float testFrequency = 2;                     // test signal frequency (Hz)
FilterOnePole filterOneHighpassX( HIGHPASS, testFrequency );  // create a one pole (RC) highpass filter pmcFilter
FilterOnePole filterOneHighpassY( HIGHPASS, testFrequency );  // create a one pole (RC) highpass filter pmcFilter
FilterOnePole filterOneHighpassZ( HIGHPASS, testFrequency );  // create a one pole (RC) highpass filter pmcFilter

// Parameter variables
long  ACNmax, ACNmin;               // Max net acceleration
short RSL, RSLref, RSLmin, RSLmax;    // Mr. P parameter value
long IQR, IQRref, IQRmin, IQRmax;    // Interquiarlie Values
short ZC, ZCref, ZCmax, ZCmin;     //  Zero crossing
long CAV, CAVref, CAVmax, CAVmin;    // Cumulative Acceleration Value

short trigger;                 // 0:notrigger / 1:RSL / 2:ZIC

/* AP settings */
char ssid[] = _SSID;           //  your network SSID (name)
char pass[] = _NETPASS;       // your network password
WiFiManager wifiManager;


/* NTP parameters and NTP Servers: */
//static const char ntpServerName[] = "us.pool.ntp.org";
char ntpServerName[40];
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
const unsigned long HTTP_TIMEOUT = 10000;  // max respone time from server
const size_t MAX_CONTENT_SIZE = 512;       // max size of the HTTP response



/* geolocation variables */
//const char* host = "freegeoip.net";        // host where the ip is get it
const char* host = "ip-api.com";
const char* googleApiKey = "AIzaSyDrRpRVLNpwu9GsMMB4XyAbE8JrNLG9d98";
WifiLocation location(googleApiKey);
location_t loc;
char latitude[32];                         // latitude value from host
char longitude[32];          // longitude value from host


bool toggle = false;
bool eventtoggle = false;
bool statustoggle = false;


void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(EventPin, OUTPUT);
  pinMode(StatusPin, OUTPUT);
  pinMode(ButtonPin, INPUT_PULLUP);


  Serial.begin(500000);
  // Wire.begin(int sda, int scl)
  // Wire.begin(4, 5);       // join i2c bus (address optional for master)
  Wire.begin();       // join i2c bus (address optional for master)
  while (!Serial) ; // Needed for Leonardo only
  delay(250);
  tc_wifirst = 0;

  WIFIRST = true;
  digitalWrite(LED_BUILTIN,LOW );
  delay(2000);
  digitalWrite(LED_BUILTIN,HIGH );

  //reset saved settings
  //wifiManager.resetSettings();
  while (digitalRead(ButtonPin) == LOW && WIFIRST == true){
    analogWrite(StatusPin, 256-tc_wifirst*51);
    analogWrite(EventPin, 256- tc_wifirst*51);
    analogWrite(LED_BUILTIN, tc_wifirst*51);
    if(tc_wifirst >= 5){
      wifiManager.resetSettings();
      WIFIRST = false;
    }
    tc_wifirst++;
    delay(1000);
  }

  //first parameter is name of access point, second is the password
  wifiManager.autoConnect();

  digitalWrite(StatusPin, LOW);
  digitalWrite(EventPin, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Prosismic");
  Serial.print("Connected to ");
  // Serial.println(ssid);
  // WiFi.begin(ssid, pass);

  /* wait until wifi is connected */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }



  Serial.println("");
  Serial.print("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");

  /* NTP time config*/
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  time_t t_ntp = 0;

  setSyncProvider(getNtpTime);
  setSyncInterval(SECS_PER_HOUR * 1);
  Serial.print("Current time: ");
  digitalClockDisplay();




  /* geolocation config */
  loc = location.getGeoFromWiFi();
  dtostrf(loc.lat, 7 , 5, latitude); // Leave room for too large numbers!
  dtostrf(loc.lon, 7 , 5, longitude); // Leave room for too large numbers!
  Serial.println("Location request data");
  Serial.println(location.getSurroundingWiFiJson());
  Serial.println(latitude);
  Serial.println(longitude);
  Serial.println("Accuracy: " + String(loc.accuracy));




  /* acclereometer setup , see library for details */
  acc_id = 0;
  do {
    if (!compass.init()) {
      Serial.println("Could not find a valid LSM303DLH sensor, check wiring or I2C program pins!");
      delay(500);
    } else {
      acc_id = LSMACC;
    }
    if (!accel.init(SCALE_2G, ODR_400)) {
      Serial.println("Could not find a valid MMA8452Q sensor, check wiring or I2C program pins!");
      delay(500);
    }  else {
      acc_id = MMAACC;
    }
    if (!accelerometer.begin()) {
      Serial.println("Could not find a valid ADXL345 sensor, wiring or I2C program pins!");
      delay(500);
    } else {
      acc_id = ADXLACC;
    }
  } while (acc_id == 0);

  char _buff[4];
  switch (acc_id) {
    case LSMACC:
    {
      compass.enableDefault();
      snprintf(ID, sizeof(ID),"IMU%d", ESP.getChipId() );
      snprintf(UBIDIR, sizeof(UBIDIR)
      , "http://things.ubidots.com/api/v1.6/devices/PS%dIMU/RSL/values?token=wsFGicZyoCaq4uVDQcDm2btS3YpahA"
      , ESP.getChipId());
      Serial.println("LSM303DLH setup complete!");
    }
    break;
    case MMAACC:
    {
      snprintf(ID, sizeof(ID), "MMA%d", ESP.getChipId() );
      snprintf(UBIDIR, sizeof(UBIDIR)
      , "http://things.ubidots.com/api/v1.6/devices/PS%dMMA/RSL/values?token=wsFGicZyoCaq4uVDQcDm2btS3YpahA"
      , ESP.getChipId());
      Serial.println("MMA8452Q setup complete!");
    }
    break;
    case ADXLACC:
    {
      accelerometer.useInterrupt(ADXL345_INT1);
      accelerometer.setRange(ADXL345_RANGE_2G);
      accelerometer.setDataRate(ADXL345_DATARATE_400HZ);
      snprintf(ID, sizeof(ID), "ADXL%d", ESP.getChipId() );
      snprintf(UBIDIR, sizeof(UBIDIR)
      , "http://things.ubidots.com/api/v1.6/devices/PS%dADXL/RSL/values?token=wsFGicZyoCaq4uVDQcDm2btS3YpahA"
      , ESP.getChipId());
      Serial.println("ADXL345 setup complete !");
    }
    break;
    default:
    {
      Serial.println("MATRIX ERROR");
    }
    break;
  }




  /* setup init values */
  for (int k = 0; k < SAMPLES ; k++) {
    AccNetnow[k] = 0;
    acc_x[k] = 0;
    acc_y[k] = 0;
    acc_z[k] = 0;
  }

  /* Starting values */
  old_x = 0;
  old_y = 0;
  old_z = 0;

  /* parameter variables */
  RSLref = 32767;
  IQRref = 50000.00;
  ZCref = 32767;
  CAVref = 32767;
  IQRmax = 500;
  IQRmin = 500;
  CAVmax = 1000;
  CAVmin = 1000;
  ACNmax = 0;
  ACNmin = 0;
  ZCmax = 22;
  ZCmin = 22;

  /*counters*/
  sample = 0;
  psample = 0;
  eventsample = 0 ; // samples after first event detected
  s_cserver = 0;
  c_mevent = 0;

  /* time counter variables */
  tc_mevent = 0;
  tc_pdevent = 0;
  tc_wifirst = 0;

  /* time variables */
  t_offsetlap = now();
  t_online = now();
  t_mevent = 0;
  t_dpevent = 0;
  t_eventled =  now()-8*SECS_PER_MIN;

  wifiLap = now();
  ntpmicro = millis();

  /* status variables */
  DATASEND = false;
  ISCALC = false;
  EVENT = false;
  CSERVER = true;
  REFERENCE = true;
  MCHECK = false;
  ISAFULL = false;
  FILTER = true;


  /*Device ID */

  /* after 3 seconds calculate the offset */
  delay(1000);
  if(FILTER == false)
  offset(N_OFFSET);  // 50 samples for the offset media
  Serial.println();



  digitalWrite(StatusPin, HIGH);
  digitalWrite(EventPin, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  digitalWrite(StatusPin, LOW);
  digitalWrite(EventPin, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
}



void loop()
{

  int td_sampling;
  int td_calc;
  char line[200];
  /* restore sampling window */
  if (sample == SAMPLES)
  sample = 0;
  /* read values at sampling rate */

  if (millis() - tiempo >= INTERVALO) {

    tiempo = millis();
    Serial.print(now());
    digitalClockDisplay();

    td_sampling = accSampling3();

    snprintf(line, sizeof(line), "%dsam %d[ms] %d %d %d %d", sample,td_sampling, X[0],Y[0], Z[0], XYZ[0] );
    Serial.print(line);
    Serial.print(" ");

    td_calc = calcParam2(sample);

    snprintf(line, sizeof(line)," %d[ms] %d %d %d %d",td_calc, acc_xMax, acc_yMax, acc_zMax, ACNmax);
    Serial.print(line);
    Serial.print(" ");

    snprintf(line, sizeof(line),"%d %d %d %d", ZC, IQR, CAV, RSL);
    Serial.print(line);
    Serial.print(" ");

    snprintf(line, sizeof(line),"%d %d %d %d", ZCref, IQRref, CAVref, RSLref);
    Serial.print(line);
    Serial.print(" ");



    trigger = 0;

    /* check for sensor movement */
    if (MCHECK == true) {
      if (tc_mevent <= 1500) { // check 1500 count
        snprintf(line, sizeof(line),"MEvent %d/5 in %d count", c_mevent, (1500-tc_mevent));
        Serial.print(line);
        Serial.print(" ");
        tc_mevent++;

        if (c_mevent >= 5) { // are 5 event
          DATASEND = false;
          EVENT = false;
          MCHECK = false;
          DPCHECK = false;
          eventsample = 0; // ready for next eventsamples
          Serial.print("Pause event sending until offset");
          Serial.print(" ");
        }
      }

      else {
        tc_mevent = 0;
        c_mevent = 0;
        if (EVENT == false)
        MCHECK = false;
      }
    }



    /* check for sensor small displacement and prolongate seism*/
    if (DPCHECK == true) {
      char dpline[100];


      if (tc_pdevent <= 6000) { // check 6000 count windwow
        snprintf(dpline,sizeof(dpline),"PDEvent %d/3 in %4d count - ec1 %d ec2 %3d", c_pdevent,6000-tc_pdevent,tc_temp1,tc_temp2);
        Serial.print(dpline);
        Serial.print(" ");

        if(c_pdevent == 1){
          tc_temp1++;
        }

        if(c_pdevent == 2){

          if(tc_temp1 >= 300){
            tc_temp2++;
          }
          else {
            tc_temp1 = 0;
            c_pdevent = 1;
          }

        }

        if(c_pdevent == 3){

          if(tc_temp2 >= 300){
            t_dpevent = 0;
            c_pdevent = 0;
            t_temp = 0;
            DATASEND = false;
            EVENT = false;
            DPCHECK = false;
            MCHECK = false;
            Serial.print(" Pause event sending until offset");
            Serial.print(" ");
          }
          else {
            tc_temp1 = 0;
            tc_temp2 = 0;
            c_pdevent = 1;
          }
        }

      }


      else {
        t_dpevent = 0;
        c_pdevent = 0;
        t_temp = 0;
        DPCHECK = false;
      }

      tc_pdevent++;
    }


    /* count samples for restore event detector */
    if (EVENT) { // pause after eventis detected

      /* check if event count complete */
      if (eventsample >= SAMPLES / 2) {
        eventsample = 0;
        DATASEND = true;
        EVENT = false;


      }
      else {
        Serial.print("Pause event for ");
        Serial.print(SAMPLES / 2 - eventsample);
        Serial.print(" eventsamples");
        Serial.print(" ");
        eventsample++;

      }

    }


    if (!CSERVER) { // wait for server resend
      Serial.print("Pause event for ");
      Serial.print(SAMPLES / 2 - s_cserver);
      Serial.print(" s_cserver");
      Serial.print(" ");
      s_cserver++;

      /* check server count complete */
      if (s_cserver >= SAMPLES / 2) {
        s_cserver = 0;
        DATASEND = true;
        CSERVER = true;

      }
    }


    /* reclaculate offset and reference */
    if (!DATASEND && REFERENCE && (X.size()>= SAMPLES)) {
      if(psample == 0 && FILTER == false){
        snprintf(line, sizeof(line), "Offset(x, y, z) = (%d,%d,%d)", offset_x, offset_y, offset_z);
        Serial.print(line);
        Serial.print(" ");
      }
      if (CalcRef(psample, ISCALC, SAMPLES * 5) == 1) {
        Serial.print("References Calculated");
        Serial.print(" ");
        DATASEND = true;
        REFERENCE = false;
        psample = 0;
        sample = 0;

        t_dpevent = 0;
        c_pdevent = 0;
        tc_temp1 = 0;
        tc_temp2 = 0;
        DPCHECK = false;
        tc_mevent = 0;
        c_mevent = 0;
        MCHECK = false;

      }
    }

    if(REPORT == true){
      Serial.print("Sensor Report Status");
      REPORT == false;
    }


    snprintf(line, sizeof(line), "");
    Serial.println();
    sample++; // window sample counter
    psample++;// parameter calculator counter


  } // end sampling counter




  /* sync the milliseconds with ntp time */
  if (timeStatus() != timeNotSet) {
    if (now() != prevtime) { //update the display only if time has changed
      prevtime = now();
      ntpmicro = millis();

      if(DATASEND == true){
        statustoggle = false; // check
        eventtoggle = false;  // check
        toggle = !toggle;
      }
      else {

        statustoggle =  eventtoggle;
        eventtoggle = !statustoggle;
        toggle = true;
      }
    }
  }

  /* led display */
  if (EVENT == true){
    toggle = true;
    statustoggle = false; // check
    eventtoggle = !eventtoggle;  // check
  }

  else { // EVENT == false

    /* led event display*/
    if(((now()-t_eventled) < 7*SECS_PER_MIN) && DATASEND == true){
      /* update every 0.5 secs */
      if((millis()-ntpmicro) >= 500){
        statustoggle = false;
      }
      else {
        statustoggle = true;
      }
    }



  }






  digitalWrite(LED_BUILTIN,toggle);
  digitalWrite(EventPin,eventtoggle);
  digitalWrite(StatusPin,statustoggle);





  /* Sesim event detect and check every 1 TASD secs after first detection*/

  trigger = 0;
  // if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
  if ( (IQR > IQRref && CAV > CAVref)) {
    //if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref)) {
    //  if ((IQR > IQRref && ZC < ZCref && CAV > CAVref)) trigger = 2;
    if ((IQR > IQRref && CAV > CAVref)) trigger = 2;
    // if (RSL >= RSLref)  trigger = 1;

    t_eventled = now();

    if (DATASEND) {


      DATASEND = false;

      c_mevent++;
      if (MCHECK == false) {
        tc_mevent = 0;
        MCHECK = true;
      }

      c_pdevent++;
      if (DPCHECK == false) {
        tc_pdevent = 0;
        tc_temp1 = 0;
        tc_temp2 = 0;
        DPCHECK = true;
      }


      eventtoggle = true;

      if (!sendPost(now())) { // if the server response
        EVENT = true;
        t_offsetlap = now();
        eventtoggle = false;
      }

      else { //if the server does not respones
        CSERVER = false;
        MCHECK = false;
        DPCHECK = false;
        Serial.print("Server not found. Cannot send event");
      }

      Serial.println();
    }

  }


  /* offset recalcs every TRNM minutes*/
  if ((now() - t_offsetlap) >= (SECS_PER_MIN * T_OFFSET)) {
    t_offsetlap = now();
    if (!EVENT) {
      if(FILTER == false)
      offset(N_OFFSET);
      REFERENCE = true;
      DATASEND = false;
      MCHECK = false;
      DPCHECK = false;
      psample = 0;
    }
  }

  /* Status send*/
  if ((now() - t_online) >= (SECS_PER_MIN * T_STATUS)) {
    t_online = now();
    sendStatus(1);
    REPORT = true;
  }

  /* Check wifi connection */
  if ((now() - wifiLap) == (SECS_PER_HOUR * TWC)) {
    wifiLap = now();
    /* wait until wifi is connected*/
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");

    }
  }

}

/* Read values from accelerometer and store it in the array */
int accSampling2() {
  int _x , _y, _z;
  int start = millis();

  switch (acc_id) {
    case LSMACC:
    {
      compass.read();
      _x = 0.5 * (compass.a.x / 16 - offset_x) + 0.5 * old_x;
      _y = 0.5 * (compass.a.y / 16 - offset_y) + 0.5 * old_y;
      _z = 0.5 * (compass.a.z / 16 - offset_z) + 0.5 * old_z;
    }
    break;
    case MMAACC:
    {
      accel.readRaw(); // mma
      _x = 0.5 * (accel.x - offset_x) + 0.5 * old_x;
      _y = 0.5 * (accel.y - offset_y) + 0.5 * old_y;
      _z = 0.5 * (accel.z - offset_z) + 0.5 * old_z;

    }
    break;
    case ADXLACC:
    {
      sca = accelerometer.readmg();
      _x = 0.5 * (sca.XAxis - offset_x) + 0.5 * old_x;
      _y = 0.5 * (sca.YAxis - offset_y) + 0.5 * old_y;
      _z = 0.5 * (sca.ZAxis - offset_z) + 0.5 * old_z;

    }
    break;
    default:
    {
      Serial.println("MATRIX ERROR");
    }
    break;
  }

  if(X.size() == SAMPLES){
    X.pop_back();
    Y.pop_back();
    Z.pop_back();
    XYZ.pop_back();
  }

  X.insert(X.begin(),_x);
  Y.insert(Y.begin(),_y);
  Z.insert(Z.begin(),_z);

  XYZ.insert(XYZ.begin(),sqrt(_x*_x + _y*_y + _z*_z) * 100.0);

  old_x = _x;
  old_y = _y;
  old_z = _z;

  return (millis()-start);

}

/* Read values from accelerometer and store it in the array */
int accSampling3() {
  int _x , _y, _z;
  int start;

  switch (acc_id) {
    case LSMACC:
    {
      compass.read();
      filterOneHighpassX.input( compass.a.x  / 16 ); // update the one pole lowpass filter, and statistics pmcFilter
      filterOneHighpassY.input( compass.a.y  / 16 ); // update the one pole lowpass filter, and statistics pmcFilter
      filterOneHighpassZ.input( compass.a.z  / 16 ); // update the one pole lowpass filter, and statistics pmcFilter
    }
    break;
    case MMAACC:
    {
      accel.readRaw(); // mma
      filterOneHighpassX.input( accel.x ); // update the one pole lowpass filter, and statistics pmcFilter
      filterOneHighpassY.input( accel.y ); // update the one pole lowpass filter, and statistics pmcFilter
      filterOneHighpassZ.input( accel.z ); // update the one pole lowpass filter, and statistics pmcFilter

    }
    break;
    case ADXLACC:
    {
      sca = accelerometer.readmg();
      filterOneHighpassX.input( sca.XAxis ); // update the one pole lowpass filter, and statistics pmcFilter
      filterOneHighpassY.input( sca.YAxis ); // update the one pole lowpass filter, and statistics pmcFilter
      filterOneHighpassZ.input( sca.ZAxis ); // update the one pole lowpass filter, and statistics pmcFilter

    }
    break;
    default:
    {
      Serial.println("MATRIX ERROR");
    }
    break;
  }

  _x = 0.5 * (filterOneHighpassX.output()) + 0.5 * old_x;
  _y = 0.5 * (filterOneHighpassY.output()) + 0.5 * old_y;
  _z = 0.5 * (filterOneHighpassZ.output()) + 0.5 * old_z;

  if(X.size() == SAMPLES){
    X.pop_back();
    Y.pop_back();
    Z.pop_back();
    XYZ.pop_back();
  }

  X.insert(X.begin(),_x);
  Y.insert(Y.begin(),_y);
  Z.insert(Z.begin(),_z);

  XYZ.insert(XYZ.begin(),sqrt(_x*_x + _y*_y + _z*_z) * 100.0);

  old_x = _x;
  old_y = _y;
  old_z = _z;



  return (millis()-start);;

}

/* Calcs CAV , RSL , IQR and ZC parameters and returns time
of the operation in milliseconds */
time_t calcParam2(int _sample) {
  long sumshort = 0;
  long sumlong = 0;
  float cavlong = 0;
  float cavshort = 0;
  float _smpl = SAMPLES/8;
  bool xcurrent;
  bool xprevious;
  bool ycurrent;
  bool yprevious;
  bool zcurrent;
  bool zprevious;
  short xzcr = 0;
  short yzcr = 0;
  short zzcr = 0;
  time_t startcalc;
  short acc_xabs;
  short acc_yabs;
  short acc_zabs;
  short kkkk;
  short kkk2 = 0;
  etl::vector<long,SAMPLES> XYZ_temp;
  int start = millis();

  acc_xabs = abs(X[0]);
  acc_yabs = abs(Y[0]);
  acc_zabs = abs(Z[0]);

  /* first values for max and min comparison */
  acc_xMax = abs(X[0]);
  acc_yMax = abs(Y[0]);
  acc_zMax = abs(Z[0]);
  ACNmax =XYZ[0];


  if(X.size() >= SAMPLES){
    /* Sampling window analysis */
    for (int jj = 0; jj < X.size(); jj++)
    {

      /* Max and min comparison */
      acc_xabs = abs(X[jj]);
      acc_yabs = abs(Y[jj]);
      acc_zabs = abs(Z[jj]);

      if (XYZ[jj]  > ACNmax) {
        ACNmax = XYZ[jj];
        mag_t = jj;
        amaxm = (millis() - ntpmicro) % 1000;
      }
      if ( acc_xabs > acc_xMax) {
        acc_xMax =  acc_xabs;
      }
      if (acc_yabs > acc_yMax) {
        acc_yMax = acc_yabs;
      }
      if (acc_zabs > acc_zMax) {
        acc_zMax = acc_zabs;
      }



      /* CAV arrays */
      XYZ_temp.insert(XYZ_temp.begin(),XYZ[jj]);
      sumlong += XYZ[jj];

      if (jj <_smpl) {
        sumshort += XYZ[jj];
      }


      /* ZC */
      if(jj>=1){
        if (X[jj - 1] < 0)
        xprevious = false;
        if (X[jj] < 0 )
        xcurrent = false;
        if (Y[jj] < 0 )
        ycurrent = false;
        if (Z[jj] < 0 )
        zcurrent = false;
        if (Z[jj - 1] < 0)
        zprevious = false;
        if (Y[jj - 1] < 0)
        yprevious = false;


        if (X[jj] > 0 )
        xcurrent = true;
        if (X[jj - 1] > 0)
        xprevious = true;
        if (Y[jj] > 0 )
        ycurrent = true;
        if (Y[jj - 1] > 0)
        yprevious = true;
        if (Z[jj] > 0 )
        zcurrent = true;
        if (Z[jj - 1] > 0)
        zprevious = true;



        // if the sign is different
        if ((xcurrent != xprevious) && X[jj] != 0)

        {
          // add one to the zero crossing rate
          xzcr = xzcr + 1;

        }
        if ((ycurrent != yprevious) && Y[jj] != 0)
        {
          // add one to the zero crossing rate
          yzcr = yzcr + 1;

        }
        if ((zcurrent != zprevious) && Z[jj] != 0)
        {
          // add one to the zero crossing rate
          zzcr = zzcr + 1;

        }
      }
      /*
      snprintf(line, sizeof(line), "% 3d % 2d % 2d % 2d % 4d ", sample, X[jj],Y[jj], Z[jj], XYZ[jj] );
      Serial.println(line);
      */
    }


    cavlong = (sumlong / SAMPLES) ;
    cavshort = (sumshort / _smpl) ;
    CAV = cavlong;
    ZC = (xzcr + yzcr + zzcr)*10.0 / (3.0 * 2.0);
    RSL = int((cavshort / cavlong) * 100); //

    std::sort (XYZ_temp.begin(), XYZ_temp.end(), myfunction);
    /*
    Serial.println();
    Serial.print("XYZtemp");
    for(int _i0 = 0; _i0<X.size(); _i0++){
    Serial.print(" ");
    Serial.print(XYZ_temp[_i0]);
  }
  Serial.println();
  Serial.print("XYZ");
  for(int _i0 = 0; _i0<X.size(); _i0++){
  Serial.print(" ");
  Serial.print(XYZ[_i0]);
}
Serial.println();
Serial.print("temp");
for(int _i0 = 0; _i0<X.size(); _i0++){
Serial.print(" ");
Serial.print(temp[_i0]);
}
Serial.println();
*/
//  IQR = (XYZ_temp[149]- XYZ_temp[49]);
IQR = (XYZ_temp[SAMPLES / 4 * 3 - 1] - XYZ_temp[SAMPLES / 4 - 1]);
}

return (millis() - start);
}



/* sampling every INTERVALO and SAMPLES times */
int accSampling(int _sample) {

  /* read values at sampling rate */
  if (millis() - tiempo >= INTERVALO && _sample < SAMPLES) {
    tiempo = millis();
    digitalClockDisplay();


    switch (acc_id) {
      case LSMACC:
      {
        compass.read();
        acc_x[_sample] = 0.5 * (compass.a.x / 16 - offset_x) + 0.5 * old_x;
        acc_y[_sample] = 0.5 * (compass.a.y / 16 - offset_y) + 0.5 * old_y;
        acc_z[_sample] = 0.5 * (compass.a.z / 16 - offset_z) + 0.5 * old_z;
      }
      break;
      case MMAACC:
      {
        accel.readRaw(); // mma
        acc_x[sample] = 0.5 * (accel.x - offset_x) + 0.5 * old_x;
        acc_y[sample] = 0.5 * (accel.y - offset_y) + 0.5 * old_y;
        acc_z[sample] = 0.5 * (accel.z - offset_z) + 0.5 * old_z;

      }
      break;
      case ADXLACC:
      {
        sca = accelerometer.readmg();
        acc_x[sample] = 0.5 * (sca.XAxis - offset_x) + 0.5 * old_x;
        acc_y[sample] = 0.5 * (sca.YAxis - offset_y) + 0.5 * old_y;
        acc_z[sample] = 0.5 * (sca.ZAxis - offset_z) + 0.5 * old_z;

      }
      break;
      default:
      {
        Serial.println("MATRIX ERROR");
      }
      break;
    }

    old_x = acc_x[_sample];
    old_y = acc_y[_sample];
    old_z = acc_z[_sample];

    float aax = acc_x[_sample] * acc_x[_sample];
    float aay = acc_y[_sample] * acc_y[_sample];
    float aaz = acc_z[_sample] * acc_z[_sample];

    AccNetnow[_sample] = sqrt(aax + aay + aaz) * 100.0; // net aceleration with 2 decimals integer
    char line[30];
    snprintf(line, sizeof(line), " % 3d % 2d % 2d % 2d % 4d  ", sample, acc_x[_sample], acc_y[_sample], acc_z[_sample], AccNetnow[_sample] );
    Serial.print(line);
    return 0;
  }
  return 1;
}


/* Calcs CAV , RSL , IQR and ZC parameters and returns time
of the operation in milliseconds */
time_t calcParam(int _sample) {
  long sumshort = 0;
  long sumlong = 0;
  float cavlong = 0;
  float cavshort = 0;
  float _smpl = 25.0;
  bool xcurrent;
  bool xprevious;
  bool ycurrent;
  bool yprevious;
  bool zcurrent;
  bool zprevious;
  short xzcr = 0;
  short yzcr = 0;
  short zzcr = 0;
  long temp[SAMPLES];
  long temp1[SAMPLES];
  time_t startcalc;
  short acc_xabs;
  short acc_yabs;
  short acc_zabs;
  short kkkk;
  short kkk2 = 0;



  startcalc = millis();
  acc_xabs = abs(acc_x[0]);
  acc_yabs = abs(acc_y[0]);
  acc_zabs = abs(acc_z[0]);

  /* first values for max and min comparison */
  acc_xMax = acc_xabs;
  acc_yMax = acc_yabs;
  acc_zMax = acc_zabs;
  ACNmax = AccNetnow[0];

  /* Sampling window analysis */
  for (int jj = 1; jj < SAMPLES; jj++)
  {

    /* Max and min comparison */
    acc_xabs = abs(acc_x[jj]);
    acc_yabs = abs(acc_y[jj]);
    acc_zabs = abs(acc_z[jj]);

    if (AccNetnow[jj]  > ACNmax) {
      ACNmax = AccNetnow[jj];
      mag_t = jj;
      amaxm = (millis() - ntpmicro) % 1000;
    }
    if (AccNetnow[jj] <= ACNmin) {
      ACNmin = AccNetnow[jj];
    }
    if ( acc_xabs > acc_xMax) {
      acc_xMax =  acc_xabs;
    }
    if (acc_yabs > acc_yMax) {
      acc_yMax = acc_yabs;
    }
    if (acc_zabs > acc_zMax) {
      acc_zMax = acc_zabs;
    }

    /* CAV arrays */
    temp[jj - 1] = AccNetnow[jj - 1];
    temp1[jj - 1] = AccNetnow[jj - 1];


    sumlong += AccNetnow[jj - 1];
    kkkk = _sample - jj + 1;
    if (kkkk <= 0) {
      kkkk = (SAMPLES) - (jj - _sample);
    }

    if (jj <= _smpl) {
      sumshort += AccNetnow[kkkk];
    }


    if (acc_x[jj - 1] < 0)
    xprevious = false;
    if (acc_x[jj] < 0 )
    xcurrent = false;
    if (acc_y[jj] < 0 )
    ycurrent = false;
    if (acc_z[jj] < 0 )
    zcurrent = false;
    if (acc_z[jj - 1] < 0)
    zprevious = false;
    if (acc_y[jj - 1] < 0)
    yprevious = false;


    if (acc_x[jj] > 0 )
    xcurrent = true;
    if (acc_x[jj - 1] > 0)
    xprevious = true;
    if (acc_y[jj] > 0 )
    ycurrent = true;
    if (acc_y[jj - 1] > 0)
    yprevious = true;
    if (acc_z[jj] > 0 )
    zcurrent = true;
    if (acc_z[jj - 1] > 0)
    zprevious = true;



    // if the sign is different
    if ((xcurrent != xprevious) && acc_x[jj] != 0)
    {
      // add one to the zero crossing rate
      xzcr = xzcr + 1;

    }
    if ((ycurrent != yprevious) && acc_y[jj] != 0)
    {
      // add one to the zero crossing rate
      yzcr = yzcr + 1;

    }
    if ((zcurrent != zprevious) && acc_z[jj] != 0)
    {
      // add one to the zero crossing rate
      zzcr = zzcr + 1;

    }
  }
  cavlong = (sumlong / SAMPLES) ;
  cavshort = (sumshort / _smpl) ;
  CAV = cavlong;
  ZC = (xzcr + yzcr + zzcr) / (3 * 2);
  RSL = int((cavshort / cavlong) * 100.0); //
  //  sortArray(temp, SAMPLES);
  std::sort(temp1, temp1 + SAMPLES);
  /*
  Serial.println();
  Serial.print("temp");
  for(int _i0 = 0; _i0<SAMPLES; _i0++){
  Serial.print(" ");
  Serial.print(temp[_i0]);
}
Serial.println();
Serial.print("temp1");
for(int _i0 = 0; _i0<SAMPLES; _i0++){
Serial.print(" ");
Serial.print(temp[_i0]);
}
Serial.println();
Serial.print("ACC");

for(int _i0 = 0; _i0<SAMPLES; _i0++){
Serial.print(" ");
Serial.print(AccNetnow[_i0]);
}

Serial.println();
*/
IQR = (temp1[151] - temp1[51]);
//IQR = (temp[SAMPLES / 4 * 3 - 1] - temp[SAMPLES / 4 - 1]);
char line[160];
snprintf(line, sizeof(line),
" % 1lu[ms] % 2d % 2d % 2d % 4d " " % 3d % 4d % 4d % 4d % 3d % 4d % 4d % 4d ",
(millis() - startcalc), acc_xMax, acc_yMax, acc_zMax, ACNmax, ZC, IQR, CAV, RSL, ZCref, IQRref, CAVref, RSLref);
Serial.print(line);
return (millis() - startcalc);
}

/* get sign of number */
byte getSign(short data)
{
  if (data > 0)   /* positif data */
  return (1);
  if (data < 0)         /* negatif data */
  return (0);
  else
  return (2); /*zero data */
}

/*
Calc references with max and min values of the parameteres
return 1 if ok and 0 if still calculating
*/

int CalcRef(int _i, bool _iscalc, int _samples) {

  char line[160];
  if (_iscalc) { // true if variable reference

    if (_i < _samples) { // use total _samples for calculation

      if (_i >= 400) { //use the samples -1 values to find max and min

        if (IQR > IQRmax) {
          IQRmax = IQR;
        }
        if (IQR <= IQRmin) {
          IQRmin = IQR;
        }

        if (ZC > ZCmax) {
          ZCmax = ZC;
        }
        if (ZC <= ZCmin) {
          ZCmin = ZC;
        }

        if (CAV > CAVmax) {
          CAVmax = CAV;
        }
        if (CAV <= CAVmin) {
          CAVmin = CAV;
        }

        if (RSL > RSLmax) {
          RSLmax = RSL;
        }
        if (RSL <= RSLmin) {
          RSLmin = RSL;
        }

        Serial.print("Max/Min setting for ");
        Serial.print(_i);
        Serial.print(" count");
        /* Calculates de references each time for debug */
        IQRref = max(300, (IQRmax - IQRmin) * Factor_IQR);
        CAVref = max(500, (CAVmax - CAVmin) * Factor_CAV);
        ZCref =  float((ZCmax + ZCmin) / 2) - float((ZCmax - ZCmin) * Factor_ZC);
        RSLref = max(155, (RSLmax - RSLmin) * Factor_RSL);

      }

      else { // use the first value for max and min

        IQRmax = -1;
        IQRmin = 9999999;
        ZCmax = -1;
        ZCmin = 32767;
        CAVmax = -1;
        CAVmin = 9999999;
        RSLmax = -1;
        RSLmin = 32767;

        Serial.print("Sensor stabilizing for ");
        Serial.print(_i);
        Serial.print(" count");

      }


      snprintf(line, sizeof(line), " %d %d %d %d %d %d %d %d %d %d %d %d",
      ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, RSLmax, RSLmin, ZCref, IQRref, CAVref, RSLref);
      Serial.print(line);
      return 0; // still calculates until samples are done
    }

  }


  else { // if _iscalc false use fixed reference

    IQRref = 500;  // [mg] integer
    CAVref = 1000; // [mg] integer
    RSLref = 300;  // percent para medir la ligua
    ZCref =  22;
    char line[160];
    snprintf(line, sizeof(line), "FIXED REF %d %d %d %d ", ZCref, IQRref, CAVref, RSLref);
    Serial.print(line);

  }
  return 1; // finish when samples are done || isclac us false
}


const  short max(const short& a, const short& b)
{
  return (a < b) ? b : a;
}

const  long max(const long& a, const long& b)
{
  return (a < b) ? b : a;
}
const  int max(const int& a, const int& b)
{
  return (a < b) ? b : a;
}
void DRAcc_ISR () {

}

void restore_parameters() {
  IQRmax = 0;
  IQRmin = 0;
  CAVmax = 0;
  CAVmin = 0;
  ACNmax = 0;
  ACNmin = 0;
  ZCmax = 0;
  ZCmin = 0;
}

void offset(int samples) {
  int j = 0;
  offset_x = 0;
  offset_y = 0;
  offset_z = 0;
  while (j < samples) {
    if (millis() - tiempo >= INTERVALO) {
      tiempo = millis();
      switch (acc_id) {
        case LSMACC:
        {
          compass.read();
          offset_x += compass.a.x / 16;
          offset_y += compass.a.y / 16;
          offset_z += compass.a.z / 16;
        }
        break;
        case MMAACC:
        {
          accel.readRaw();
          offset_x += accel.x;
          offset_y += accel.y;
          offset_z += accel.z;
        }
        break;
        case ADXLACC:
        {
          sca = accelerometer.readmg();
          offset_x += sca.XAxis;
          offset_y += sca.YAxis;
          offset_z += sca.ZAxis;
        }
        break;
        default:
        {
          Serial.println("MATRIX ERROR");
        }
        break;
      }
      j++;
    }
  }

  offset_x = offset_x / samples;
  offset_y = offset_y / samples;
  offset_z = offset_z / samples;


}



int sendPost(time_t _time) {

  WiFiClient client; // Use WiFiClient class to create TCP connections
  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    http.begin("http://prosismic.zeke.cl/registrarEvento"); //HTTP
    http.addHeader("Content-Type", "text/plain"); // we will just send a simple string in the body.
    char line[160];
    snprintf(line, sizeof(line), "%s;%lu;%lu;%d;%lu;%d;%d;%d;%d;%d;%d;%s;%s",
    ID, _time, (millis() - ntpmicro) % 1000, ACNmax, trigger, ZCref, IQRref, CAVref,
    ZC, IQR, CAV, latitude, longitude);
    //  Serial.print("Event ");
    //  Serial.println(line);
    int httpCode = http.POST(line);
    String payload = http.getString();
    http.end();
    if (payload.equals("1\n")) {
      return 0;
    }
    else {
      return 1;
    }
  } else {

    return 1;
  }
}

int sendStatus(int _status) {
  /*
  1. Activo/Habilitado
  2. Activo/Inhibido
  3 .Activo/error de parametro
  4 ....
  5  ....
  4 y 5 por definir
  */
  WiFiClient client; // Use WiFiClient class to create TCP connections
  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    http.begin("http://prosismic.zeke.cl/registrarEstado"); //HTTP
    http.addHeader("Content-Type", "text/plain"); // we will just send a simple string in the body.

    char line[160];
    snprintf(line, sizeof(line), "%s;%lu;%d"
    ,ID, now(),_status);
    //Serial.print("Event ");
    //Serial.println(line);
    int httpCode = http.POST(line);
    String payload = http.getString();
    http.end();
    if (payload.equals("1\n")) {
      return 0;
    }
    else {
      //  Serial.print("   payload  ");
      //    Serial.print(payload);    //Print request response payload
      return 1;
    }
  } else {
    return 1;
  }
}

int sendStatusPost(time_t _time) {

  WiFiClient client; // Use WiFiClient class to create TCP connections
  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    http.begin(UBIDIR); //HTTP
    http.addHeader("Content-Type", "application/json"); // we will just send a simple string in the body.

    char line[160];
    snprintf(line, sizeof(line), "{\"value\":%d, \"context\":{\"lat\":%s, \"lng\":%s}}"
    , RSL, latitude, longitude);
    //Serial.print("Event ");
    //Serial.println(line);
    int httpCode = http.POST(line);
    String payload = http.getString();
    http.end();
    if (payload.equals("1\n")) {
      return 0;
    }
    else {
      //    Serial.print("   payload  ");
      //    Serial.print(payload);    //Print request response payload
      return 1;
    }
  } else {

    return 1;
  }
}

void print_mag(int _sample) {
  Serial.print(AccNetnow[_sample]);
}

void digitalClockDisplay()
{
  Serial.print(" ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());
  Serial.print(" ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(".");
  printmillisntp(millis());

}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
  Serial.print('0');
  Serial.print(digits);
}
void printmillisntp(time_t _millis) {
  char line[10];
  snprintf(line, sizeof(line), "%d ", (_millis - ntpmicro) % 1000);
  Serial.print(line);
}


/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address
  byte server = 1;
  do {
    switch (server) {
      case 1:
      {
        strcpy(ntpServerName, "us.pool.ntp.org");
        //    Serial.println("us.pool.ntp.org");
        server = 2;
      }
      break;
      case 2:
      {
        strcpy(ntpServerName, "time-a.timefreq.bldrdoc.gov");
        //  Serial.println("time-a.timefreq.bldrdoc.gov");
        server = 3;
      }
      break;
      case 3:
      {
        strcpy(ntpServerName, "time-b.timefreq.bldrdoc.gov");
        //  Serial.println("time-b.timefreq.bldrdoc.gov");
        server = 4;
      }
      break;
      case 4:
      {
        strcpy(ntpServerName, "time-c.timefreq.bldrdoc.gov");
        //  Serial.println("time-c.timefreq.bldrdoc.gov");
        server = 5;
      }
      break;
      default:
      {
        strcpy(ntpServerName, "time.nist.gov");
        //  Serial.println("time.nist.gov");
        server = 0;
      }
      break;
    }


    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    //Serial.println("Transmit NTP Request");
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    //Serial.print(ntpServerName);
    //Serial.print(": ");
    //Serial.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
      int size = Udp.parsePacket();
      if (size >= NTP_PACKET_SIZE) {
        //Serial.println("Receive NTP Response");
        Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
        unsigned long secsSince1900;
        // convert four bytes starting at location 40 to a long integer
        secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
        secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
        secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
        secsSince1900 |= (unsigned long)packetBuffer[43];
        //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
        return secsSince1900 - 2208988800UL;
      }
    }

  } while (server != 0);
  //Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress & address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
