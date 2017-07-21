/* LUNES 17 julio al 2017
   REVISAR
   - calculo de parametros y ooffset cada 10 min
   - rutina de inhibición de disparos indeseados (5 disparos en 15 segundos)
   - revisar conección con asistente wifi
   - revisar el overflow de parametros
   TRABAJO PENDIENTE
   - 400 datos previos a un sismo
   - rutina de inhibición de disparos indeseados (3 disparos separados por 3 segundos en una ventana de 60 segundos)
   - Revisar en serivor rutina  Emisión de Aviso de Reporte
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LSM303.h>
#include <ADXL345.h>
#include <SparkFun_MMA8452Q.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoSort.h>
#include <ArduinoJson.h>
#include <WifiLocation.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <StackArray.h>

/*
  ID 1 = "Pato"
  ID 2 = "Isi"
  ID 3 = "Chrysalis"
  ID 4 = "Zeke I+D"
  ID 5 = "Zeke Stgo"
*/

#define N_OFFSET 32
#define SAMPLES 200           // samples number
#define N_WIN 2               // Windwos 

#define TRNM 1UL             // Parameter in between windwos time in hour
#define T_OFFSET 1          // offset recalculation in minutes
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

int WifiPin = D6;     // wifi status LED
int AccPin = D7;      // accelerometer data readings LED

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
time_t startTime;
time_t t_online;      //

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

// variabales counters per samples
int psample;                   // sample counter for parameters calculation
int s_cserver;                 // samples counter when server is not responding
int sample;                    // sample index
int c_mevent;                  // movement event counter
int c_pdevent;                 // prolongate seism and displacement for seims event counter
int eventsample;               // samples after first event detected

// timer counters
int tc_mevent;                 // cunter for an event start && movement checker
int tc_pdevent;                // for event start && displacement checker
int tc_temp;                   // counter temporal for PD event

//acelerometer objets
LSM303 compass;               // acelerometer objet
MMA8452Q accel;
ADXL345 accelerometer;
acc sca;

// acceleromter variables

long offset_x, offset_y, offset_z;         // offset data for normalization
short old_x, old_y, old_z;                 // old acceleration data for normalization
long AccNetnow[SAMPLES];                   // net acceleration
short acc_x[SAMPLES];                      // xyz axis acceleration array
short acc_y[SAMPLES];                      // xyz axis acceleration array
short acc_z[SAMPLES];                      // xyz axis acceleration array
short acc_xMax, acc_yMax, acc_zMax;        // xyzaxis max acceleration
short mag_t;                               // sample number of aceleration magnitude



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


void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WifiPin, OUTPUT);
  pinMode(AccPin, OUTPUT);


  Serial.begin(500000);
  // Wire.begin(int sda, int scl)
  // Wire.begin(4, 5);       // join i2c bus (address optional for master)
  Wire.begin();       // join i2c bus (address optional for master)
  while (!Serial) ; // Needed for Leonardo only
  delay(250);
  //reset saved settings
  // wifiManager.resetSettings();


  //first parameter is name of access point, second is the password
  wifiManager.autoConnect();

  Serial.println("Prosismic");
  Serial.print("Connected to ");
  // Serial.println(ssid);
  // WiFi.begin(ssid, pass);

  /* wait until wifi is connected*/
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(WifiPin, LOW);
  }
  digitalWrite(WifiPin, HIGH);
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

  switch (acc_id) {
    case LSMACC:
      {
        compass.enableDefault();
        snprintf(ID, sizeof(ID), "PS%dIMU", ESP.getChipId() );
        snprintf(UBIDIR, sizeof(UBIDIR)
                 , "http://things.ubidots.com/api/v1.6/devices/PS%dIMU/RSL/values?token=wsFGicZyoCaq4uVDQcDm2btS3YpahA"
                 , ESP.getChipId());
        Serial.println("LSM303DLH setup complete!");
      }
      break;
    case MMAACC:
      {
        snprintf(ID, sizeof(ID), "PS%dMMA", ESP.getChipId() );
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
        snprintf(ID, sizeof(ID), "PS%dADXL", ESP.getChipId() );
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

  /* time variables */
  t_offsetlap = now();
  t_online = now();
  t_mevent = 0;
  t_dpevent = 0;
  wifiLap = now();
  ntpmicro = millis();
  startTime = millis();

  /* status variables */
  DATASEND = false;
  ISCALC = true;
  EVENT = false;
  CSERVER = true;
  REFERENCE = false;
  MCHECK = false;


  /*Device ID */







  /* after 3 seconds calculate the offset */
  delay(3000);
  offset(N_OFFSET);  // 50 samples for the offset media
  Serial.println();



  /* start taking samples */
  digitalWrite(AccPin, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

}



void loop()
{

  if (!accSampling(sample)) {
    digitalWrite(AccPin, HIGH);
    calcParam(sample);
    trigger = 0;


    //check for sensor movement


    /* check for sensor movement */
    if (MCHECK) {
      if (tc_mevent <= 1500) { // check 1500 count

        Serial.print("\t MEvent number ");
        Serial.print(c_mevent);
        Serial.print("\t");
        Serial.print(1500 - tc_mevent);
        Serial.print(" samples for MWindow");
        tc_mevent++;

        if (c_mevent >= 5) { // are 5 event
          DATASEND = false;
          EVENT = false;
          MCHECK = false;
          eventsample = 0; // ready for next eventsamples
          Serial.print(" Pause event sending until offset");
        }

      }

      else {
        tc_mevent = 0;
        c_mevent = 0;
        Serial.print(" Restart 1500 count windwow");
        if (EVENT == false)
          MCHECK = false;
      }
    }


    /* check for sensor small displacement and prolongate seism 
    if (DPCHECK == true) {
      if (tc_pdevent <= 6000) { // check 6000 count windwow

        Serial.print("\t PDEvent number ");
        Serial.print(c_pdevent);
        Serial.print("\t");
        Serial.print(6000 - tc_pdevent);
        Serial.print(" count for PDWindow");
        Serial.print("\t");
        Serial.print((tc_temp));
        Serial.print(" count since last PDEvent " );
        tc_pdevent++;
        tc_temp++;

        if (tc_temp >= 300) {
         
          
          Serial.print(c_pdevent);
          Serial.print("/3 PDevent count");
          tc_temp = 0;

          if ( c_pdevent >= 3) {
            DATASEND = false;
            EVENT = false;
            eventsample = 0; // ready for next eventsamples

          }

          else {
            c_pdevent = 0;
            tc_pdevent = 0;
            tc_temp = 0;
            DPCHECK = false;
          }

        }

      }

      else {
        t_dpevent = 0;
        c_pdevent = 0;
        t_temp = 0;
        DPCHECK = false;
        Serial.print(" Restore P&D 60 sec windwow");
      }

    }

*/

    /* count samples for restore event detector
      if (MCHECK) {
      if ((millis() - t_mevent) <= 15000) { // check 15 seconds windwow

      Serial.print("\t MEvent number ");
      Serial.print(c_mevent);
      Serial.print("\t");
      Serial.print(15000 - (millis() - t_mevent));
      Serial.print(" secs for MWindow");

      if (c_mevent >= 5) { // are 5 event
      DATASEND = false;
      EVENT = false;
      MCHECK = false;
      eventsample = 0; // ready for next eventsamples
      Serial.print(" Pause event sending until offset");
      }

      }

      else {
      t_mevent = 0;
      c_mevent = 0;
      Serial.print(" Restart 15 secs windwow");
      if (EVENT == false)
      MCHECK = false;
      }
      }
    */

    /* count samples for restore event detector */
    if (EVENT) { // pause after eventis detected

      /* check if event count complete */
      if (eventsample >= SAMPLES / 2) {
        eventsample = 0;
        DATASEND = true;
        EVENT = false;
        digitalWrite(LED_BUILTIN, LOW);
      }
      else {
        Serial.print("  Pause even for ");
        Serial.print(SAMPLES / 2 - eventsample);
        Serial.print(" eventsamples");
        eventsample++;
        digitalWrite(LED_BUILTIN, HIGH);
      }

    }


    if (!CSERVER) { // wait for server resend
      Serial.print("  Pause even for ");
      Serial.print(SAMPLES / 2 - s_cserver);
      Serial.print(" s_cserver");
      s_cserver++;
      digitalWrite(LED_BUILTIN, HIGH);
      /* check server count complete */
      if (s_cserver >= SAMPLES / 2) {
        s_cserver = 0;
        DATASEND = true;
        CSERVER = true;
        digitalWrite(LED_BUILTIN, LOW);
      }
    }





    /* reclaculate offset and reference */
    if (!DATASEND && REFERENCE) {
      if (CalcRef(psample, ISCALC, SAMPLES * 5)) {
        Serial.print("References Calculated");
        DATASEND = true;
        REFERENCE = false;
        psample = 0;
        sample = 0;

        t_dpevent = 0;
        c_pdevent = 0;
        t_temp = 0;
        DPCHECK = false;
        tc_mevent = 0;
        c_mevent = 0;
        MCHECK = false;

      }
    }



    if (!DATASEND && (millis() - startTime) <= TST) {
      // use for the first 3SAMPLES samples, for 10[ms] and 100 khz = 3 [s]
      if (CalcRef(psample, ISCALC, SAMPLES * 5)) {
        DATASEND = true; //enable seism event
        psample = 0;
      }
    }




    Serial.println();
    sample++; // window sample counter
    psample++;// parameter calculator counter
  }


  /* restore sampling window */
  if (sample == SAMPLES)
    sample = 0;


  /* sync the milliseconds with ntp time */
  if (timeStatus() != timeNotSet) {
    if (now() != prevtime) { //update the display only if time has changed
      prevtime = now();
      ntpmicro = millis();
    }
  }


  /* Sesim event detect and check every 1 TASD secs after first detection*/

  trigger = 0;
  // if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
  if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref)) {
    if ((IQR > IQRref && ZC < ZCref && CAV > CAVref)) trigger = 2;
    // if (RSL >= RSLref)  trigger = 1;

    if (DATASEND) {


      DATASEND = false;
      digitalWrite(LED_BUILTIN, HIGH);
      digitalClockDisplay();

      c_mevent++;
      if (MCHECK == false) {
        tc_mevent = 0;
        MCHECK = true;
      }

      c_pdevent++;
      if (DPCHECK == false) {
        tc_pdevent = 0;
        tc_temp = 0;
        DPCHECK = true;
      }


      if (!sendPost(now())) { // if the server response
        EVENT = true;
        t_offsetlap = now();
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(AccPin, HIGH);
      }

      else { //if the server does not respones
        CSERVER = false;
        Serial.print("Server not found. Cannot send event");
        digitalWrite(AccPin, LOW);
        digitalWrite(LED_BUILTIN, LOW);
      }

      Serial.println();
    }

  }



  /* offset recalcs every TRNM minutes*/
  if ((now() - t_offsetlap) >= (SECS_PER_MIN * T_OFFSET)) {
    Serial.print("Offset recalculation ");
    t_offsetlap = now();
    if (!EVENT) {
      offset(N_OFFSET);
      REFERENCE = true;
      DATASEND = false;
      psample = 0;
    }
  }

  /* Status send*/
  if ((now() - t_online) >= (SECS_PER_HOUR * T_STATUS)) {
    Serial.print("Sensor Report Status");
    t_online = now();
    sendStatusPost(t_online);
  }

  /* Check wifi connection */
  if ((now() - wifiLap) == (SECS_PER_MIN * TWC)) {
    wifiLap = now();
    /* wait until wifi is connected*/
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      digitalWrite(WifiPin, LOW);
    }
    digitalWrite(WifiPin, HIGH);
    Serial.println("");
  }

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
  for (int i = 1; i < SAMPLES; i++)
  {

    
    
 
    
    /* Max and min comparison */
    acc_xabs = abs(acc_x[i]);
    acc_yabs = abs(acc_y[i]);
    acc_zabs = abs(acc_z[i]);

    if (AccNetnow[i]  > ACNmax) {
      ACNmax = AccNetnow[i];
      mag_t = i;
      amaxm = (millis() - ntpmicro) % 1000;
    }
    if (AccNetnow[i] <= ACNmin) {
      ACNmin = AccNetnow[i];
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
    temp[i - 1] = AccNetnow[i - 1];

    
    sumlong += AccNetnow[i - 1];



    kkkk = _sample - i + 1;
        
    if (kkkk <= 0) {
    kkkk = (SAMPLES) - (i - _sample);
    }

    if (i <= _smpl) {
      sumshort += AccNetnow[kkkk];
    }


    if (acc_x[i - 1] < 0)
      xprevious = false;
    if (acc_x[i] < 0 )
      xcurrent = false;
    if (acc_y[i] < 0 )
      ycurrent = false;
    if (acc_z[i] < 0 )
      zcurrent = false;
    if (acc_z[i - 1] < 0)
      zprevious = false;
    if (acc_y[i - 1] < 0)
      yprevious = false;


    if (acc_x[i] > 0 )
      xcurrent = true;
    if (acc_x[i - 1] > 0)
      xprevious = true;
    if (acc_y[i] > 0 )
      ycurrent = true;
    if (acc_y[i - 1] > 0)
      yprevious = true;
    if (acc_z[i] > 0 )
      zcurrent = true;
    if (acc_z[i - 1] > 0)
      zprevious = true;



    // if the sign is different
    if ((xcurrent != xprevious) && acc_x[i] != 0)
    {
      // add one to the zero crossing rate
      xzcr = xzcr + 1;

    }
    if ((ycurrent != yprevious) && acc_y[i] != 0)
    {
      // add one to the zero crossing rate
      yzcr = yzcr + 1;

    }
    if ((zcurrent != zprevious) && acc_z[i] != 0)
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
  sortArray(temp, SAMPLES);
  IQR = (temp[SAMPLES / 4 * 3 - 1] - temp[SAMPLES / 4 - 1]);
  char line[160];
  snprintf(line, sizeof(line), " % 1lu[ms] % 2d % 2d % 2d % 4d % 3d % 4d % 4d % 4d % 3d % 4d % 4d % 4d ",
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

        Serial.print("First parameter setting for ");
        Serial.print(_i);
        Serial.print(" count");

      }

      /* Calculates de references each time for debug */
      IQRref = max(300, (IQRmax - IQRmin) * Factor_IQR);
      CAVref = max(500, (CAVmax - CAVmin) * Factor_CAV);
      ZCref =  float((ZCmax + ZCmin) / 2) - float((ZCmax - ZCmin) * Factor_ZC);
      RSLref = max(155, (RSLmax - RSLmin) * Factor_RSL);





      snprintf(line, sizeof(line), "\t % d % d % d % d % d % d % d % d \t % d % d % d % d",
               ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, RSLmin, RSLmax, ZCref, IQRref, CAVref, RSLref);
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
    snprintf(line, sizeof(line), "FIXED REF % d % d % d % d ", ZCref, IQRref, CAVref, RSLref);
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
  char line[40];
  snprintf(line, sizeof(line), "Offset(x, y, z) = ( % 2d, % 2d, % 2d) \n", offset_x, offset_y, offset_z);
  Serial.print(line);

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
    Serial.print("Event ");
    Serial.println(line);
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
    digitalWrite(WifiPin, LOW);
    Serial.println("Error in WiFi connection");
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
    Serial.print("Event ");
    Serial.println(line);
    int httpCode = http.POST(line);
    String payload = http.getString();
    http.end();
    if (payload.equals("1\n")) {
      return 0;
    }
    else {
      Serial.print("   payload  ");
      Serial.print(payload);    //Print request response payload
      return 1;
    }
  } else {
    digitalWrite(WifiPin, LOW);
    Serial.println("Error in WiFi connection");
    return 1;
  }
}

void print_all_raw(int _sample) {
  digitalClockDisplay();
  Serial.print("  ");
  Serial.print(_sample);
  Serial.print("   ");
  print_acc_raw();
  Serial.print("   ");
  print_acc_values_raw(_sample);
  Serial.print("   ");
  print_mag(_sample);
  Serial.print("\t");
  print_parameter_raw();

}
void print_parameter() {
  Serial.print("IQR");
  Serial.print("    ");
  Serial.print(IQR);
  Serial.print("    ");
  Serial.print("CAV");
  Serial.print("    ");
  Serial.print(CAV);
  Serial.print(" ");
  Serial.print("ZC");
  Serial.print("    ");
  Serial.print(ZC);
  Serial.print("    ");
  Serial.print("ACNmax");
  Serial.print("    ");
  Serial.print(ACNmax);
  Serial.print("    ");
  Serial.print("RSL");
  Serial.print("    ");
  Serial.print(RSL);
}

void print_acc_values(int _sample) {
  Serial.print("x-axis");
  Serial.print("   ");
  Serial.print(acc_x[_sample]);
  Serial.print("   ");
  Serial.print("y-axis");
  Serial.print("   ");
  Serial.print(acc_y[_sample]);
  Serial.print("   ");
  Serial.print("z-axis");
  Serial.print("   ");
  Serial.print(acc_z[_sample]);
  Serial.print("   ");
  Serial.print("NA");
  Serial.print("    ");
  Serial.print(AccNetnow[_sample]);
}


void print_ref() {
  Serial.print("IQRref");
  Serial.print("  ");
  Serial.print(IQRref);
  Serial.print("  ");
  Serial.print("CAVref");
  Serial.print(" ");
  Serial.print(CAVref);
  Serial.print("  ");
  Serial.print("ZCref");
  Serial.print("  ");
  Serial.print(ZCref);
  Serial.print("  ");
  Serial.print("RSLref");
  Serial.print("  ");
  Serial.print(RSLref);
}

void print_parameter_raw() {
  Serial.print(IQR);
  Serial.print("\t");
  Serial.print(CAV);
  Serial.print("\t");
  Serial.print(ZC);
  Serial.print("\t");
  Serial.print(ACNmax);
  Serial.print("\t");
  Serial.print(RSL);
}

void print_acc_values_raw(int _sample) {

  Serial.print(acc_x[_sample]);
  Serial.print("   ");
  Serial.print(acc_y[_sample]);
  Serial.print("   ");
  Serial.print(acc_z[_sample]);
  Serial.print("   ");
}

void print_acc_raw() {

  Serial.print(compass.a.x);
  Serial.print("   ");
  Serial.print(compass.a.y);
  Serial.print("   ");
  Serial.print(compass.a.z);
  Serial.print("   ");
}

void print_ref_raw() {

  Serial.print(IQRref);
  Serial.print(" ");
  Serial.print(CAVref);
  Serial.print(" ");
  Serial.print(ZCref);
  Serial.print(" ");
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
  Serial.print("   ");
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
  snprintf(line, sizeof(line), "%d\t", (_millis - ntpmicro) % 1000);
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
          Serial.println("us.pool.ntp.org");
          server = 2;
        }
        break;
      case 2:
        {
          strcpy(ntpServerName, "time-a.timefreq.bldrdoc.gov");
          Serial.println("time-a.timefreq.bldrdoc.gov");
          server = 3;
        }
        break;
      case 3:
        {
          strcpy(ntpServerName, "time-b.timefreq.bldrdoc.gov");
          Serial.println("time-b.timefreq.bldrdoc.gov");
          server = 4;
        }
        break;
      case 4:
        {
          strcpy(ntpServerName, "time-c.timefreq.bldrdoc.gov");
          Serial.println("time-c.timefreq.bldrdoc.gov");
          server = 5;
        }
        break;
      default:
        {
          strcpy(ntpServerName, "time.nist.gov");
          Serial.println("time.nist.gov");
          server = 0;
        }
        break;
    }


    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    Serial.println("Transmit NTP Request");
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    Serial.print(ntpServerName);
    Serial.print(": ");
    Serial.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
      int size = Udp.parsePacket();
      if (size >= NTP_PACKET_SIZE) {
        Serial.println("Receive NTP Response");
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
  Serial.println("No NTP Response :-(");
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
