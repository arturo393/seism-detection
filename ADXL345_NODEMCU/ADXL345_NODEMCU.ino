#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <ADXL345.h>
#include <Filters.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoSort.h>
#include <ArduinoJson.h>
#include <WifiLocation.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

/*
  ID 1 = "Pato"
  ID 2 = "Isi"
  ID 3 = "Chrysalis"
  ID 4 = "Zeke I+D"
  ID 5 = "Zeke Stgo"
*/
#define SAMPLES 200           // samples number
#define N_WIN 6               // Windwos 
#define ID 2                  // device number identification
#define TRNM 1UL             // Parameter in between windwos time in hour
#define TST 10000UL                // Start parameter windwos in secs
#define TRNS 40               // Sample windows time in millisecs
#define TASD 1000               // time after send data in msecs
#define TWC 1                 // time Wifi check
#define Factor_IQR  4.0         // Interquarile reference factor
#define Factor_CAV  7         // Cumulative acelerator vector reference factor
#define Factor_ACN  3.0         // Aceleration magnitude reference factor
#define Factor_ZC   0.25      // Zero Crossing rate reference factor
#define Factor_RSL  7
#define StartWindow 20
//#define _SSID "katyred"  // network name
//#define _NETPASS "katynoseanflaites" // network password
//#define _SSID "zekefi-interno"  // network name
//#define _NETPASS "JtXDF5jK79es" // network password
//#define _SSID "zekefi"  // network name
//#define _NETPASS "0000000000001500000000000015" // network password
//#define _SSID "VTR-3040015"   // network name+
//#define _NETPASS "r6msMmgncj6x" // network password
//#define _SSID "Familia"   // network name
//#define _NETPASS "1234familia" // network password

#define OFFSET true              // set median dc offset remove
#define REF_VAR  false          // indicate if reference are variables or fixed values

int WifiPin = D7;     // wifi status LED
int AccPin = D6;      // accelerometer data readings LED
//int WifiPin;     // wifi status LED
//int AccPin;      // accelerometer data readings LED

/* Sampling freq */
#define FrecRPS  100                   // Tasa de muestreo definida por el usuario (100 a 200)
unsigned int INTERVALO = (1000 / FrecRPS); // s

/* time variables */
time_t tiempo;        // ?
time_t prevtime;      // time for check every one second
time_t t_offsetlap;   // time between offset recalculation
time_t sendTime;      // when an event start
time_t wifiLap;       // counter time for wifi check
time_t ntpmicro;      // ntp microseconds
time_t eventAccion;   // time after event is on
time_t amaxm;         // millsecond of the max acceleration
time_t startTime;


/* accelerometer parameters and variables  */
int psample;  // sample counter for parameters calculation
bool ISCALC;  // parameter calculation enable/disable
bool PARAMCALC; // parameter calculation event
bool EVENT;             // event detection on/off
int eventsample = 0 ; // samples after first event detected
bool DATASEND;             // enable/disable send event to server
int sample;                // sample index
int prevsample;               // sample index
ADXL345 accelerometer;
Vector sca;
short trigger;                      // 0:notrigger / 1:RSL / 2:ZIC
long offset_x, offset_y, offset_z; // offset data for normalization
short old_x, old_y, old_z;         // old acceleration data for normalization
short ACC_sample[3];              // [0] = x_acc [1] = y_acc [z] = z_acc
int AccNetnow[SAMPLES]; // net acceleration
short acc_x[SAMPLES];     // x-axis acceleration array
short acc_y[SAMPLES];     // x-axis acceleration array
short acc_z[SAMPLES];     // x-axis acceleration array
short acc_xMax;           // x-axis max acceleration
short acc_yMax;           // y-axis max acceleration
short acc_zMax;           // z-axis max acceleration
float AccNetnowMax;       // max net acceleration
short mag_t;              // sample number of aceleration magnitude
short x_t;                // sample number of aceleration x-axis
short y_t;                // sample number of aceleration y-axis
short z_t;                // sample number of aceleration z-axis
short IQRmax;  // maximun value Interquiarlie
short IQRmin;  // minimun value Interquiarlie
short CAVmax;  // maximun value Cumulative aceleration vector
short CAVmin;  // minimun value Cumulative aceleration vector
long ACNmax;  // maximun value ?
short ACNmin;  // minimun value ?
short ZCmax;   // maximun value Zero crossing
short ZCmin;   // minimun value Zero crossing
short RSL;     //
short IQR;    // Interquiarlie value
short ZC;     // each axis media Zero crossing porcentaje entre 0-100
long CAV;    // Cumulative Acceleration vector value
short RSL_prev;     //
short IQR_prev;    // Interquiarlie value
short ZC_prev;     // each axis media Zero crossing porcentaje entre 0-100
long CAV_prev;    // Cumulative Acceleration vector value
short RSLref;     //
short IQRref;    // Interquiarlie value reference
short ZCref;     // each axis Zero crossing sum reference
long CAVref;    // Cumulative Acceleration vector reference
short ACNref;    // Cumulative Acceleration vector reference

/* AP settings */
//char ssid[] = _SSID;  //  your network SSID (name)
//char pass[] = _NETPASS;       // your network password

/* NTP parameters and NTP Servers: */
//static const char ntpServerName[] = "us.pool.ntp.org";
static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

/* geolocation variables */
//const char* host = "freegeoip.net";        // host where the ip is get it
const char* host = "ip-api.com";
char latitude[32];                         // latitude value from host
char longitude[32];                        // longitude value from host
const unsigned long HTTP_TIMEOUT = 10000;  // max respone time from server
const size_t MAX_CONTENT_SIZE = 512;       // max size of the HTTP response
const char* googleApiKey = "AIzaSyDrRpRVLNpwu9GsMMB4XyAbE8JrNLG9d98";
WifiLocation location(googleApiKey);
location_t loc;
WiFiManager wifiManager;    // for wifi ssid and password configuration

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WifiPin, OUTPUT);
  pinMode(AccPin, OUTPUT);
  digitalWrite(AccPin, LOW);
  digitalWrite(WifiPin, LOW);

  Serial.begin(500000);
  // Wire.begin(int sda, int scl)
  Wire.begin(4, 5);       // join i2c bus (address optional for master)
  while (!Serial) ; // Needed for Leonardo only
  delay(250);

  //first parameter is name of access point, second is the password
  wifiManager.autoConnect();

  Serial.println("Prosismic");
  Serial.print("Connecting to ");
  //  Serial.println(ssid);
  //  WiFi.begin(ssid, pass);

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
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  Serial.print("Current time: ");
  digitalClockDisplay();
  /* geolocation config */
  loc = location.getGeoFromWiFi();
  dtostrf(loc.lat, 7 , 5, latitude); // Leave room for too large numbers!
  dtostrf(loc.lon, 7 , 5, longitude); // Leave room for too large numbers!
  Serial.println("Location request data");
  Serial.println(location.getSurroundingWiFiJson());
  Serial.print("Geolocation ");
  Serial.print(latitude);
  Serial.print(" ");
  Serial.print(longitude);
  Serial.print(" ");
  Serial.println("Accuracy: " + String(loc.accuracy));

  /* acclereometer setup , see library for details */
  while (!accelerometer.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(5000);
  }
  accelerometer.useInterrupt(ADXL345_INT1);
  accelerometer.setRange(ADXL345_RANGE_2G);
  accelerometer.setDataRate(ADXL345_DATARATE_400HZ);
  Serial.print("ADXL345 setup complete !");

  //  pinMode(DRINTPin, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(DRINTPin), DRAcc_ISR , RISING);

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
  RSLref = 32767;
  IQRref = 50000.00;
  ZCref = 32767;
  CAVref = 32767;
  ACNref = 32767;
  IQRmax = 0.0;
  IQRmin = 0.0;
  CAVmax = 0;
  CAVmin = 0;
  ACNmax = 0;
  ACNmin = 0;
  ZCmax = 0;
  ZCmin = 0;
  sample = 0;
  prevsample = 0;
  psample = 0;
  eventsample = 0 ; // samples after first event detected
  t_offsetlap = now();
  sendTime = millis();
  wifiLap = now();
  ntpmicro = millis();
  startTime = millis();
  DATASEND = false;
  ISCALC = false;
  EVENT = false;
  /* after 3 seconds calculate the offset */
  delay(3000);
  offset(32);  // 50 samples for the offset media
  Serial.println();
  /* start taking samples */
  digitalWrite(AccPin, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
}
void loop()
{

  if (!accSampling(sample)) {
    calcParam(sample);

    if (!DATASEND && (millis() - startTime) <= TST) {
      if (CalcRef(psample, ISCALC)) {
        DATASEND = true; //enable seism event
      } else {
        psample++;// parameter calculator counter
      }
    }

    /* count samples for restore event detector */
    if (EVENT) {
      Serial.print("  Pause even for ");
      Serial.print(SAMPLES / 2 - eventsample);
      Serial.print(" samples");
      eventsample++;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    Serial.println();
    sample++; // window sample counter
  }

  /* check if event count complete */
  if (eventsample == SAMPLES / 2) {
    eventsample = 0;
    DATASEND = true;
    EVENT = false;
    digitalWrite(LED_BUILTIN, LOW);
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
  if (DATASEND) {
    //if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
    if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref)) {
      if ((IQR > IQRref && ZC < ZCref && CAV > CAVref)) trigger = 2;
      if (RSL >= RSLref)  trigger = 1;
      digitalWrite(LED_BUILTIN, HIGH);
      digitalClockDisplay();
      sendTime = millis();
      if (!sendPost()) {
        DATASEND = false;
        EVENT = true;
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(AccPin, HIGH);
      }
      else {
        Serial.print("cannot send event");
        digitalWrite(AccPin, LOW);
        digitalWrite(LED_BUILTIN, LOW);
      }
      Serial.println();
    }
  }

  /* offset  recalcs every TRNM minutes*/
  if ((now() - t_offsetlap) >= (SECS_PER_HOUR * TRNM / 12)) {
    Serial.print("Offset recalculation");
    t_offsetlap = now();
    offset(50);
  }
  
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
  // if (compass.isDataReady()) {
  sca = accelerometer.readmg();
  /* read values at sampling rate */
  if (millis() - tiempo >= INTERVALO && _sample < SAMPLES) {
    tiempo = millis();
    digitalClockDisplay();
    acc_x[sample] = 0.5 * (sca.XAxis - offset_x) + 0.5 * old_x;
    acc_y[sample] = 0.5 * (sca.YAxis - offset_y) + 0.5 * old_y;
    acc_z[sample] = 0.5 * (sca.ZAxis - offset_z) + 0.5 * old_z;
    old_x = acc_x[_sample];
    old_y = acc_y[_sample];
    old_z = acc_z[_sample];

    float aax = acc_x[_sample] * acc_x[_sample];
    float aay = acc_y[_sample] * acc_y[_sample];
    float aaz = acc_z[_sample] * acc_z[_sample];

    AccNetnow[_sample] = sqrt(aax + aay + aaz) * 100.0; // net aceleration with 2 decimals integer
    char line[30];
    snprintf(line, sizeof(line), "%d %d %d %d %d  ", sample, acc_x[_sample], acc_y[_sample], acc_z[_sample], AccNetnow[_sample] );
    Serial.print(line);
    return 0;
  }
  //}
  return 1;
}

/* Calcs CAV , RSL , IQR and ZC parameters and returns time
   of the operation in milliseconds */
time_t calcParam(int _sample) {
  float sumshort = 0;
  float sumlong = 0;
  float cavlong = 0;
  float cavshort = 0;
  short _smpl = 25;
  bool xcurrent;
  bool xprevious;
  bool ycurrent;
  bool yprevious;
  bool zcurrent;
  bool zprevious;
  float xzcr = 0;
  float yzcr = 0;
  float zzcr = 0;
  int temp[SAMPLES];
  time_t startcalc;
  short acc_xabs;
  short acc_yabs;
  short acc_zabs;


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
    if (i >= SAMPLES - _smpl) {
      sumshort += AccNetnow[i - 1];
    }

    /* ZC sum */
    xcurrent = (acc_x[i] > 0);
    xprevious = (acc_x[i - 1] > 0);
    ycurrent = (acc_y[i] > 0);
    yprevious = (acc_y[i - 1] > 0);
    zcurrent = (acc_z[i] > 0);
    zprevious = (acc_z[i - 1] > 0);

    // if the sign is different
    if (xcurrent != xprevious)
    {
      // add one to the zero crossing rate
      xzcr = xzcr + 1;
    }
    if (ycurrent != yprevious)
    {
      // add one to the zero crossing rate
      yzcr = yzcr + 1;
    }
    if (zcurrent != zprevious)
    {
      // add one to the zero crossing rate
      zzcr = zzcr + 1;
    }
  }
  cavlong = (sumlong / SAMPLES) ;
  cavshort = (sumshort / _smpl) ;
  CAV = cavlong;
  ZC = (xzcr + yzcr + zzcr) * (100.0 / (SAMPLES * 3));
  RSL = (cavshort / cavlong) * 100.0; //
  sortArray(temp, SAMPLES);
  IQR = (temp[SAMPLES / 4 * 3 - 1] - temp[SAMPLES / 4 - 1]);

  char line[160];
  snprintf(line, sizeof(line), "%lu %d %d %d %d %lu %d %d %d %d ", (millis() - startcalc), acc_xMax, acc_yMax , acc_zMax, ACNmax , amaxm, ZC, IQR, CAV, RSL);
  Serial.print(line);
  snprintf(line, sizeof(line), " %d %d %d %d ", ZCref, IQRref, CAVref, RSLref);
  Serial.print(line);
  return (millis() - startcalc);
}

int CalcRef(int _sample, bool _iscalc) {

  if (_iscalc) {
    if (_sample < SAMPLES * 2 ) {

      if (_sample >= SAMPLES) {
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
        IQRref = (IQRmax - IQRmin) * Factor_IQR;
        CAVref = (CAVmax - CAVmin) * Factor_CAV;
        ACNref = (ACNmax - ACNmin) * Factor_ACN;
        ZCref =  float(ZCmax) - float(ZCmin * Factor_ZC);
        RSLref = 250;
        char line[160];
        snprintf(line, sizeof(line), "Maxmin var %d %d %d %d %d %d %d %d %d ", ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, ZCref, IQRref, CAVref);
        Serial.print(line);

      } else {
        IQRmax = IQR;
        IQRmin = IQR;
        ZCmax = ZC;
        ZCmin = ZC;
        CAVmax = CAV;
        CAVmin = CAV;
        char line[160];
        snprintf(line, sizeof(line), "Maxmin fijo %d %d %d %d %d %d %d %d %d", ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, ZCref, IQRref, CAVref);
        Serial.print(line);
      }


    } else { // end _sample < SAMPLES * 2  condition

      return 1;
    }

  } else { // if _iscalc false use fixed reference
    ACNref = (ACNmax - ACNmin) * Factor_ZC;
    IQRref = 500;  // [mg] integer
    CAVref = 1000; // [mg] integer
    RSLref = 250;  // percent para medir la ligua
    ZCref =  22;
    char line[160];
    snprintf(line, sizeof(line), "Param fijo %d %d %d %d ", ZCref, IQRref, CAVref, RSLref);
    Serial.print(line);
    DATASEND = true; //enable seism event
    return 1;

  }

  return 0;
}

void MaxMinRef() {

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

  char line[30];
  snprintf(line, sizeof(line), " %d %d %d %d \n", ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin);
  digitalClockDisplay();
  Serial.print(line);
}

void InitRef() {
  IQRmax = IQR;
  IQRmin = IQR;
  ZCmax = ZC;
  ZCmin = ZC;
  CAVmax = CAV;
  CAVmin = CAV;
  char line[30];
  snprintf(line, sizeof(line), " %d %d %d %d \n", ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin);
  digitalClockDisplay();
  Serial.print(line);
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
    sca = accelerometer.readmg();
    if (millis() - tiempo >= INTERVALO) {
      tiempo = millis();
      offset_x += sca.XAxis;
      offset_y += sca.YAxis;
      offset_z += sca.ZAxis;
      j++;
    }
  }
  offset_x = offset_x / samples;
  offset_y = offset_y / samples;
  offset_z = offset_z / samples;
  char line[30];
  snprintf(line, sizeof(line), "Xoffset %l Yoffset %l zoffset %l", offset_x, offset_y, offset_z);
  Serial.print(line);  
}

/* Ip geolotacion */
void getGeo() {

  Serial.print("connecting to ");
  Serial.println(host);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;

  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }

  // We now create a URI for the request
  String url = "/json/";
  // url += streamId;
  //  url += "?private_key=";
  //  url += privateKey;
  //  url += "&value=";
  //  url += value;

  Serial.print("Requesting URL: ");
  Serial.println(url);
  Serial.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();

  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // Skip HTTP headers so that we are at the beginning of the response's body
  // HTTP headers end with an empty line
  char endOfHeaders[] = "\r\n\r\n";

  client.setTimeout(HTTP_TIMEOUT);
  bool ok = client.find(endOfHeaders);

  if (!ok) {
    Serial.println("No response or invalid response!");
  }

  // Compute optimal size of the JSON buffer according to what we need to parse.
  // This is only required if you use StaticJsonBuffer.
  const size_t BUFFER_SIZE =
    JSON_OBJECT_SIZE(8)    // the root object has 8 elements
    + JSON_OBJECT_SIZE(5)  // the "address" object has 5 elements
    + JSON_OBJECT_SIZE(2)  // the "geo" object has 2 elements
    + JSON_OBJECT_SIZE(3)  // the "company" object has 3 elements
    + MAX_CONTENT_SIZE;    // additional space for strings

  // Allocate a temporary memory pool
  DynamicJsonBuffer jsonBuffer(BUFFER_SIZE);

  JsonObject& root = jsonBuffer.parseObject(client);

  if (!root.success()) {
    Serial.println("JSON parsing failed!");
    return;
  }

  // Here were copy the strings we're interested in
  strcpy(latitude, root["lat"]);
  strcpy(longitude, root["lon"]);
  // It's not mandatory to make a copy, you could just use the pointers
  // Since, they are pointing inside the "content" buffer, so you need to make
  // sure it's still in memory when you read the string

  Serial.print("Longitude = ");
  Serial.println(longitude);
  Serial.print("Latitude = ");
  Serial.println(latitude);
  Serial.println("Disconnect");
  client.stop();
}

int sendPost() {

  WiFiClient client; // Use WiFiClient class to create TCP connections
  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    http.begin("http://prosismic.zeke.cl/registrarEvento"); //HTTP
    http.addHeader("Content-Type", "text/plain"); // we will just send a simple string in the body.
    char line[160];
    snprintf(line, sizeof(line), "%d;%lu;%lu;%d;%lu;%d;%d;%d;%d;%d;%d;%s;%s",
             ID, now(), (millis() - ntpmicro) % 1000, ACNmax, trigger, acc_xMax, acc_yMax, acc_zMax,
             ZC, IQR, CAV, latitude, longitude);
    Serial.print("Event ");
    Serial.print(line);
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

  Serial.print(sca.XAxis);
  Serial.print("   ");
  Serial.print(sca.YAxis);
  Serial.print("   ");
  Serial.print(sca.ZAxis);
  Serial.print("   ");
}

void print_ref_raw() {

  Serial.print(IQRref);
  Serial.print(" ");
  Serial.print(CAVref);
  Serial.print(" ");
  Serial.print(ZCref);
  Serial.print(" ");
  Serial.print(ACNref);
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
  Serial.print((_millis - ntpmicro) % 1000);
  Serial.print("   ");
}


/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

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
