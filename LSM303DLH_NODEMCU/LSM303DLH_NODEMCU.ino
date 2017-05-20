#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LSM303.h>
#include <Filters.h>
#include <ESP8266HTTPClient.h>
#include "QuickStats.h"
#include <ArduinoJson.h>

#define SAMPLES 200           // samples number
#define N_WIN 2               // Windwos 
#define ID 4                  // device number identification
#define TRNM 10UL             // Parameter in between windwos time in min
#define TRST 20                // Start parameter windwos in secs
#define TRNS 40               // Sample windows time in millisecs
#define TASD 20               // time after send data in secs
#define TWC 1                 // time Wifi check
#define Factor_IQR  4.0         // Interquarile reference factor
#define Factor_CAV  7         // Cumulative acelerator vector reference factor
#define Factor_ACN  3.0         // Aceleration magnitude reference factor
#define Factor_ZC   0.25      // Zero Crossing rate reference factor
#define Factor_RSL  7
#define StartWindow 20
#define _SSID "zekefi-interno"  // network name
#define _NETPASS "JtXDF5jK79es" // network password    
#define OFFSET true              // set median dc offset remove
#define REF_VAR  false          // indicate if reference are variables or fixed values

int WifiPin = D7;     // wifi status LED
int AccPin = D6;      // accelerometer data readings LED

typedef float REAL;
typedef short SIGNAL;

typedef struct
{
  int windowSize;
  int sampleRate;
} ZeroCross;


/* one pole filter variables */
float testFrequency = 1;                     // test signal frequency (Hz)
float testAmplitude = 100;                   // test signal amplitude
float testOffset = 100;
FilterOnePole filterOneHighpassX;
FilterOnePole filterOneHighpassY;
FilterOnePole filterOneHighpassZ;

/* Sampling freq */
#define FrecRPS  100                   // Tasa de muestreo definida por el usuario (100 a 200)
unsigned int INTERVALO = (1000 / FrecRPS); // ms

/* time variables */
time_t tiempo;        // ?
time_t startSampling; // when a new simpling windwow start
time_t eventStart;    // when a calculation windwos start
time_t windowStart;   // ?
time_t startTime;     // time after setup() if finished
time_t parameterLap;  // time between reference recalculation
time_t sendTime;      // when an event start
time_t wifiLap;       // counter time for wifi check

/* accelerometer parameters and variables  */
int sample;                // sample index
LSM303 compass;                     // acelerometer objet
long offset_x, offset_y, offset_z; // offset data for normalization
short old_x, old_y, old_z;         // old acceleration data for normalization
short ACC_sample[3];              // [0] = x_acc [1] = y_acc [z] = z_acc
float AccNetnow[SAMPLES]; // net acceleration
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


float IQRmax;  // maximun value Interquiarlie
float IQRmin;  // minimun value Interquiarlie
short CAVmax;  // maximun value Cumulative aceleration vector
short CAVmin;  // minimun value Cumulative aceleration vector
float ACNmax;  // maximun value ?
float ACNmin;  // minimun value ?
short ZCmax;   // maximun value Zero crossing
short ZCmin;   // minimun value Zero crossing
float RSL;     //
float IQR;    // Interquiarlie value
short ZC;     // each axis media Zero crossing porcentaje entre 0-100
short ZC_test;
long CAV;    // Cumulative Acceleration vector value
float RSLref;     //
float IQRref;    // Interquiarlie value reference
short ZCref;     // each axis Zero crossing sum reference
short CAVref;    // Cumulative Acceleration vector reference
float ACNref;    // Cumulative Acceleration vector reference

/* AP settings */
char ssid[] = _SSID;  //  your network SSID (name)
char pass[] = _NETPASS;       // your network password

/* NTP parameters and NTP Servers: */
//static const char ntpServerName[] = "us.pool.ntp.org";
static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";
const int timeZone = -3;
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
QuickStats stats; //initialize an instance of this class

/* geolocation variables */
const char* host = "freegeoip.net";        // host where the ip is get it
char latitude[32];                         // latitude value from host
char longitude[32];                        // longitude value from host
const unsigned long HTTP_TIMEOUT = 10000;  // max respone time from server
const size_t MAX_CONTENT_SIZE = 512;       // max size of the HTTP response
void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WifiPin, OUTPUT);
  pinMode(AccPin, OUTPUT);

  Serial.begin(115200);
  // Wire.begin(int sda, int scl)
  Wire.begin(4, 5);       // join i2c bus (address optional for master)
  while (!Serial) ; // Needed for Leonardo only
  delay(250);
  Serial.println("Prosismic");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

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
  getGeo();


  /* acclereometer setup , see library for details */
  while (!compass.init()) {
    Serial.println("LSM303DLH not setup, check device !");
    delay(5000);
  }
  compass.enableDefault();
  Serial.print("LSM303DLH setup complete !");

  //  pinMode(DRINTPin, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(DRINTPin), DRAcc_ISR , RISING);

  /* setup init values */
  for (int k = 0; k < SAMPLES ; k++) {
    AccNetnow[k] = 0;
    acc_x[k] = 0;
    acc_y[k] = 0;
    acc_z[k] = 0;
  }
  RSLref = 0;
  IQRref = 100.00;    // Interquiarlie value reference
  ZCref = 100;     // each axis Zero crossing sum reference
  CAVref = 200;    // Cumulative Acceleration vector reference
  ACNref = 100;
  IQRmax = 0.0;
  IQRmin = 0.0;
  CAVmax = 0;
  CAVmin = 0;
  ACNmax = 0;
  ACNmin = 0;
  ZCmax = 0;
  ZCmin = 0;

  if (OFFSET) {
    offset(SAMPLES);
  }
  else {

    filterOneHighpassX.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a o:qne pole (RC) highpass filter
    filterOneHighpassY.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a one pole (RC) highpass filter
    filterOneHighpassZ.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a one pole (RC) highpass filter
    offset_x = 0;
    offset_y = 0;
    offset_z = 0;
  }
  sample = 0;

  parameterLap = now();
  startTime = now();
  sendTime = now();
  wifiLap = now();

  filterOneHighpassX.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a o:qne pole (RC) highpass filter
  filterOneHighpassY.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a one pole (RC) highpass filter
  filterOneHighpassZ.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a one pole (RC) highpass filter
  // offset(SAMPLES);
}


void loop()
{

  if ((now()) != startSampling && sample == SAMPLES) {
    digitalWrite(AccPin, LOW);
    startSampling = now();
    eventStart = millis();
    print_ref();
    print_parameter();

    /* All over again */
    sample = 0;
    IQRmax = 0.0;
    IQRmin = 0.0;
    CAVmax = 0;
    CAVmin = 0;
    ACNmax = 0;
    ACNmin = 0;
    ZCmax = 0;
    ZCmin = 0;

    time_t windowEnd = millis() - windowStart;
    /*
      Serial.print("   Widow sampling time ");
      Serial.print(windowEnd);
      Serial.println("   ms ");
    */
    windowStart = millis();
  }


  /* paramater calcs at first TRST seconds */
  if ((now() - startTime) == TRST) {
    calc_ref();
    print_ref();
    digitalClockDisplay();
  }


  /* parameter calcs every TRNM minutes and restore offset values */
  if ((now() - parameterLap) == SECS_PER_MIN * TRNM) {
    parameterLap = now();
    offset(SAMPLES);
    calc_ref();
    print_ref();
    digitalClockDisplay();
  }

  if ((now() - wifiLap) == SECS_PER_MIN * TWC) {
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


  compass.read();

  if (!OFFSET) {
    filterOneHighpassX.input(compass.a.x / 16);
    filterOneHighpassY.input(compass.a.y / 16);
    filterOneHighpassZ.input(compass.a.z / 16);
  }

  /* read values at sampling rate */
  if (millis() - tiempo >= INTERVALO) {
    tiempo = millis();
    digitalWrite(AccPin, HIGH);

    if (OFFSET) {
      acc_x[sample] = 0.5 * (compass.a.x/16 - offset_x) + 0.5 * old_x;
      acc_y[sample] = 0.5 * (compass.a.y/16 - offset_y) + 0.5 * old_y;
      acc_z[sample] = 0.5 * (compass.a.z/16 - offset_z) + 0.5 * old_z;
    } else {
      acc_x[sample] = 0.5 * (filterOneHighpassX.output()) + 0.5 * old_x;
      acc_y[sample] = 0.5 * (filterOneHighpassY.output()) + 0.5 * old_y;
      acc_z[sample] = 0.5 * (filterOneHighpassZ.output()) + 0.5 * old_z;
    }
    /* aceleracion neta (guardar decimales ) */
    AccNetnow[sample] = sqrt(acc_x[sample] * acc_x[sample] + acc_y[sample] * acc_y[sample] + acc_z[sample] * acc_z[sample]);

    if ( sample == 0) {
      acc_xMax = abs(acc_x[sample]);
      acc_yMax = abs(acc_y[sample]);
      acc_zMax = abs(acc_z[sample]);
      ACNmax = AccNetnow[sample];
    }

    short acc_xabs = abs(acc_x[sample]);
    short acc_yabs = abs(acc_y[sample]);
    short acc_zabs = abs(acc_z[sample]);

    /* finds max aceleration values */
    if (AccNetnow[sample]  > ACNmax) {
      ACNmax = AccNetnow[sample];
      mag_t = sample;
    }

    if (AccNetnow[sample] <= ACNmin) {
      ACNmin = AccNetnow[sample];
    }

    if (acc_xabs > acc_xMax) {
      acc_xMax =  acc_xabs;
    }
    if (acc_yabs > acc_yMax) {
      acc_yMax = acc_yabs;
    }
    if (acc_zabs > acc_zMax) {
      acc_zMax = acc_zabs;
    }

    print_acc_values();

    old_x = acc_x[sample];
    old_y = acc_y[sample];
    old_z = acc_z[sample];
    /* Sampling end */

    /* Parameter cals */

    float CAVshort = cummulativeMeasureAmplitud(AccNetnow, SAMPLES, 25) * 1000; // Mag(acc) short term media
    CAV = cummulativeMeasureAmplitud(AccNetnow, SAMPLES, SAMPLES) * 1000; // Mag(acc)  long term media
    RSL = (CAVshort) / (CAV) * 100; // con
    stats.bubbleSort(AccNetnow, SAMPLES);
    float Q1 = AccNetnow[SAMPLES / 4 - 1];
    float Q3 = AccNetnow[SAMPLES / 4 * 3 - 1];
    IQR = (Q3 - Q1) * 1000; // IQR
    ZC = int((zeroCrossingRate(acc_x, SAMPLES) + zeroCrossingRate(acc_y, SAMPLES) + zeroCrossingRate(acc_z, SAMPLES)) * 100 / 3);
    // ZC_test  = int( zeroCross(acc_x, SAMPLES, FrecRPS) *+ zeroCross(acc_y, SAMPLES, FrecRPS) + zeroCross(acc_z, SAMPLES, FrecRPS) *100 /3 );

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
    sample++;

    //  print_ref();
    //  print_results();

    digitalWrite(LED_BUILTIN, LOW);
    //    if ((now() - sendTime >= TASD) && (IQR > IQRref && ZC > ZCref && CAV > CAVref  && RSLref)) {

  }

  if (now() - sendTime >= TASD) {
    if ( (IQR > IQRref && ZC > ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
      digitalClockDisplay();
      sendTime = now();
      sendPost(ID, now(), ACNmax, mag_t, acc_xMax , acc_yMax, acc_zMax, ZC, IQR, CAV);
    }
  }

}

void DRAcc_ISR () {

}

void print_parameter() {
  Serial.print("IQR ");
  Serial.print("\t");
  Serial.print(IQR);
  Serial.print("\t");
  Serial.print("CAV ");
  Serial.print("\t");
  Serial.print(CAV);
  Serial.print("\t");
  Serial.print("ZC");
  Serial.print("\t");
  Serial.print(ZC);
  Serial.print("\t");
  Serial.print("ACNmax ");
  Serial.print("\t");
  Serial.print(ACNmax);
  Serial.print("\t");
  Serial.print("RSL");
  Serial.print("\t");
  Serial.println(RSL);

}

void print_parameter_raw() {

  Serial.print(IQR);
  Serial.print(" ");
  Serial.print(CAV);
  Serial.print(" ");
  Serial.print(ZC);
  Serial.print(" ");
  Serial.println(ACNmax);

}
void print_acc_values() {
  Serial.print("x-axis ");
  Serial.print("\t");
  Serial.print(acc_x[sample]);
  Serial.print("\t");
  Serial.print("y-axis ");
  Serial.print("\t");
  Serial.print(acc_y[sample]);
  Serial.print("\t");
  Serial.print("z-axis ");
  Serial.print("\t");
  Serial.print(acc_z[sample]);
  Serial.print("\t");
  Serial.print("Net acceleration ");
  Serial.print("\t");
  Serial.println(AccNetnow[sample]);
}
void print_acc_values_raw() {

  Serial.print(acc_x[sample]);
  Serial.print(" ");
  Serial.print(acc_y[sample]);
  Serial.print(" ");
  Serial.print(acc_z[sample]);
  Serial.print(" ");
  Serial.println(AccNetnow[sample]);
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

void calc_ref() {
  if ( REF_VAR == true) {
    IQRref = (IQRmax - IQRmin) * Factor_IQR;
    CAVref = (CAVmax - CAVmin) * Factor_CAV;
    ACNref = (ACNmax - ACNmin) * Factor_ACN;
    ZCref = ZCmax - ZCmin * Factor_ACN;
    RSLref = (RSLref - RSLref) * Factor_RSL;
  } else {
    ACNref = (ACNmax - ACNmin) * Factor_ACN;
    IQRref = 5000;
    CAVref = 10000;
    RSLref = 145;
    ZCref = ZCmax - ZCmin * Factor_ACN;;
  }
}
void print_ref_raw() {

  Serial.print(IQRref);
  Serial.print(" ");
  Serial.print(CAVref);
  Serial.print(" ");
  Serial.print(ZCref);
  Serial.print(" ");
  Serial.println(ACNref);
}
void print_ref() {
  Serial.print("IQRref ");
  Serial.print("\t");
  Serial.print(IQRref);
  Serial.print("\t");
  Serial.print("CAVref ");
  Serial.print("\t");
  Serial.print(CAVref);
  Serial.print("\t");
  Serial.print("ZCref ");
  Serial.print("\t");
  Serial.print(ZCref);
  Serial.print("\t");
  Serial.print("ACNref ");
  Serial.print("\t");
  Serial.print(ACNref);
  Serial.print("\t");
  Serial.print("RSLref ");
  Serial.print("\t");
  Serial.println(RSLref);


}

short cummulativeMeasureAmplitud (short _buffer[], int _size, int _id) {
  int b = _size - 1;
  int a = _size - _id;

  short f_x = 0;
  for (int i = b; i < _size; i++) {
    f_x += _buffer[i];
  }
  return f_x;
}

float cummulativeMeasureAmplitud (float _buffer[], int _size, int _id) {
  int b = _size;
  int a = _size - _id;

  float f_x = 0;
  for (int i = a; i < b; i++) {
    f_x += _buffer[i];
  }
  return f_x / float (_size);
}

void offset(int samples) {
  int j = 1;

  while (j < samples) {
    compass.read();
    if (millis() - tiempo >= INTERVALO) {
      
      offset_x += compass.a.x / 16;
      offset_y += compass.a.y / 16;
      offset_z += compass.a.z / 16;
      j++;
    }

  }
  offset_x = offset_x / samples;
  offset_y = offset_y / samples;
  offset_z = offset_z / samples;

}

//------------------------------------------------------------------------------
// Computes the absolute difference between two given samples. Used to determine
// a change in sign in the zero crossing rate estimator.
//------------------------------------------------------------------------------
REAL absdif(REAL x, REAL y)
{
  REAL r = x - y;
  if (r < 0)
  {
    return -1 * r;
  }
  else
  {
    return r;
  }
}
//------------------------------------------------------------------------------
// Indicates the sign of the sample. -1 = -ve, +1 = +ve
//------------------------------------------------------------------------------
REAL sgn( REAL sample )
{
  if ( sample < 0 )
  {
    return -1;
  }
  else
  {
    return 1;
  }
}
//------------------------------------------------------------------------------
// Returns the absolute (positive) value of a given sample.
//------------------------------------------------------------------------------
REAL getabs( REAL val )
{
  if ( val < 0 )
  {
    return -1 * val;
  }
  else
  {
    return val;
  }
}

//------------------------------------------------------------------------------
// Computes the zero crossing rate estimator of a given input buffer. This
// input can be pre-processed with a lowpass filter to increase robustness to
// noise.
//------------------------------------------------------------------------------
REAL zeroCross(SIGNAL* input, int length, int sampleRate)
{
  int i;

  // interpolate crossings
  int firstCross = -1;
  int lastCross = -1;
  float iLeft;
  float iRight;

  int count = 0;
  REAL sgn0 = sgn(input[0]);
  REAL sgn1 = 0;

  for (i = 1; i < length; i++)
  {
    sgn1 = sgn(input[i]);
    if (absdif(sgn0, sgn1) > 1)
    {
      if (firstCross < 0)
      {
        firstCross = i;
        lastCross = firstCross;
        iLeft = (i - 1) + (input[i - 1] / (REAL)(input[i - 1] - input[i]));
      }
      else
      {
        lastCross = i;
      }
      count++;
    }
    sgn0 = sgn1;
  }


  if ( lastCross - firstCross <= 0)
  {
    return 0;
  }
  else
  {
    iRight = (lastCross - 1) + (input[lastCross - 1] / (REAL)(input[lastCross - 1] - input[lastCross]));
    return sampleRate * (count - 1) / ((REAL)2.0 * (iRight - iLeft));
  }
}
/* considerar problema de señal digital */
short zeroCrossingNum (short buffer[], int _size )
{
  // create a variable to hold the zero crossing rate
  short zcr = 0;

  // for each audio sample, starting from the second one
  for (int i = 1; i < _size; i++)
  {
    // initialise two booleans indicating whether or not
    // the current and previous sample are positive
    bool current = (buffer[i] > 0);
    bool previous = (buffer[i - 1] > 0);

    // if the sign is different
    if (current != previous)
    {
      // add one to the zero crossing rate
      zcr = zcr + 1;
    }
  }

  // return the zero crossing rate
  return zcr;
}

float zeroCrossingRate (short buffer[], int _size)
{
  // create a variable to hold the zero crossing rate
  float zcr = 0;

  // for each audio sample, starting from the second one
  for (int i = 1; i < _size; i++)
  {
    // initialise two booleans indicating whether or not
    // the current and previous sample are positive
    bool current = (buffer[i] > 0);
    bool previous = (buffer[i - 1] > 0);

    // if the sign is different
    if (current != previous)
    {
      // add one to the zero crossing rate
      zcr = zcr + 1.0;
    }
  }

  // return the zero crossing rate
  return zcr / _size;
}


void sort(short a[], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
      }
    }
  }
}

void sort(float a[], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        float t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;
      }
    }
  }
}


float shortTofloat(short _value) {
  return ((float) _value / (float)(1 << 11) * (float)(2));
}

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
  strcpy(latitude, root["latitude"]);
  strcpy(longitude, root["longitude"]);
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


int sendPost(byte _id, time_t _tiempo, short _amax, short _tamax, short _ax, short _ay, short _az, short _zc, short _iq, short _cav) {

  WiFiClient client; // Use WiFiClient class to create TCP connections
  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    digitalWrite(WifiPin, HIGH);
    http.begin("http://prosismic.zeke.cl/registrarEvento"); //HTTP
    // http.begin("http://jsonplaceholder.typicode.com/users"); //HTTP
    http.addHeader("Content-Type", "text/plain"); // we will just send a simple string in the body.

    char xbuff[10];  //Cadena donde almacenaremos el número convertido
    char ybuff[10];
    char zbuff[10];
    char abuff[10];
    char zcbuff[10];
    char iqrbuff[10];
    char line[160];

    //  snprintf(line, sizeof(line), "-F 'id=%d' -F 'tiempo=%lu' -F 'amax=%s'  -F 'tamax=%d' -F 'ax=%s' -F 'ay=%s' -F 'az=%s' -F 'zc=%s' -F 'iqr=%s' ", _id, now(), abuff, _tamax, xbuff, ybuff, zbuff, zcbuff, iqrbuff);
    snprintf(line, sizeof(line), "%d;%lu;%d;%d;%d;%d;%d;%d;%d;%d", _id, _tiempo, _amax, _tamax, _ax, _ay, _az, _zc, _iq, _cav,longitude,latitude);

    int httpCode = http.POST(line);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print(line);
    String payload = http.getString();
    if (payload.equals("1"))
      digitalWrite(LED_BUILTIN, HIGH);
    http.end();
    Serial.print("   httpCode  ");
    Serial.print(httpCode);   //Print HTTP return code
    Serial.print("   payload  ");
    Serial.println(payload);    //Print request response payload

    return 0;

  } else {
    digitalWrite(WifiPin, LOW);
    Serial.println("Error in WiFi connection");
    return 1;
  }

}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.print(" ");
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
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
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
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
