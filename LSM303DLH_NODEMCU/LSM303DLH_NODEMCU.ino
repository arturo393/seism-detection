#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LSM303.h>
#include <Filters.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoSort.h>
#include <ArduinoJson.h>

#define SAMPLES 200           // samples number
#define N_WIN 2               // Windwos 
#define ID 4                  // device number identification
#define TRNM 86400UL             // Parameter in between windwos time in min
#define TRST 10000UL                // Start parameter windwos in secs
#define TRNS 40               // Sample windows time in millisecs
#define TASD 1000               // time after send data in msecs
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

/* one pole filter variables */
float testFrequency = 1;                     // test signal frequency (Hz)
float testAmplitude = 100;                   // test signal amplitude
float testOffset = 100;
FilterOnePole filterOneHighpassX;
FilterOnePole filterOneHighpassY;
FilterOnePole filterOneHighpassZ;

/* Sampling freq */
#define FrecRPS  100                   // Tasa de muestreo definida por el usuario (100 a 200)
unsigned int INTERVALO = (1000 / FrecRPS); // s

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
long CAVref;    // Cumulative Acceleration vector reference
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
const int timeZone = -4;
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
//QuickStats stats; //initialize an instance of this class

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

  Serial.begin(500000);
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
  RSLref = 32767;
  IQRref = 50000.00;    // Interquiarlie value reference
  ZCref = 32767;     // each axis Zero crossing sum reference
  CAVref = 32767;    // Cumulative Acceleration vector reference
  ACNref = 32767;
  IQRmax = 0.0;
  IQRmin = 0.0;
  CAVmax = 0;
  CAVmin = 0;
  ACNmax = 0;
  ACNmin = 0;
  ZCmax = 0;
  ZCmin = 0;
  delay(3000);
  if (OFFSET) {
    offset(50);
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
  startTime = millis();
  sendTime = millis();
  startSampling = millis();
  wifiLap = now();
  digitalWrite(AccPin, LOW);
}

void loop()
{
  compass.read();
  if (!OFFSET) {
    filterOneHighpassX.input(compass.a.x / 16);
    filterOneHighpassY.input(compass.a.y / 16);
    filterOneHighpassZ.input(compass.a.z / 16);
  }

  /* read values at sampling rate */
  if (millis() - tiempo >= INTERVALO) {

    if (sample == SAMPLES) {
      Serial.print(millis() - startSampling);
      Serial.print(" ");
      digitalClockDisplay();
      Serial.println();
      startSampling = millis();
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
      digitalWrite(LED_BUILTIN, LOW);
    }

    tiempo = millis();
    if (OFFSET) {
      acc_x[sample] = 0.5 * (compass.a.x / 16 - offset_x) + 0.5 * old_x;
      acc_y[sample] = 0.5 * (compass.a.y / 16 - offset_y) + 0.5 * old_y;
      acc_z[sample] = 0.5 * (compass.a.z / 16 - offset_z) + 0.5 * old_z;
    } else {
      acc_x[sample] = 0.5 * (filterOneHighpassX.output()) + 0.5 * old_x;
      acc_y[sample] = 0.5 * (filterOneHighpassY.output()) + 0.5 * old_y;
      acc_z[sample] = 0.5 * (filterOneHighpassZ.output()) + 0.5 * old_z;
    }
    /* aceleracion neta (guardar decimales ) */
    AccNetnow[sample] = sqrt(acc_x[sample] * acc_x[sample] + acc_y[sample] * acc_y[sample] + acc_z[sample] * acc_z[sample]) * 100;

    MaxMinAcc();
    calcParam();
    MaxMinParam();

    old_x = acc_x[sample];
    old_y = acc_y[sample];
    old_z = acc_z[sample];
    /* Sampling end */
    print_all();
    sample++;
    digitalWrite(LED_BUILTIN, LOW);
  }
  /* Sesim event detect and check every 1 TASD secs after first detection*/
  if (millis() - sendTime >= TASD) {
    if ( (IQR > IQRref && ZC > ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalClockDisplay();
      sendTime = millis();
      sendPost(ID, now(), ACNmax / 100, mag_t, acc_xMax , acc_yMax, acc_zMax, ZC, IQR, CAV);
    }
  }
  /* paramater calcs at first TRST seconds */
  if ((millis() - startTime) == TRST) {
    calc_ref();
    print_ref();
    digitalClockDisplay();
    digitalWrite(AccPin, HIGH);
  }
  /* parameter calcs every TRNM minutes and restore offset values */
  if ((now() - parameterLap) == SECS_PER_MIN * TRNM) {
    parameterLap = now();
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
}
/* finds max aceleration values */
void MaxMinAcc() {

  if ( sample == 0) {
    acc_xMax = abs(acc_x[sample]);
    acc_yMax = abs(acc_y[sample]);
    acc_zMax = abs(acc_z[sample]);
    ACNmax = AccNetnow[sample];
  }

  short acc_xabs = abs(acc_x[sample]);
  short acc_yabs = abs(acc_y[sample]);
  short acc_zabs = abs(acc_z[sample]);

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
}

/* Calcs CAV , RSL , IQR and ZC parameters */
void calcParam() {
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
  /* Parameter cals */
  for (int i = 1; i < SAMPLES; i++)
  {
    temp[i - 1] = AccNetnow[i - 1];
    sumlong += AccNetnow[i - 1];
    if (i >= SAMPLES - _smpl) {
      sumshort += AccNetnow[i - 1];
    }
    // initialise two booleans indicating whether or not
    // the current and previous sample are positive
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
}

void MaxMinParam() {
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
    ZCmax = ZC;
  }
  if (CAV <= CAVmin) {
    CAVmin = CAV;
  }
}

//funcion para dividir el array y hacer los intercambios
int dividir(int *array, int inicio, int fin)
{
  int izq;
  int der;
  int pibote;
  int temp;

  pibote = array[inicio];
  izq = inicio;
  der = fin;

  //Mientras no se cruzen los índices
  while (izq < der) {
    while (array[der] > pibote) {
      der--;
    }

    while ((izq < der) && (array[izq] <= pibote)) {
      izq++;
    }

    // Si todavia no se cruzan los indices seguimos intercambiando
    if (izq < der) {
      temp = array[izq];
      array[izq] = array[der];
      array[der] = temp;
    }
  }

  //Los indices ya se han cruzado, ponemos el pivote en el lugar que le corresponde
  temp = array[der];
  array[der] = array[inicio];
  array[inicio] = temp;

  //La nueva posición del pivote
  return der;
}

//            Funcion Quicksort
//======================================================================
//funcion recursiva para hacer el ordenamiento
void quicksort(int *array, int inicio, int fin)
{
  int pivote;
  if (inicio < fin)
  {
    pivote = dividir(array, inicio, fin );
    quicksort( array, inicio, pivote - 1 );//ordeno la lista de los menores
    quicksort( array, pivote + 1, fin );//ordeno la lista de los mayores
  }
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
void calc_ref() {
  if (REF_VAR) {
    IQRref = (IQRmax - IQRmin) * Factor_IQR;
    CAVref = (CAVmax - CAVmin) * Factor_CAV;
    ACNref = (ACNmax - ACNmin) * Factor_ACN;
    ZCref =  float(ZCmax) - float(ZCmin * Factor_ZC);
    RSLref = 145;
  } else {
    ACNref = (ACNmax - ACNmin) * Factor_ZC;
    IQRref = 5000;
    CAVref = 10000;
    RSLref = 145;
    ZCref =  float(ZCmax) - float(ZCmin * Factor_ZC);
  }
}

void offset(int samples) {
  int j = 0;
  while (j < samples) {
    compass.read();
    if (millis() - tiempo >= INTERVALO) {
      tiempo = millis();
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
    http.addHeader("Content-Type", "text/plain"); // we will just send a simple string in the body.
    char line[160];
    snprintf(line, sizeof(line), "%d;%lu;%d;%d;%d;%d;%d;%d;%d;%d;%s;%s", _id, _tiempo, _amax, _tamax, _ax, _ay, _az, _zc, _iq, _cav, longitude, latitude);
    int httpCode = http.POST(line);
    Serial.print(line);
    String payload = http.getString();
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
void print_all(){
   //   print_acc_values();
   //   Serial.print("\t");
      print_parameter();
      Serial.print("\t");
      print_ref();
      Serial.println();
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

void print_parameter_raw() {

  Serial.print(IQR);
  Serial.print(" ");
  Serial.print(CAV);
  Serial.print(" ");
  Serial.print(ZC);
  Serial.print(" ");
  Serial.print(ACNmax);

}
void print_acc_values() {
  Serial.print("x-axis");
  Serial.print("   ");
  Serial.print(acc_x[sample]);
  Serial.print("   ");
  Serial.print("y-axis");
  Serial.print("   ");
  Serial.print(acc_y[sample]);
  Serial.print("   ");
  Serial.print("z-axis");
  Serial.print("   ");
  Serial.print(acc_z[sample]);
  Serial.print("   ");
  Serial.print("NA");
  Serial.print("    ");
  Serial.print(AccNetnow[sample]);
}
void print_acc_values_raw() {

  Serial.print(acc_x[sample]);
  Serial.print(" ");
  Serial.print(acc_y[sample]);
  Serial.print(" ");
  Serial.print(acc_z[sample]);
  Serial.print(" ");
  Serial.print(AccNetnow[sample]);
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
  Serial.print("ACNref");
  Serial.print("  ");
  Serial.print(ACNref);
  Serial.print("  ");
  Serial.print("RSLref");
  Serial.print("  ");
  Serial.print(RSLref);
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
