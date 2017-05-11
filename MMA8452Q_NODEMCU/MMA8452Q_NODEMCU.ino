#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#include <ESP8266HTTPClient.h>


#define SAMPLES 200  // samples number
#define N_WIN 2      // Windwos 
#define ID  1  // #1 device number
#define OFFSET_DIGITAL false

//int DRINTPin = 4;   // accelerometer data ready interrupt
//int LEDPin = LED_BUILTIN;   // server data ok

/* Sampling freq */
const int FrecRPS = 100; // Tasa de muestreo definida por el usuario (100 a 200)
// numer of windows for sampling
unsigned int INTERVALO = (1000 / FrecRPS); // ms
volatile unsigned long tiempo;
volatile int sample;   // accerelometer sample index

/* accelerometer parameters and variables  */
MMA8452Q accel;
int amp = 1000;           // set data to mg
volatile unsigned long offset_x, offset_y, offset_z; // offset data for normalization
volatile short old_x, old_y, old_z;  // old acceleration data
short ACC_sample[3];      // [0] = x_acc [1] = y_acc [z] = z_acc
short AccNetnow[SAMPLES]; // net acceleration
short acc_x[SAMPLES];     // x-axis acceleration array
short acc_y[SAMPLES];     // x-axis acceleration array
short acc_z[SAMPLES];     // x-axis acceleration array
short acc_xMax;           // x-axis max acceleration
short acc_yMax;           // y-axis max acceleration
short acc_zMax;           // z-axis max acceleration
short AccNetnowMax;       // max net acceleration
volatile short mag_t;              // sample number of aceleration magnitude
volatile short x_t;                // sample number of aceleration x-axis
volatile short y_t;                // sample number of aceleration y-axis
volatile short z_t;                // sample number of aceleration z-axis

#define TRNM 30   // Parameter in between windwos time in min
#define TRST 40   // Start parameter windwos in secs
#define TRNS 10   // Sample windows time in secs
#define Factor_IQR  4
#define Factor_CAV  7
#define Factor_ACN  3
#define Factor_ZC   0.75
#define StartWindow 20 //

volatile short IQRmax;  // maximun value Interquiarlie
volatile short IQRmin;  // minimun value Interquiarlie
volatile short CAVmax;  // maximun value Cumulative aceleration vector
volatile short CAVmin;  // minimun value Cumulative aceleration vector
volatile short ACNmax;  // maximun value ?
volatile short ACNmin;  // minimun value ?
volatile short ZCmax;   // maximun value Zero crossing
volatile short ZCmin;   // minimun value Zero crossing
volatile short RSL;     //
volatile short IQR;    // Interquiarlie value
volatile short ZC;     // each axis Zero crossing sum value
volatile short CAV;    // Cumulative Acceleration vector value
volatile short RSLref;     //
volatile short IQRref;    // Interquiarlie value reference
volatile short ZCref;     // each axis Zero crossing sum reference
volatile short CAVref;    // Cumulative Acceleration vector reference
volatile short ACNref;    // Cumulative Acceleration vector reference

volatile int startSampling = 0; // when the digital clock was displayed
volatile time_t eventStart;
time_t windowStart;


time_t startSecond;
time_t parameterLap;
/* AP settings */
char ssid[] = "zekefi-interno";  //  your network SSID (name)
char pass[] = "JtXDF5jK79es";       // your network password

/* NTP parameters and NTP Servers: */
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = -3;
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t acc_time;

void setup()
{
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
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  Serial.print("Current time: ");
  digitalClockDisplay();

  /* acclereometer setup , see library for details */
  while (!accel.setup()) {
    Serial.print("MMA8452Q not setup, check device !");
    delay(5000);
  }
  Serial.print("MMA8452Q setup complete !");
  pinMode(LED_BUILTIN, OUTPUT);
  //  pinMode(DRINTPin, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(DRINTPin), DRAcc_ISR , RISING);

  /* setup init values */

  for (int k = 0; k < SAMPLES ; k++) {
    AccNetnow[k] = 0;
    acc_x[k] = 0;
    acc_y[k] = 0;
    acc_z[k] = 0;
  }
  offset_x = 0;
  offset_y = 0;
  offset_z = 0;
  sample = 0;
  parameterLap = 0;
  startSecond = second();
}


void loop()
{
  if (timeStatus() != timeNotSet) {
    //update the display only if time has changed
    if ((now()) != startSampling && sample == SAMPLES) {
      digitalWrite(LED_BUILTIN, LOW);
      startSampling = now();


      eventStart = millis();




      /*
            Serial.print(AccNetnowMax);
            Serial.print(" ");

            Serial.print(" ");
            Serial.print(acc_xMax);
            Serial.print(" ");

            Serial.print(" ");
            Serial.print(acc_yMax);
            Serial.print(" ");

            Serial.print(" ");
            Serial.print(acc_zMax);

            Serial.print(" ");
            Serial.print(cav);

            Serial.print(" ");
            Serial.print(iqr);

            Serial.print(" ");
            Serial.println(zcr);


            Serial.print("   Event calc time ");
            Serial.print(millis() - eventStart);
            Serial.println(" ms");
      */
      /* All over again */
      sample = 0;
      time_t windowEnd = millis() - windowStart;
      Serial.print("   Widow sampling time ");
      Serial.print(windowEnd);
      Serial.println("   ms ");
      windowStart = millis();

     
    }

    /* paramater calcs every TRNM */
    if ( (minute() - parameterLap) == TRNM || (second() - startSecond) ==  TRST) {
      parameterLap = minute();
      IQRref = (IQRmax - IQRmin) * Factor_IQR;
      CAVref = (CAVmax - CAVmin) * Factor_CAV;
      ACNref = (ACNmax - ACNmin) *Factor_ACN;
      ZCref = (ZCmax - ZCmin)* Factor_ZC;  //float ?
    }


    /* read values at sampling rate */
    if (millis() - tiempo >= INTERVALO) {
      tiempo = millis();
      accel.readRaw();

      acc_x[sample] = 0.5 * (accel.x - offset_x) + 0.5 * old_x;
      acc_y[sample] = 0.5 * (accel.y - offset_y) + 0.5 * old_y;
      acc_z[sample] = 0.5 * (accel.z - offset_z) + 0.5 * old_z;

      AccNetnow[sample] = sqrt(acc_x[sample] * acc_x[sample] + acc_y[sample] * acc_y[sample] + acc_z[sample] * acc_z[sample]);

      if ( sample == 0) {
        acc_xMax = abs(acc_x[sample]);
        acc_yMax = abs(acc_y[sample]);
        acc_zMax = abs(acc_z[sample]);
        AccNetnowMax = AccNetnow[sample];

      }
      short acc_xabs = abs(acc_x[sample]);
      short acc_yabs = abs(acc_y[sample]);
      short acc_zabs = abs(acc_z[sample]);

      /* finds max aceleration values */

      if (AccNetnow[sample]  > ACNmax) {
        ACNmax = AccNetnow[sample];
        acc_time = millis();
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

      
      /*
          Serial.print(now());
          Serial.print(" ");
          Serial.print(sample);
          Serial.print(" ");
          Serial.print(acc_x[sample]);
          Serial.print(" ");
          Serial.print(acc_y[sample]);
          Serial.print(" ");
          Serial.print(acc_z[sample]);
          Serial.print(" ");
          Serial.println(AccNetnow[sample]);
         
      */
      old_x = acc_x[sample];
      old_y = acc_y[sample];
      old_z = acc_z[sample];

      sample++;

    /* Sampling end */

    /* Parameter cals */ 
    
      short cavshort = cummulativeMeasureAmplitud(AccNetnow, SAMPLES, 25); // Mag(acc) short term media
      CAV = cummulativeMeasureAmplitud(AccNetnow, SAMPLES, SAMPLES); // Mag(acc)  long term media
      RSL = (cavshort / 25) / (CAV / SAMPLES); //
      sort(AccNetnow, SAMPLES);
      short Q1 = AccNetnow[SAMPLES / 4 - 1];
      short Q3 = AccNetnow[SAMPLES * 3 / 4 - 1];
      IQR = Q3 - Q1;   // IQR
      ZC = zeroCrossingNum(acc_x, SAMPLES) + zeroCrossingNum(acc_y, SAMPLES) + zeroCrossingNum(acc_z, SAMPLES);

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

     if(IQR > IQRref || ZC < ZCref || CAV > CAVref)
     sendPost(ID, now(), ACNmax, mag_t, acc_xMax , acc_yMax, acc_zMax, ZC, IQR);
  
    }
  }
}

void DRAcc_ISR () {

}



short cummulativeMeasureAmplitud (short _buffer[], int _size, int _id) {
  int b = _size - 1;
  int a = _size - _id;

  short f_x;
  for (int i = b; i >= a; i--) {
    f_x += _buffer[i];
  }
  return f_x;
}

void offset(int samples) {
  int j = 1;
  while (j < samples) {
    if (accel.available()) {
      accel.read();
      offset_x += accel.x;
      offset_y += accel.y;
      offset_z += accel.z;
      j++;
    }
  }
  offset_x = offset_x / samples;
  offset_y = offset_y / samples;
  offset_z = offset_z / samples;

}

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

float zeroCrossingRate (short buffer[], int size)
{
  // create a variable to hold the zero crossing rate
  float zcr = 0;

  // for each audio sample, starting from the second one
  for (int i = 1; i < size; i++)
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
  return zcr / size;
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

float shortTofloat(short _value) {
  return ((float) _value / (float)(1 << 11) * (float)(2));
}

int sendPost(byte _id, time_t _tiempo, short _amax, short _tamax, short _ax, short _ay, short _az, short _zc, short _iq) {

  WiFiClient client; // Use WiFiClient class to create TCP connections
  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
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
    snprintf(line, sizeof(line), "%d;%lu;%d;%d;%d;%d;%d;%d;%d", _id, _tiempo, _amax, _tamax, _ax, _ay, _az, _zc, _iq);

    int httpCode = http.POST(line);

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
  Serial.println();
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
