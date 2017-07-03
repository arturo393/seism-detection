#include <Wire.h>
#include <LSM303.h>
#include <ADXL345.h>
#include <SparkFun_MMA8452Q.h>
#include <ArduinoSort.h>
#include "SparkTime.h"

/*

  ID 1 = "Pato"
  ID 2 = "Isi"
  ID 3 = "Chrysalis"
  ID 4 = "Zeke I+D"
  ID 5 = "Zeke Stgo"
*/

#define SAMPLES 200           // samples number
#define N_WIN 2               // Windwos
#define ID 4                  // device number identification
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
#define _SSID "zekefi-interno"  // network name
#define _NETPASS "JtXDF5jK79es" // network password
#define OFFSET true              // set median dc offset remove
#define REF_VAR  false          // indicate if reference are variables or fixed values

#define LSMACC   1          // LSM303 acceleromter ID
#define MMAACC   2          // MMA8452Q acceleromter ID
#define ADXLACC  3          // ADXL345 acceleromter ID
byte acc_id;                // accelerometer identification

int WifiPin = D5;     // wifi status LED
int AccPin = D6;      // accelerometer data readings LED

/* Sampling freq */
#define FrecRPS  100                   // Tasa de muestreo definida por el usuario (100 a 200)
unsigned int INTERVALO = (1000 / FrecRPS); // s

/* time variables */
UDP UDPClient;
SparkTime rtc;

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
bool EVENT;             // event detection on/off
bool PARAMCALC; // parameter calculation event
int eventsample = 0 ; // samples after first event detected
bool DATASEND;             // enable/disable send event to server
int sample;                // sample index
int prevsample;
LSM303 compass;                     // acelerometer objet
MMA8452Q accel;
ADXL345 accelerometer;
acc sca;
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
char ssid[] = _SSID;  //  your network SSID (name)
char pass[] = _NETPASS;       // your network password

/* NTP parameters and NTP Servers: */
//static const char ntpServerName[] = "us.pool.ntp.org";
static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

unsigned int localPort = 8888;  // local port to listen for UDP packets
//QuickStats stats; //initialize an instance of this class

/* geolocation variables */
//const char* host = "freegeoip.net";        // host where the ip is get it
const char* host = "ip-api.com";
const char* googleApiKey = "AIzaSyDrRpRVLNpwu9GsMMB4XyAbE8JrNLG9d98";
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
  // Wire.begin(4, 5);       // join i2c bus (address optional for master)
  Wire.begin();       // join i2c bus (address optional for master)
  while (!Serial) ; // Needed for Leonardo only
  delay(250);

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
        Serial.println("LSM303DLH setup complete!");
      }
      break;
    case MMAACC:
      {
        Serial.println("MMA8452Q setup complete!");
      }
      break;
    case ADXLACC:
      {
        accelerometer.useInterrupt(ADXL345_INT1);
        accelerometer.setRange(ADXL345_RANGE_2G);
        accelerometer.setDataRate(ADXL345_DATARATE_400HZ);
        Serial.println("ADXL345 setup complete !");
      }
      break;
    default:
      {
        Serial.println("MATRIX ERROR");
      }
      break;
  }
 /*
   Serial.println("Setting NTP Time");
   rtc.begin(&UDPClient, "north-america.pool.ntp.org");
   Serial.print("Current Time ");
   digitalClockDisplay();
*/
Serial.println("Setting Time and timers");
Serial.println(Time.now()); // 1400647897
Serial.println(Time.timeStr()); // Wed May 21 01:08:47 2014
t_offsetlap = Time.now();
sendTime = millis();
ntpmicro = millis();
startTime = millis();

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

  DATASEND = false;
  ISCALC = false;
  EVENT = false;
  PARAMCALC = true;




  /* after 3 seconds calculate the offset */
  delay(3000);
  offset(50);  // 50 samples for the offset media
  Serial.println();



  /* start taking samples */
  digitalWrite(AccPin, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

}



void loop()
{
  if (!accSampling(sample)) {
    calcParam(sample);
    trigger = 0;
    if (!DATASEND && ((millis() - startTime) <= TST)) {
      if (CalcRef(psample, ISCALC)) {
        DATASEND = true; //enable seism event
        //  PARAMCALC = false;
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

  /* restore sampling window */
  if (sample == SAMPLES)
    sample = 0;



  /* check if event count complete */
  if (eventsample == SAMPLES / 2) {
    eventsample = 0;
    DATASEND = true;
    EVENT = false;
    digitalWrite(LED_BUILTIN, LOW);
  }




  /* sync the milliseconds with ntp time */

    if (Time.now() != prevtime) { //update the display only if time has changed
      prevtime = Time.now();
      ntpmicro = millis();
    }




  /* Sesim event detect and check every 1 TASD secs after first detection */
  if (DATASEND) {
    trigger = 0;
    if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
      if ((IQR > IQRref && ZC < ZCref && CAV > CAVref)) trigger = 2;
      if (RSL >= RSLref)  trigger = 1;
      digitalWrite(LED_BUILTIN, HIGH);
      digitalClockDisplay();
      sendTime = millis();
      if (!sendPost(Time.now())) {
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




  /* offset recalcs every TRNM minutes*/
  if ((Time.now() - t_offsetlap) >= (SECS_PER_HOUR * TRNM / 12)) {
    Serial.print("Offset recalculation");
    t_offsetlap = Time.now();
    offset(50);
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
    snprintf(line, sizeof(line), "%d %d %d %d %d  ", sample, acc_x[_sample], acc_y[_sample], acc_z[_sample], AccNetnow[_sample] );
    Serial.print(line);
    return 0;
  }
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

  short xc;
  short xp;


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
    xc = getSign(acc_x[i]);
    xp = getSign(acc_y[i-1]);



    if ((acc_x[i] * acc_x[i - 1]) < 0)
      xzcr = xzcr + 1;
    if ((acc_y[i] * acc_y[i - 1]) < 0)
      yzcr = yzcr + 1;
    if ((acc_z[i] * acc_z[i - 1]) < 0)
      zzcr = zzcr + 1;


    /* ZC sum
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
      xzcr =  + 1;
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
    */
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
  return (millis() - startcalc);
}
/* get sign of number */
byte getSign(short data)
{
  if(data>0)      /* positif data */
    return (1);
  if(data<0)            /* negatif data */
    return (0);
   else
   return(2);  /*zero data */
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
        RSLref = 300;
        char line[160];
        snprintf(line, sizeof(line), "Variable %d %d %d %d %d %d %d %d %d ", ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, ZCref, IQRref, CAVref);
        Serial.print(line);

      } else {
        IQRmax = IQR;
        IQRmin = IQR;
        ZCmax = ZC;
        ZCmin = ZC;
        CAVmax = CAV;
        CAVmin = CAV;
        char line[160];
        snprintf(line, sizeof(line), "Fijo %d %d %d %d %d %d %d %d %d", ZCmax, ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, ZCref, IQRref, CAVref);
        Serial.print(line);
      }


    } else { // end _sample < SAMPLES * 2  condition

      return 1;
    }

  } else { // if _iscalc false use fixed reference
    ACNref = (ACNmax - ACNmin) * Factor_ZC;
    IQRref = 500;  // [mg] integer
    CAVref = 1000; // [mg] integer
    RSLref = 300;  // percent para medir la ligua
    ZCref =  22;
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
  char line[30];
  snprintf(line, sizeof(line), "Xoffset %d Yoffset %d zoffset %d", offset_x, offset_y, offset_z);
  Serial.print(line);

}


int sendPost(time_t _time) {

    char line[160];
    snprintf(line, sizeof(line), "%d;%lu;%lu;%d;%lu;%d;%d;%d;%d;%d;%d;%s;%s",
             ID, _time, (millis() - ntpmicro) % 1000, ACNmax, trigger, acc_xMax, acc_yMax, acc_zMax,
             ZC, IQR, CAV, latitude, longitude);

             Serial.println(line);
return 0;
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
  Serial.print(ACNref);
}
void print_mag(int _sample) {
  Serial.print(AccNetnow[_sample]);
}

void digitalClockDisplay()
{
  Serial.println(Time.timeStr()); // Wed May 21 01:08:47 201
  Serial.pint(".")
  printmillisntp(millis());
  Serial.print("   ");
}

void printmillisntp(time_t _millis) {
  Serial.print((_millis - ntpmicro) % 1000);
  Serial.print("   ");
}
