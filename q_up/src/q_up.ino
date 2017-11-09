/*
 * Project q_up
 * Description: Programa para la detección temprana de sismos
 * Author: Arturo Veras
 * Date: 31 de octubre del 2017
 */



// This #include statement was automatically added by the Particle IDE.
#include <Wire.h>
#include <LSM6.h>
#undef min
#undef max
#include <algorithm>

// This #include statement was automatically added by the Particle IDE.


#define LOGGING
#include <HttpClient.h>


// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_LSM303_U.h>

// This #include statement was automatically added by the Particle IDE.
#include <google-maps-device-locator.h>

#include <vector>

#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define SEGUNDOS_POR_MINUTO 60
#define SEGUNDOS_POR_HORA 3600


PRODUCT_ID(00); // replace by your product ID
PRODUCT_VERSION(1); // increment each time you upload to the console
SYSTEM_THREAD(ENABLED);
// for filter

HttpClient http;
TCPClient client;
// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
	//  { "Content-Type", "application/json" },
	//  { "Accept" , "application/json" },
	//   { "Accept" , "*/*"},
	{"Content-Type", "text/plain"},
	{NULL, NULL}  // NOTE: Always terminate headers will NULL
};

http_request_t request;
http_response_t response;

enum FILTER_TYPE { HIGHPASS, LOWPASS, INTEGRATOR, DIFFERENTIATOR };

// the recursive filter class implements a recursive filter (low / pass /
// highpass
// note that this must be updated in a loop, using the most recent acquired
// values and the time acquired
//   Y = a0*X + a1*Xm1
//              + b1*Ylast
struct FilterOnePole {
  FILTER_TYPE FT;
  float TauUS;     // decay constant of the filter, in US
  float TauSamps;  // tau, measued in samples (this changes, depending on how
  // long between input()s

  // filter values - these are public, but should not be set externally
  float Y;      // most recent output value (gets computed on update)
  float Ylast;  // prevous output value

  float X;  // most recent input value

  // elapsed times are kept in long, and will wrap every
  // 35 mins, 47 seconds ... however, the wrap does not matter,
  // because the delta will still be correct (always positive and small)
  float ElapsedUS;  // time since last update
  long LastUS;      // last time measured

  FilterOnePole(FILTER_TYPE ft = LOWPASS, float fc = 1.0,
      float initialValue = 0);

  // sets or resets the parameters and state of the filter
  void setFilter(FILTER_TYPE ft, float tauS, float initialValue);

  void setFrequency(float newFrequency);

  void setTau(float newTau);

  float input(float inVal);

  float output();

  void print();

  void test();

  void setToNewValue(float newVal);  // resets the filter to a new value
};

// two pole filter, these are very useful
struct FilterOnePoleCascade {
  FilterOnePole Pole1;
  FilterOnePole Pole2;

  FilterOnePoleCascade(
      float riseTime = 1.0,
      float initialValue = 0);  // rise time to step function, 10% to 90%

  // rise time is 10% to 90%, for a step input
  void setRiseTime(float riseTime);

  void setToNewValue(float newVal);

  float input(float inVal);

  float output();

  void test();
};

// variables configurables
const int FrecRPS = 100;  // frecuencia de muestreo en [hz]
const int DPTIME = 6000;  // tiempo  en [ms] para determinar desplazamiento
const int MTIME = 1500;   // tiempo en[ms] para determinar moviento
int RestartTime = 10 * SEGUNDOS_POR_MINUTO;  // tiempo en recalcular el
int OnlineTime = 60*SEGUNDOS_POR_MINUTO;                         // offset, las referencias y
// reestablecer las variables
short Factor_IQR = 5.0;  // Interquarile reference factor
short Factor_CAV = 1.5;  // Cumulative acelerator vector reference factor
short Factor_ZC = 1;     // Zero Crossing rate reference factor
short Factor_RSL = 3.5;
// variables de muestreo

unsigned int INTERVALO = (1000 / FrecRPS);  // periodo de muestreo en [ms]
unsigned int SAMPLES = 200;  // numero de muestras para el calculo
unsigned short c_muestra;    // cuentas de la ventana de analisis

std::vector<short> X;  // vector de la aceleración en el eje X
std::vector<short> Y;  // vector de la aceleración en el eje Y
std::vector<short> Z;  // vector de la aceleración en el eje Z
std::vector<long> XYZ;
short old_x, old_y, old_z;  // muestra anterior
float cutFrequency = 2;     // frecuancia de corte en [hz]


// Filtros paca da uno de los ejes
FilterOnePole filterOneHighpassX(
    HIGHPASS,
    cutFrequency);  // create a one pole (RC) highpass filter pmcFilter
FilterOnePole filterOneHighpassY(
    HIGHPASS,
    cutFrequency);  // create a one pole (RC) highpass filter pmcFilter
FilterOnePole filterOneHighpassZ(
    HIGHPASS,
    cutFrequency);  // create a one pole (RC) highpass filter pmcFilter

bool myfunction(long i, long j) { return (i < j); }

// variables para el cálculo de los parámetros
// Parameter variables
long ACNmax, ACNmin;                 // Aceleración neta máxima y mínima
int RSL, RSLref, RSLmin, RSLmax;   // Parametro Mr. P (Razon entre promedios)
long IQR, IQRref, IQRmin, IQRmax;    // Rango intercuartílico
int ZC, ZCref, ZCmax, ZCmin;       // tasa de cruces por cero
long CAV, CAVref, CAVmax, CAVmin;    // Aceleración neta acumulada
short acc_xMax, acc_yMax, acc_zMax;  // xyz-axis max acceleration
short t_amax;   // tiempo donde fue la aceleración máxima en [ms]
int trigger;  // 0:notrigger / 1:RSL / 2:ZIC / 3:avisoreceptor

// variables de tiempo
time_t t_millis;
time_t t_muestreo;
int    t_prevsecond;
time_t t_eventoled;        
time_t tc_reset;           //  reinicio a variables iniciales del sistema
time_t t_online;           // contador de tiempo para enviar un estado de conexión
// variables de estado
bool EVENT;                // event detection on/off
bool CSERVER;              // estado del servidor
bool DATASEND;             // envio de datos
bool MCHECK;               // movimiento
bool DPCHECK;              // desplazamiento
bool REFERENCIA_VARIABLE;  // parametros true = variables / false = fijos
bool REFERENCIA_CALCULO;   // parametros true = se están calculandos / false = no
bool REINICIO;             // reinicio de variables
bool CONECTADO;             // conectado al servidor prosismic
// se calculan
bool FILTER;               // true = habilitado / false = deshabilidato
bool toggle = false;       // led funcioamiento normal del sensor    
bool eventtoggle = false ; // led evento detectado
bool statustoggle = false; // led estados

// contadores de tiempo
unsigned short tc_mcheck;      // muestras que está en estado de movimiento
unsigned short tc_dpcheck;     // cuantas que está en estado de desplazamiento
unsigned short tc_temp1;       // counter temporal for DP event
unsigned short tc_temp2;       // counter temporal for DP event
unsigned short tc_cserver;     // muestras que el servidor no responde
unsigned short tc_dpchecktmp;  // temporal para estado de desplazamiento
unsigned short tc_referencia;  // muestras para calcular las referencias
unsigned short tc_geo;         // cuentas en que no se encuentra la georefencia de google maps
unsigned long lastSync = millis();

// contadores de estado
unsigned short ce_mcheck;   //  veces que está en movimiento
unsigned short ce_dpcheck;  //  veces que está en desplazamiento
unsigned short ce_event;    //  cuentas en estado de detección

// Definiciones de los pines
int EventPin = D5;   // wifi status LED
int StatusPin = D4;  // accelerometer data readings LED
int ProblemPin = D3;


// String para guardar la georeferencia
char latitude[32];   // latitude value from host
char longitude[32];  // longitude value from host

// integración de google maps con el Cloud de particle
GoogleMapsDeviceLocator locator;
int t_locationsync = ONE_DAY_MILLIS;

String ID;          // identifiación de cada sensor
char id[25];

LSM6 imu;          //variable del acelerómetro

// funciones callbacks
void myHandler(const char* event, const char* data);
void locationCallback(float lat, float lon, float accuracy);
int setZC(String _zc);
int setIQR(String _iqr);
int setCAV(String _cav);
int setRSL(String _rsl);
int setRestartTime(String _time);
int setOnlineTime(String _time);



void setup() {
  // Configuración inicial
  pinMode(EventPin, OUTPUT);
  pinMode(StatusPin, OUTPUT);
  pinMode(ProblemPin,OUTPUT);
  // Subscribe to the integration response event
  //Particle.subscribe("hook-response/Sismo", myHandler);
  //Particle.subscribe("particle/device/name", handler);
  //Particle.publish("particle/device/name");
  Particle.variable("ZCref", ZCref);
  Particle.variable("IQRref", IQRref);
  Particle.variable("CAVref", CAVref);
  Particle.variable("RSLref", RSLref);
  Particle.variable("Trigger", trigger);
  Particle.variable("Latitude", latitude);
  Particle.variable("Longitude", longitude);
  Particle.function("setZC",setZC);
  Particle.function("setIQR",setIQR);
  Particle.function("setCAV",setCAV);
  Particle.function("setRSL",setRSL);

  ID = System.deviceID();
  ID.toCharArray(id,ID.length());
  Serial.begin(50000);
  Wire.begin();

  blinking_delay(5000);

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    delay (1000);
  }
  imu.enableDefault();


  blinking_delay(5000);
  locator.withSubscribe(locationCallback).withLocatePeriodic(t_locationsync);

  // Valores iniciales
  t_prevsecond = millis();
  t_eventoled = 0;

  c_muestra = 0;
  old_x = 0;
  old_y = 0;
  old_z = 0;

  /* parameter variables */
  IQRref = 500;   // [mg] integer
  CAVref = 1000;  // [mg] integer
  RSLref = 300;   // percent para medir la ligua
  ZCref = 22;

  t_muestreo = millis();
  t_eventoled = 0;

  EVENT = false;
  CSERVER = true;
  DATASEND = false;
  MCHECK = false;
  DPCHECK = false;
  REFERENCIA_VARIABLE = true;
  REFERENCIA_CALCULO = true;
  FILTER = true;
  REINICIO = true;

  tc_mcheck = 0;
  tc_dpcheck = 0;
  tc_dpchecktmp = 0;
  tc_temp1 = 0;
  tc_temp2 = 0;
  tc_cserver = 0;
  tc_referencia = 0;
  tc_reset = Time.now();  // IMPORTANTE
  tc_geo = 0;

  ce_mcheck = 0;
  ce_dpcheck = 0;
  ce_event = 0;

}

void loop() {
  locator.loop();


  short t_dmuestreo;
  short t_dparam;
  char line[200];

  // reestablece el contador de muestras
  if (c_muestra == SAMPLES) c_muestra = 0;

  // variables de muestreo

  if (millis() - t_muestreo >= INTERVALO) {
    t_muestreo = millis();

    Serial.print(Time.now());
    Serial.print(" ");
    printDate();
    Serial.print(" ");

    Serial.print(" ");
    printTime();
    Serial.print(" ");

    t_dmuestreo = accSampling();

    snprintf(line,sizeof(line),"%hdsam %hd[ms] %hd %hd %hd %ld", c_muestra,
        t_dmuestreo, X[0], Y[0], Z[0], XYZ[0]);
    Serial.print(line);
    Serial.print(" ");

    t_dparam = calcParam(c_muestra);

    snprintf(line, sizeof(line), " %hd[ms] %hd %hd %hd %ld", t_dparam, acc_xMax,
        acc_yMax, acc_zMax, ACNmax);
    Serial.print(line);
    Serial.print(" ");

    snprintf(line, sizeof(line), "%d %ld %ld %d", ZC, IQR, CAV, RSL);
    Serial.print(line);
    Serial.print(" ");

    snprintf(line, sizeof(line), "%d %ld %ld %d", ZCref, IQRref, CAVref, RSLref);
    Serial.print(line);
    Serial.print(" ");


    antimovimiento();
    antidesplazamiento();
    control_de_eventos_consecutivos();
    evento_sin_conexion();
    reinicio_de_variables();

    Serial.print(" t_online  ");
    Serial.print((OnlineTime) - (Time.now()  - t_online));
    Serial.print("(s) ");
    //Serial.print(" lat: ");
    //Serial.print(latitude);
    //Serial.print(" lon: ");
    //Serial.print(longitude);
    //Serial.print(" ");
    //Serial.print(ID);

    Serial.println();


    c_muestra++;  // window sample counter
    tc_referencia++;


    // final del muestreo
  }

  // sincronización del tiempo una vez al día
  if (millis() - lastSync > ONE_DAY_MILLIS) {
    // Request time synchronization from the Particle Cloud
    Particle.syncTime();
    lastSync = millis();
  }

  // sincronización cada 1 segundo
  if (Time.second() != t_prevsecond) {
    t_prevsecond = Time.second();
    t_millis = millis();

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


    // En caso de no encontrar la posición que lo intente cada 4 segundos
    if(String(latitude).equals(NULL) == true ){
      locator.publishLocation();
      tc_geo++;
    }

    if(tc_geo >= 4){
      String(-32).toCharArray(latitude, 32);
      String(-72).toCharArray(longitude, 32);
    }


  }

  led_display();


  digitalWrite(LED_BUILTIN,toggle);
  digitalWrite(EventPin,eventtoggle);
  digitalWrite(StatusPin,statustoggle);

  // Detección de sismo según los parametros calculados y las referencias
  trigger = 0;
  // if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref) || (RSL >= RSLref) ) {
  if ((IQR > IQRref && CAV > CAVref)) {
    // if ( (IQR > IQRref && ZC < ZCref && CAV > CAVref)) {
    //  if ((IQR > IQRref && ZC < ZCref && CAV > CAVref)) trigger = 2;
    if ((IQR > IQRref && CAV > CAVref)) trigger = 2;
    // if (RSL >= RSLref)  trigger = 1;


    t_eventoled = Time.now();

    if (DATASEND) {
      DATASEND = false;

      ce_mcheck++;

      if (MCHECK == false) {
        tc_mcheck = 0;
        MCHECK = true;
      }

      ce_dpcheck++;
      if (DPCHECK == false) {
        tc_dpcheck = 0;
        tc_temp1 = 0;
        tc_temp2 = 0;
        DPCHECK = true;
      }

      eventtoggle = true;
      if (postRequestv2(Time.now()) == 1) {
        sendParticle();
        EVENT = true;
        eventtoggle = false;
        //   tc_reset = Time.now();

      } else {
        CSERVER = false;
        MCHECK = false;
        DPCHECK = false;
        Serial.print("No se encontró el servidor");
      }


    }


  }


  if ((Time.now() - tc_reset) >= (RestartTime)) {
    Serial.print(" Reinicio de variables ");
    tc_reset = Time.now();

    if (!EVENT) {
      // if (FILTER == false)
      //  offset(N_OFFSET);
      // t_eventoled = 0;

      c_muestra = 0;

      EVENT = false;
      CSERVER = true;
      DATASEND = false;
      MCHECK = false;
      DPCHECK = false;
      REFERENCIA_VARIABLE = false;
      REFERENCIA_CALCULO = true;
      REINICIO = true;
    }
  }

  /* Status send*/
  if ((Time.now() - t_online) >=  OnlineTime) {
    t_online = Time.now();
    sendStatus(1);
    Particle.publish("Online");

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
  char line[60];
  snprintf(line, sizeof(line), "%s;%lu;%d",id, Time.now(),_status);
  Serial.println(line);
  request.hostname = "prosismic.zeke.cl";
  request.path = "/registrarEstado";
  request.body = String(line);
  http.post(request, response, headers);
 return 0;
}

int led_display(){

  if (EVENT == true){
    toggle = true;
    statustoggle = false; // check
    eventtoggle = !eventtoggle;  // check
    return 0;
  }

  else { // EVENT == false

    /* led event display*/
    if(((Time.now()-t_eventoled) < 7*SEGUNDOS_POR_MINUTO) && DATASEND == true){
      /* update every 0.5 secs */

      if((millis()-t_millis) >= 500)
        statustoggle = false;


      else 
        statustoggle = true;

    }

    return 1;

  }

}


// se guarda la latitude y longitud
void locationCallback(float lat, float lon, float accuracy) {
  String(lat).toCharArray(latitude, 32);
  String(lon).toCharArray(longitude, 32);

}

// Se reinician los valores iniciales, se ajusta el offset y se calculan las
// referencias.

int reinicio_de_variables() {

  Serial.print(" t_reset ");
  Serial.print((RestartTime) - (Time.now()  - tc_reset));
  Serial.print("(s) ");
  if (( REINICIO == true ) && (X.size() >= SAMPLES)) {
    // En caso de no usar el filtro
    // if (tc_referencia == 0 && FILTER == false) {
    //     snprintf(line, sizeof(line), "Offset(x, y, z) = (%d,%d,%d)",
    //     offset_x, offset_y, offset_z);
    //  Serial.print(line);
    //        Serial.print(" ");
    //}

    // Se toman SAMPLES * PARAMETER_SAMPLES como muestras para calculo de las
    // referencias

    if (REFERENCIA_VARIABLE &&
        (CalcRef(tc_referencia, REFERENCIA_CALCULO, SAMPLES * 5) == 0)) {
      Serial.print("References Calculated");
      Serial.print(" ");
      DATASEND = true;
      REFERENCIA_CALCULO = false;
      REINICIO = false;
    }

    else {
      // usar parametros variables por particle
      Serial.print("Referencia Fija");
      IQRref = 500;   // [mg] integer
      CAVref = 1000;  // [mg] integer
      RSLref = 300;   // percent para medir la ligua
      ZCref = 22;
      DATASEND = true;
      REFERENCIA_CALCULO = false;
      REINICIO = false;
      tc_referencia = 0;
      c_muestra = 0;
      Serial.print(" ");
    }

    tc_dpcheck = 0;
    tc_mcheck = 0;
    tc_temp1 = 0;
    tc_temp2 = 0;
    tc_mcheck = 0;
    ce_mcheck = 0;

    client.stop();
    CONECTADO = client.connect("prosismic.zeke.cl",80);

    if(CONECTADO){
      Serial.print(" Connected to Prosismc");
    }
    else{
      client.stop();
      Serial.print(" Cannot Connect to Prosismic ");
    }
    return 1;
  }

  return 0;
}

// avisa cuando un evento no es notificado al servidor
int evento_sin_conexion() {
  if (!CSERVER) {  // wait for server resend
    Serial.print("Pause event for ");
    Serial.print(SAMPLES / 2 - tc_cserver);
    Serial.print(" ce_cserver");
    Serial.print(" ");
    tc_cserver++;

    /* check server count complete */
    if (tc_cserver >= SAMPLES / 2) {
      tc_cserver = 0;
      DATASEND = true;
      CSERVER = true;
    }
    return 1;
  }
  return 0;
}





// revisa si hay eventos conseuctivos y detiene el envío de eventos en caso
// afirmativo
int control_de_eventos_consecutivos() {
  if (EVENT) {  // pause after eventis detected

    /* check if event count comp	// Initialize sensor
       LIS3DHConfig config;
       config.setAccelMode(LIS3DH::RATE_100_HZ);lete */
    if (ce_event >= SAMPLES / 2) {
      ce_event = 0;
      DATASEND = true;
      EVENT = false;
    } else {
      Serial.print("Pause event for ");
      Serial.print(SAMPLES / 2 - ce_event);
      Serial.print(" ce_event");
      Serial.print(" ");
      ce_event++;
    }
    return 1;
  }
  return 0;
}





// revisa si hay desplazamiento del sensor
int antidesplazamiento() {
  if (DPCHECK == true) {
    char dpline[100];

    if (tc_dpcheck <= DPTIME) {  // check 6000 count windwow
      snprintf(dpline, sizeof(dpline),
          "PDEvent %d/3 in %4d count - ec1 %d ec2 %3d", ce_dpcheck,
          DPTIME - tc_dpcheck, tc_temp1, tc_temp2);
      Serial.print(dpline);
      Serial.print(" ");

      if (ce_dpcheck == 1) {
        tc_temp1++;
      }

      if (ce_dpcheck == 2) {
        if (tc_temp1 >= 300) {
          tc_temp2++;
        } else {
          tc_temp1 = 0;
          ce_dpcheck = 1;
        }
      }

      if (ce_dpcheck == 3) {
        if (tc_temp2 >= 300) {
          tc_dpcheck = 0;
          ce_dpcheck = 0;
          tc_dpchecktmp = 0;
          DATASEND = false;
          EVENT = false;
          DPCHECK = false;
          MCHECK = false;
          Serial.print(" Pause event sending until offset");
          Serial.print(" ");
        } else {
          tc_temp1 = 0;
          tc_temp2 = 0;
          ce_dpcheck = 1;
        }
      }
    }

    else {
      tc_dpcheck = 0;
      ce_dpcheck = 0;
      tc_dpchecktmp = 0;
      DPCHECK = false;
    }

    tc_dpcheck++;
    return 1;
  }
  return 0;
}




// revisa si hay movimiento duerante muestras


int antimovimiento() {
  char line[150];
  if (MCHECK == true) {
    if (tc_mcheck <= MTIME) {  // check 1500 count
      snprintf(line, sizeof(line), "MEvent %d/5 in %d count", ce_mcheck,
          (DPTIME - tc_mcheck));
      Serial.print(line);
      Serial.print(" ");
      tc_mcheck++;

      if (ce_mcheck >= 5) {  // are 5 event
        DATASEND = false;
        EVENT = false;
        MCHECK = false;
        DPCHECK = false;
        ce_event = 0;  // ready for next eventsamples
        Serial.print("Pause event sending until offset");
        Serial.print(" ");
      }
    }

    else {
      tc_mcheck = 0;
      ce_mcheck = 0;
      if (EVENT == false) MCHECK = false;
    }
    return 1;
  }

  return 0;
}

// Calculo de las referencias según los valores maximos y mininmos de las
// _samples muestras}


int CalcRef(int _i, bool _iscalc, int _samples) {
  char line[80];

  if (_i < _samples) {  // use total _samples for calculation

    if (_i >= 400) {  // use the samples -1 values to find max and min

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
      IQRref = maxx(long(300), (IQRmax - IQRmin) * Factor_IQR);
      CAVref = maxx(long(500), (CAVmax - CAVmin) * Factor_CAV);
      ZCref = float((ZCmax + ZCmin) / 2) - float((ZCmax - ZCmin) * Factor_ZC);
      RSLref = maxx(155, (RSLmax - RSLmin) * Factor_RSL);
    }

    else {  // use the first value for max and min

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

    snprintf(line, sizeof(line), " %d %d %ld %ld %ld %ld %d %d %d %ld %ld %d", ZCmax,
        ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, RSLmax, RSLmin, ZCref,
        IQRref, CAVref, RSLref);
    Serial.print(line);
    return 1;  // sigue calculando
  }

  return 0;
}

const short maxx(const short& a, const short& b) { return (a < b) ? b : a; }
const long maxx(const long& a, const long& b) { return (a < b) ? b : a; }
const int maxx(const int& a, const int& b) { return (a < b) ? b : a; }

int i = 0;


// Envía un evento al cloud de particle con la información del sismo.
int sendParticle() {
  char line[100];
  snprintf(line, sizeof(line),
      "%lu;%ld;%hd;%d;%ld;%ld;%d;%s;%s",
      (millis()-t_millis)%1000, ACNmax, trigger,ZC,IQR, CAV,RSL, latitude, longitude);
  Particle.publish("Sismo",line);
  EVENT = true;

  return 0;
}

// envía un post al servidor de prosismoc con la información del sismo

int postRequestv2(time_t _time) {
  // Request path and body can be set at runtime or at setup.
  char line[100];

  if (CONECTADO) {

    snprintf(line, sizeof(line), "%s;%lu;%lu;%ld;%d;%d;%ld;%ld;%d;%ld;%ld;%s;%s",
        id, _time, (millis() - t_millis) % 1000, ACNmax, trigger,
        ZCref, IQRref, CAVref, ZC, IQR, CAV, latitude, longitude);

    time_t _startcalc = millis();

    client.println("POST /registrarEvento HTTP/1.0");
    client.println("Connection: close");
    client.println("HOST: prosismic.zeke.cl");
    client.print("Content-Length: ");
    client.println(sizeof(line));
    client.println("Content-Type: text/plain");
    client.println();
    client.flush();
    client.println(line);

    Serial.println("POST /registrarEvento HTTP/1.0");
    Serial.println("Connection: close");
    Serial.println("HOST: prosismic.zeke.cl");
    Serial.print("Content-Length: ");
    Serial.println(sizeof(line));
    Serial.println("Content-Type: text/plain");
    Serial.println();
    Serial.flush();
    Serial.println(line);

    Serial.print(" ");
    // The library also supports sending a body with your request:
    // request.body = "{\"key\":\"value\"}";

    // Get request

    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }

    Serial.print(" t_send ");
    Serial.print(millis()-_startcalc);
    Serial.println("(ms) ");

    return 1;
  }
  else {
    return 0;
  }
}

// envía un post al servidor de prosismoc con la información del sismo
int postRequest(time_t _time) {
  // Request path and body can be set at runtime or at setup.
  char line[100];
  time_t _startcalc = millis();
  snprintf(line, sizeof(line), "%s;%lu;%lu;%ld;%d;%d;%ld;%ld;%d;%ld;%ld;%s;%s",
      id, _time, (millis() - t_millis) % 1000, ACNmax, trigger,
      ZCref, IQRref, CAVref, ZC, IQR, CAV, latitude, longitude);
  request.hostname = "prosismic.zeke.cl";
  // request.port = 80;
  request.path = "/registrarEvento";
  request.body = String(line);
  Serial.print(line);
  Serial.print(" ");
  // The library also supports sending a body with your request:
  // request.body = "{\"key\":\"value\"}";

  // Get request

  http.post(request, response, headers);

  Serial.print(" t_send ");
  Serial.print(millis()-_startcalc);
  Serial.println("(ms) ");
  Serial.print("Application>\tResponse status: ");
  Serial.print(response.status);
  Serial.print("Application>\tHTTP Response Body: ");
  Serial.print(response.body);
  if (response.body.toInt() == 1) {
    return 1;
  } else {
    return 0;
  }
  return 1;
}




// Calcula los parametros CAV , RSL , IQR and ZC y retorna el tiempo que se
// demora en [ms]
time_t calcParam(int _sample) {
  long sumshort = 0;
  long sumlong = 0;
  float cavlong = 0;
  float cavshort = 0;
  float _smpl = SAMPLES / 8;
  bool xcurrent;
  bool xprevious;
  bool ycurrent;
  bool yprevious;
  bool zcurrent;
  bool zprevious;
  short xzcr = 0;
  short yzcr = 0;
  short zzcr = 0;
  short acc_xabs;
  short acc_yabs;
  short acc_zabs;
  std::vector<long> XYZ_temp;
  int start = millis();

  acc_xabs = abs(X[0]);
  acc_yabs = abs(Y[0]);
  acc_zabs = abs(Z[0]);

  /* first values for max and min comparison */
  acc_xMax = abs(X[0]);
  acc_yMax = abs(Y[0]);
  acc_zMax = abs(Z[0]);
  ACNmax = XYZ[0];

  if (X.size() >= SAMPLES) {
    /* Sampling window analysis */
    for (unsigned int jj = 0; jj < X.size(); jj++) {
      /* Max and min comparison */
      acc_xabs = abs(X[jj]);
      acc_yabs = abs(Y[jj]);
      acc_zabs = abs(Z[jj]);

      if (XYZ[jj] > ACNmax) {
        ACNmax = XYZ[jj];

        t_amax = (millis() - t_millis) % 1000;
      }
      if (acc_xabs > acc_xMax) {
        acc_xMax = acc_xabs;
      }
      if (acc_yabs > acc_yMax) {
        acc_yMax = acc_yabs;
      }
      if (acc_zabs > acc_zMax) {
        acc_zMax = acc_zabs;
      }

      /* CAV arrays */
      XYZ_temp.insert(XYZ_temp.begin(), XYZ[jj]);
      sumlong += XYZ[jj];

      if (jj < _smpl) {
        sumshort += XYZ[jj];
      }

      /* ZC */
      if (jj >= 1) {
        if (X[jj - 1] < 0) xprevious = false;
        if (X[jj] < 0) xcurrent = false;
        if (Y[jj] < 0) ycurrent = false;
        if (Z[jj] < 0) zcurrent = false;
        if (Z[jj - 1] < 0) zprevious = false;
        if (Y[jj - 1] < 0) yprevious = false;

        if (X[jj] > 0) xcurrent = true;
        if (X[jj - 1] > 0) xprevious = true;
        if (Y[jj] > 0) ycurrent = true;
        if (Y[jj - 1] > 0) yprevious = true;
        if (Z[jj] > 0) zcurrent = true;
        if (Z[jj - 1] > 0) zprevious = true;

        // if the sign is different
        if ((xcurrent != xprevious) && X[jj] != 0)

        {
          // add one to the zero crossing rate
          xzcr = xzcr + 1;
        }
        if ((ycurrent != yprevious) && Y[jj] != 0) {
          // add one to the zero crossing rate
          yzcr = yzcr + 1;
        }
        if ((zcurrent != zprevious) && Z[jj] != 0) {
          // add one to the zero crossing rate
          zzcr = zzcr + 1;
        }
      }
    }

    cavlong = (sumlong / SAMPLES);
    cavshort = (sumshort / _smpl);
    CAV = cavlong;
    ZC = (xzcr + yzcr + zzcr) * 10.0 / (3.0 * 2.0);
    RSL = int((cavshort / cavlong) * 100);  //
    std::sort(XYZ_temp.begin(), XYZ_temp.end(), myfunction);
    IQR = (XYZ_temp[SAMPLES / 4 * 3 - 1] - XYZ_temp[SAMPLES / 4 - 1]);
  }

  return (millis() - start);
}

// Lee los valores del acelerometro y los guarda como Vector
int accSampling() {
  int _x;
  int _y;
  int _z;
  unsigned long _comienzo = millis();
  imu.read();

  filterOneHighpassX.input(
      imu.a.x /
      16);  // update the one pole lowpass filter, and statistics pmcFilter
  filterOneHighpassY.input(
      imu.a.y /
      16);  // update the one pole lowpass filter, and statistics pmcFilter
  filterOneHighpassZ.input(
      imu.a.z /
      16);  // update the one pole lowpass filter, and statistics pmcFilter


  _x = 0.5 * (filterOneHighpassX.output()) + 0.5 * old_x;
  _y = 0.5 * (filterOneHighpassY.output()) + 0.5 * old_y;
  _z = 0.5 * (filterOneHighpassZ.output()) + 0.5 * old_z;

  if (X.size() == SAMPLES) {
    X.pop_back();
    Y.pop_back();
    Z.pop_back();
    XYZ.pop_back();
  }

  X.insert(X.begin(), _x);
  Y.insert(Y.begin(), _y);
  Z.insert(Z.begin(), _z);

  XYZ.insert(XYZ.begin(), sqrt(_x * _x + _y * _y + _z * _z) * 100.0);

  old_x = _x;
  old_y = _y;
  old_z = _z;

  return (millis() - _comienzo);
}

void printTime() {
  Serial.print(Time.hour());
  Serial.print(":");
  Serial.print(Time.minute());
  Serial.print(":");
  Serial.print(Time.second());
  Serial.print(".");
  Serial.print((millis() - t_millis) % 1000);
}

void printDate() {
  Serial.print(Time.day());
  Serial.print("/");
  Serial.print(Time.month());
  Serial.print("/");
  Serial.print(Time.year());

}

// Encendido y apagado del led con un tiempo _delay
// lo uso para mostrar actvidad entre funciones
void blinking_delay(int _delay) {
  digitalWrite(StatusPin, HIGH);
  digitalWrite(EventPin, HIGH);
  digitalWrite(ProblemPin, HIGH);

  delay(_delay);

  digitalWrite(StatusPin, LOW);
  digitalWrite(EventPin, LOW);
  digitalWrite(ProblemPin, LOW);
}

FilterOnePole::FilterOnePole(FILTER_TYPE ft, float fc, float initialValue) {
  setFilter(ft, fc, initialValue);
}

void FilterOnePole::setFilter(FILTER_TYPE ft, float fc, float initialValue) {
  FT = ft;
  setFrequency(fc);

  Y = initialValue;
  Ylast = initialValue;
  X = initialValue;

  LastUS = micros();
}

float FilterOnePole::input(float inVal) {
  long time = micros();
  ElapsedUS = float(time - LastUS);  // cast to float here, for math
  LastUS = time;                     // update this now

  // shift the data values
  Ylast = Y;
  X = inVal;  // this is now the most recent input value

  // filter value is controlled by a parameter called X
  // tau is set by the user in microseconds, but must be converted to samples
  // here
  TauSamps = TauUS / ElapsedUS;

  float ampFactor;
#ifdef ARM_FLOAT
  ampFactor = expf(-1.0 / TauSamps);  // this is 1 if called quickly
#else
  ampFactor = exp(-1.0 / TauSamps);  // this is 1 if called quickly
#endif

  Y = (1.0 - ampFactor) * X + ampFactor * Ylast;  // set the new value

  return output();
}

void FilterOnePole::setFrequency(float newFrequency) {
  setTau(1.0 / (TWO_PI * newFrequency));  // τ=1/ω
}

void FilterOnePole::setTau(float newTau) { TauUS = newTau * 1e6; }

float FilterOnePole::output() {
  // figure out which button to read
  switch (FT) {
    case LOWPASS:
      // return the last value
      return Y;
      break;
    case INTEGRATOR:
      // using a lowpass, but normaize
      return Y * (TauUS / 1.0e6);
      break;
    case HIGHPASS:
      // highpass is the _difference_
      return X - Y;
      break;
    case DIFFERENTIATOR:
      // like a highpass, but normalize
      return (X - Y) / (TauUS / 1.0e6);
      break;
    default:
      // should never get to here, return 0 just in case
      return 0;
  }
}

void FilterOnePole::print() {
  Serial.println("");
  Serial.print(" Y: ");
  Serial.print(Y);
  Serial.print(" Ylast: ");
  Serial.print(Ylast);
  Serial.print(" X ");
  Serial.print(X);
  Serial.print(" ElapsedUS ");
  Serial.print(ElapsedUS);
  Serial.print(" TauSamps: ");
  Serial.print(TauSamps);
  // Serial.print(" ampFactor " );       Serial.print( ampFactor );
  Serial.print(" TauUS: ");
  Serial.print(TauUS);
  Serial.println("");
}

void FilterOnePole::test() {
  float tau = 10;
  float updateInterval = 1;
  float nextupdateTime = millis() * 1e-3;

  float inputValue = 0;
  FilterOnePole hp(HIGHPASS, tau, inputValue);
  FilterOnePole lp(LOWPASS, tau, inputValue);

  while (true) {
    float now = millis() * 1e-3;

    // switch input values on a 20 second cycle
    if (round(now / 20.0) - (now / 20.0) < 0)
      inputValue = 0;
    else
      inputValue = 100;

    hp.input(inputValue);
    lp.input(inputValue);

    if (now > nextupdateTime) {
      nextupdateTime += updateInterval;

      Serial.print("inputValue: ");
      Serial.print(inputValue);
      Serial.print("\t high-passed: ");
      Serial.print(hp.output());
      Serial.print("\t low-passed: ");
      Serial.print(lp.output());
      Serial.println();
    }
  }
}

void FilterOnePole::setToNewValue(float newVal) { Y = Ylast = X = newVal; }

// stuff for filter2 (lowpass only)
// should be able to set a separate fall time as well
FilterOnePoleCascade::FilterOnePoleCascade(float riseTime, float initialValue) {
  setRiseTime(riseTime);
  setToNewValue(initialValue);
}

void FilterOnePoleCascade::setRiseTime(float riseTime) {
  float tauScale = 3.36;  // found emperically, by running test();

  Pole1.setTau(riseTime / tauScale);
  Pole2.setTau(riseTime / tauScale);
}

float FilterOnePoleCascade::input(float inVal) {
  Pole2.input(Pole1.input(inVal));
  return output();
}

// clears out the values in the filter
void FilterOnePoleCascade::setToNewValue(float newVal) {
  Pole1.setToNewValue(newVal);
  Pole2.setToNewValue(newVal);
}

float FilterOnePoleCascade::output() { return Pole2.output(); }

void FilterOnePoleCascade::test() {
  // make a filter, how fast does it run:

  float rise = 1.0;
  FilterOnePoleCascade myFilter(rise);

  // first, test the filter speed ...
  long nLoops = 1000;

  Serial.print("testing filter with a rise time of ");
  Serial.print(rise);
  Serial.print("s");

  Serial.print("\n running filter speed loop ... ");

  float startTime, stopTime;

  startTime = millis() * 1e-3;
  for (long i = 0; i < nLoops; ++i) {
    myFilter.input(PI);  // use pi, so it will actually do a full calculation
  }
  stopTime = millis() * 1e-3;

  Serial.print("done, filter runs at ");
  Serial.print(float(nLoops) / (stopTime - startTime));
  Serial.print(" hz ");
  Serial.print("\n filter value: ");
  Serial.print(myFilter.output());

  myFilter.setToNewValue(0.0);
  Serial.print("\n after reset to 0: ");
  Serial.print(myFilter.output());

  Serial.print("\n testing rise time (10% to 90%) ...");

  bool crossedTenPercent = false;
  while (myFilter.output() < 0.9) {
    myFilter.input(1.0);
    if (myFilter.output() > 0.1 && !crossedTenPercent) {
      // filter first crossed the 10% point
      startTime = millis() * 1e-3;
      crossedTenPercent = true;
    }
  }
  stopTime = millis() * 1e-3;

  Serial.print("done, rise time: ");
  Serial.print(stopTime - startTime);

  Serial.print("testing attenuation at f = 1/risetime");

  myFilter.setToNewValue(0.0);

  float maxVal = 0;
  float valWasOutputThisCycle = true;

  while (true) {
    float now = 1e-3 * millis();

    float currentFilterVal = myFilter.input(sin(TWO_PI * now));

    if (currentFilterVal < 0.0) {
      if (!valWasOutputThisCycle) {
        // just crossed below zero, output the max
        Serial.print(maxVal * 100);
        Serial.print(" %\n");
        valWasOutputThisCycle = true;
      }
    }
  }
}

int setZC(String _zc){
  ZCref = _zc.toInt();
  return 0;
}
int setIQR(String _iqr){
  IQRref = _iqr.toInt();
  return 0;
}
int setCAV(String _cav){
  CAVref = _cav.toInt();
  return 0;
}
int setRSL(String _rsl){
  RSLref = _rsl.toInt();
  return 0;
}
int setRestartTime(String _time){
  RestartTime = _time.toInt()*SEGUNDOS_POR_MINUTO;
  return 0;
}
int setOnlineTime(String _time){
  OnlineTime = _time.toInt()*SEGUNDOS_POR_MINUTO;
  return 0;
}
