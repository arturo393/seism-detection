/*
 * Project q_up
 * Description: Programa para la detección de sismos a través de la medición de un acelerómetro
 * y el cácluclo de parámetros.
 * Author: Arturo Veras
 * Date: 31 de octubre del 2017
 */


// This #include statement was automatically added by the Particle IDE.
#include <Wire.h>                        // Para la comunicación I2C
#include <LSM6.h>                        // Para la comunicación con el acelerómetro
#undef min
#undef max
#include <algorithm>                     // Para ordenar el arreglego de datos
#include <HttpClient.h>                  // Para el envío del estado al servidor prosismic
#include <google-maps-device-locator.h>  // Para la georeferencia
#include <vector>                        // Para almacenar la ventana de datos
#include <Filters.h>
// Definiciones ütiles
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define SEGUNDOS_POR_MINUTO 60
#define SEGUNDOS_POR_HORA 3600


PRODUCT_ID(6135); // replace by your product ID
PRODUCT_VERSION(4); // increment each time you upload to the console
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



// Filtros paca da uno de los ejes
float cutFrequency = 2;     // frecuancia de corte en [hz]
FilterOnePole filterOneHighpassX(HIGHPASS,cutFrequency);  // create a one pole (RC) highpass filter pmcFilter
FilterOnePole filterOneHighpassY(HIGHPASS,cutFrequency);  // create a one pole (RC) highpass filter pmcFilter
FilterOnePole filterOneHighpassZ(HIGHPASS,cutFrequency);  // create a one pole (RC) highpass filter pmcFilter

bool myfunction(long i, long j) { return (i < j); }

// variables para el cálculo de los parámetros
long ACNmax, ACNmin;                 // Aceleración neta máxima y mínima
int RSL, RSLref, RSLmin, RSLmax;     // Parametro Mr. P (Razon entre promedios)
long IQR, IQRref, IQRmin, IQRmax;    // Rango intercuartílico
int ZC, ZCref, ZCmax, ZCmin;         // tasa de cruces por cero
long CAV, CAVref, CAVmax, CAVmin;    // Aceleración neta acumulada
short acc_xMax, acc_yMax, acc_zMax;  // xyz-axis max acceleration
short t_amax;                        // tiempo donde fue la aceleración máxima en [ms]
int trigger;                         // 0:notrigger / 1:RSL / 2:ZIC / 3:avisoreceptor

// variables de tiempo
time_t t_millis;
time_t t_muestreo    = millis();
int    t_prevsecond;
time_t t_eventoled   = 0;
time_t tc_reset      = Time.now();    //  reinicio a variables iniciales del sistema
time_t t_online;           // contador de tiempo para enviar un estado de conexión

// variables de estado
bool EVENT               = false; // event detection on/off
bool CSERVER             = true;  // estado del servidor
bool DATASEND            = false; // envio de datos
bool MCHECK              = false; // movimiento
bool DPCHECK             = false; // desplazamiento
bool REFERENCIA_VARIABLE = false;  // parametros true = variables / false = fijos
bool REFERENCIA_CALCULO  = true;  // parametros true = se están calculandos / false = no
bool REINICIO            = true;  // reinicio de variables
bool CONECTADO           = true;  // conectado al servidor prosismic

// cambios de estado del los leds
bool toggle       = false;  // led funcioamiento normal del sensor
bool eventtoggle  = false ; // led evento detectado
bool statustoggle = false;  // led estados

// contadores de tiempo
unsigned short tc_mcheck     = 0;     // muestras que está en estado de movimiento
unsigned short tc_dpcheck    = 0;     // cuantas que está en estado de desplazamiento
unsigned short tc_temp1      = 0;     // counter temporal for DP event
unsigned short tc_temp2      = 0;     // counter temporal for DP event
unsigned short tc_cserver    = 0;     // muestras que el servidor no responde
unsigned short tc_dpchecktmp = 0;     // temporal para estado de desplazamiento
unsigned short tc_referencia = 0;     // muestras para calcular las referencias
unsigned short tc_geo        = 0;     // cuentas en que no se encuentra la georefencia de google maps
unsigned long lastSync       = millis();


// contadores de estado
unsigned short ce_mcheck;   //  veces que está en movimiento
unsigned short ce_dpcheck;  //  veces que está en desplazamiento
unsigned short ce_event;    //  cuentas en estado de detección

// Definiciones de los pines
int EventPin  = D5;   // wifi status LED
int StatusPin = D4;  // accelerometer data readings LED
int ServerPin = D3;


// String para guardar la georeferencia
char latitude[32]="0";   // latitude value from host
char longitude[32]="0";  // longitude value from host

// integración de google maps con el Cloud de particle
GoogleMapsDeviceLocator locator;
int t_locationsync = ONE_DAY_MILLIS;

String ID;          // identifiación de cada sensor
char id[25];
LSM6 imu;          //variable del acelerómetro

// funciones callbacks
void locationCallback(float lat, float lon, float accuracy);

// funciones disposibles en el Cloud
int setZC(String _zc);
int setIQR(String _iqr);
int setCAV(String _cav);
int setRSL(String _rsl);
int setReferencia(String _ref);
int setRestartTime(String _time);
int setOnlineTime(String _time);




bool reiniciando();
int evento_sin_conexion();
int control_de_eventos_consecutivos();
int antidesplazamiento();
int antimovimiento();

int accSampling();
bool CalcRef(int _i, int _samples);
time_t calcParam(int _sample);

int sendParticle();
int postRequestv2(time_t _time);
int postRequest(time_t _time);
int sendStatus(int _status);

void printTime();
void printDate();
void blinking_delay(int _delay);
int led_display();


void setup() {

  // Configuración los LEDS
  pinMode(EventPin, OUTPUT);
  pinMode(StatusPin, OUTPUT);
  pinMode(ServerPin,OUTPUT);

  // Para leer variablse desde el Cloud
  Particle.variable("ZCref", ZCref);
  Particle.variable("IQRref", IQRref);
  Particle.variable("CAVref", CAVref);
  Particle.variable("RSLref", RSLref);
 // Particle.variable("Trigger", trigger);
  Particle.variable("Latitude", latitude);
  Particle.variable("Longitude", longitude);

  // Para ejecutar funciones desde el Cloud
  Particle.function("setZC",setZC);
  Particle.function("setIQR",setIQR);
  Particle.function("setCAV",setCAV);
  Particle.function("setRSL",setRSL);
  Particle.function("Restartmin",setRestartTime);
  Particle.function("Onlinemin",setOnlineTime);
  Particle.function("Referencia",setReferencia);

  // Nombre del dispositivo
  ID = System.deviceID();
  ID.toCharArray(id,ID.length());

  Serial.begin(50000);
  Wire.begin();

  blinking_delay(1000);

  if (!imu.init()){
    Serial.println("Failed to detect and initialize IMU!");
    delay (1000);
  }
  imu.enableDefault();


  blinking_delay(1000);
 // locator.withSubscribe(locationCallback).withLocatePeriodic(t_locationsync);

  locator.withLocateOnce();
  // Valores iniciales
  t_prevsecond = millis();
  t_eventoled = 0;

  c_muestra = 0;
  old_x = 0;
  old_y = 0;
  old_z = 0;

  IQRref =  500;
  CAVref =  1000;
  RSLref =  300;
  ZCref  =  22;

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

    snprintf(line,sizeof(line),"%4hdsam %4hd %4hd %4hd %5ld", c_muestra, X[0], Y[0], Z[0], XYZ[0]);
    Serial.print(line);
    Serial.print(" ");

    t_dparam = calcParam(c_muestra);

    snprintf(line, sizeof(line), " %3hd %3hd %3hd %5ld", acc_xMax,acc_yMax, acc_zMax, ACNmax);
    Serial.print(line);
    Serial.print(" ");

    snprintf(line, sizeof(line), "%3d %5ld %6ld %4d", ZC, IQR, CAV, RSL);
    Serial.print(line);
    Serial.print(" ");

    snprintf(line, sizeof(line), "%d %ld %ld %d", ZCref, IQRref, CAVref, RSLref);
    Serial.print(line);
    Serial.print(" ");


//    Serial.print(" t_online  ");
//    Serial.print((OnlineTime) - (Time.now()  - t_online));
//    Serial.print("(s) ");

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

      } else {
        CSERVER = false;
        MCHECK = false;
        DPCHECK = false;
        Serial.print("No se encontró el servidor");
      }


    }


  }

    antimovimiento();
    antidesplazamiento();
    control_de_eventos_consecutivos();
    evento_sin_conexion();
    reiniciando();

    Serial.println();

    c_muestra++;  // window sample counter

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
    if((String(latitude).equals("0") == true) && (tc_geo == 60) )
      locator.withSubscribe(locationCallback).publishLocation();
    if(tc_geo>60 )   
      tc_geo =0;  

    tc_geo++;




  }

  // mantiene la conexión con el servidor

  if(client.connected() == false){
    digitalWrite(ServerPin,HIGH);
    client.stop();
    client.connect("prosismic.zeke.cl",80);
    Serial.println(" Connected to Prosismc ");

  }
  else {
    if (client.available()) {
      char c = client.read();
      // Serial.write(c);
    }
    digitalWrite(ServerPin,LOW);
  }


  led_display();


  digitalWrite(LED_BUILTIN,toggle);
  digitalWrite(EventPin,eventtoggle);
  digitalWrite(StatusPin,statustoggle);


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

bool reiniciando() {

  time_t tiempo = (RestartTime) - (Time.now()  - tc_reset);

  if(tiempo % 60 == 0){
    Serial.print(" t_reset ");
    Serial.print(tiempo / 60);
    Serial.print("(min) ");
  }

  if (( REINICIO == true ) && (X.size() >= SAMPLES)) {

    if (REFERENCIA_VARIABLE  == true ){ // solo se modifica al inicio

      if( CalcRef(tc_referencia, SAMPLES * 5) == false ) { // está calculando
        DATASEND = false;
        REINICIO = true;
        tc_referencia++;
        return true;
      }
      else { // terminó el conteo para el calculo de la referencia
        DATASEND = true;
        REINICIO = false;
        tc_referencia = 0;
        client.stop();
        tc_dpcheck = 0;
        tc_mcheck = 0;
        tc_temp1 = 0;
        tc_temp2 = 0;
        tc_mcheck = 0;
        ce_mcheck = 0;
        return false;
      }

    }


    else { // REFERNCIA_VARIABLE = false
      // usar parametros variables por particle
      Serial.print("Referencia Fija");
      IQRref = 500;   // [mg] integer
      CAVref = 1000;  // [mg] integer
      RSLref = 300;   // percent para medir la ligua
      ZCref = 22;
      DATASEND = true;
      REINICIO = false;
      client.stop();
      c_muestra = 0;
      tc_dpcheck = 0;
      tc_mcheck = 0;
      tc_temp1 = 0;
      tc_temp2 = 0;
      tc_mcheck = 0;
      ce_mcheck = 0;
      Serial.print(" ");
      return false;
    }

  }  // fin del calculo de las referencias

  return false;
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
      Serial.print("Pause for ");
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
      snprintf(dpline, sizeof(dpline),"PDEvent %d/3 in %4d count - ec1 %d ec2 %d", 
          ce_dpcheck, DPTIME - tc_dpcheck, tc_temp1, tc_temp2);
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


bool CalcRef(int _i, int _samples) {
  char line[80];

  if (_i < _samples) {  // numero de muestras para el calculo

    if (_i < 400) {  // las primeras 400 muestras son de estabilización

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

    else {  // las siguientes muestras para buscar el máximo y mínimo

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


    // muestra los valores estabilizados y los máximos y mínimos
    snprintf(line, sizeof(line), " %d %d %ld %ld %ld %ld %d %d", ZCmax,
        ZCmin, IQRmax, IQRmin, CAVmax, CAVmin, RSLmax, RSLmin);
    Serial.print(line);
    return false;  // sigue calculando
  }
  else { // se terminan las muestras

    Serial.print("References Calculated");
    Serial.print(" ");

    return true; // terminó el conteo de las muestras
  }
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

  if (client.connected()) {

    snprintf(line, sizeof(line), "%s;%ld;%ld;%ld;%d;%d;%ld;%ld;%d;%ld;%ld;%s;%s",
        id, _time, (millis() - t_millis) % 1000, ACNmax, trigger,
        ZCref, IQRref, CAVref, ZC, IQR, CAV, latitude, longitude);

    time_t _startcalc = millis();

    client.print("POST /registrarEvento HTTP/1.1\r\n");
    client.print("Connection: keep-alive\r\n");
    client.print("HOST: prosismic.zeke.cl\r\n");
    client.print("Content-Length: ");
    client.print(strlen(line));
    client.print("\r\n");
    client.print("Content-Type: text/plain\r\n\r\n");
    client.print(line);
    // The library also supports sending a body with your request:
    // request.body = "{\"key\":\"value\"}";

    // Get request

    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }

    //  Serial.print(" t_send ");
    //  Serial.print(millis()-_startcalc);
    //  Serial.print("(ms) ");

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

  //  filteronehighpassx.input( imu.a.x / 16);  // update the one pole lowpass filter, and statistics pmcfilter
  //  filteronehighpassy.input(imu.a.y /16);  // update the one pole lowpass filter, and statistics pmcfilter
  //  filteronehighpassz.input(imu.a.z /16);  // update the one pole lowpass filter, and statistics pmcfilter

  filterOneHighpassX.input( imu.a.x * 0.061);  // update the one pole lowpass filter, and statistics pmcfilter
  filterOneHighpassY.input( imu.a.y * 0.061);  // update the one pole lowpass filter, and statistics pmcfilter
  filterOneHighpassZ.input( imu.a.z * 0.061);  // update the one pole lowpass filter, and statistics pmcfilter

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

  delay(_delay);

  digitalWrite(StatusPin, LOW);
  digitalWrite(EventPin, LOW);
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

int setReferencia(String _ref){

  int var = _ref.toInt();
  switch(var){
    case 1: REFERENCIA_VARIABLE = true;
    case 0: REFERENCIA_VARIABLE = false;
    default :REFERENCIA_VARIABLE = false;
  }
  return 0;
}
