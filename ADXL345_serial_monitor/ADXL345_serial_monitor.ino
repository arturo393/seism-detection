#include <Wire.h>
#include <ADXL345.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <TimeLib.h>
#include <Filters.h>


byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

#define SAMPLES 100

float testFrequency = 1;                     // test signal frequency (Hz)
float testAmplitude = 100;                   // test signal amplitude
float testOffset = 100;
float amp = 1000;

float windowLength = 20.0 / testFrequency;   // how long to average the signal, for statistist
unsigned int localPort = 8888;       // local port to listen for UDP packets

IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
char xbuffn[10];  //Cadena donde almacenaremos el número convertido
char ybuffn[10];
char zbuffn[10];
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
IPAddress server(10, 5, 0, 201); // Super Server
const int port = 10003;
EthernetClient client; // Initialize the Ethernet client


/* for accelerometer */
Vector sca;
int INTPin = 3;
int ledPin = 13; // LED connected to digital pin 13

ADXL345 accelerometer;
short old_x, old_y, old_z;
short acc_x, acc_y,acc_z;
long offset_x, offset_y, offset_z;
time_t prevDisplay = 0; // when the digital clock was displayed

time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);
FilterOnePole filterOneHighpassX;
FilterOnePole filterOneHighpassY;
FilterOnePole filterOneHighpassZ;

const int FrecRPS = 100; // Tasa de muestreo definida por el usuario (100 a 200)
unsigned int INTERVALO = 1000 / FrecRPS; // ms
volatile unsigned long tiempo;

void setup()
{

  Serial.begin(115200);
  while (Ethernet.begin(mac) == 0)
    Serial.print("Failed to configure Ethernet using DHCP");

  Serial.print("DHCP ip address: ");
  Serial.println(Ethernet.localIP());
  // while (client.connect(server, port))
  //   Serial.print("Failed to connect to Prosismc Server");


  Serial.println("Getting time from NTP");
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  Serial.print("Current time: ");
  digitalClockDisplay();
  delay(1000);

  // Serial.println("Initialize ADXL345");
  if (!accelerometer.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
  } else {
    accelerometer.useInterrupt(ADXL345_INT1);
    accelerometer.setRange(ADXL345_RANGE_2G);
    accelerometer.setDataRate(ADXL345_DATARATE_200HZ);
    pinMode(ledPin, OUTPUT);      // sets the digital pin 13 as output
    pinMode(INTPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTPin), DRAcc_ISR , RISING);
  }
  delay(1000);
/*
  filterOneHighpassX.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a o:qne pole (RC) highpass filter
  filterOneHighpassY.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a one pole (RC) highpass filter
  filterOneHighpassZ.setFilter( HIGHPASS, testFrequency, 0.0 );  // create a one pole (RC) highpass filter
*/
   offset(100);
  Serial.println("Setup Finished");

  delay(1000);


}
volatile int sample = 0;
time_t prev;
volatile time_t time1;
volatile time_t time2;
int j;

void loop()
{

 if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      sample = 0;
    }
    if (millis() - tiempo >= INTERVALO) {
    tiempo = millis();
    sca = accelerometer.readmg();

    acc_x = 0.5 * (sca.XAxis - offset_x) + 0.5 *  old_x;
    acc_y = 0.5 * (sca.YAxis - offset_y) + 0.5 *  old_y;
    acc_z = 0.5 * (sca.ZAxis - offset_z) + 0.5 *  old_z;
    
   Serial.print("RTC ");
  Serial.print(now());
  Serial.print(" ");
//  filterOneHighpassX.input( sca.XAxis );
  Serial.print(acc_x);
//  Serial.print((int) filterOneHighpassX.output());
  Serial.print(" ");
//  filterOneHighpassY.input( sca.YAxis );
//  Serial.print((int)  filterOneHighpassY.output());
  Serial.print(acc_y);
  Serial.print(" ");
  Serial.println(acc_z);
  // filterOneHighpassZ.input( sca.ZAxis );
  //Serial.println((int)  filterOneHighpassZ.output());

  old_x = acc_x;
  old_y = acc_y;
  old_z = acc_z;
  sample++;
    }
  }
}

void DRAcc_ISR () {



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

void offset(int samples) {
  int j = 0;

  while (j < samples) {
    if (millis() - tiempo >= INTERVALO) {
      
    sca = accelerometer.readmg();
      offset_x += sca.XAxis;
      offset_y += sca.YAxis;
      offset_z += sca.ZAxis;
      j++;
    }

  }
  offset_x = offset_x / samples;
  offset_y = offset_y / samples;
  offset_z = offset_z / samples;

}

/*-------- NTP code ----------*/


time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
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
      return secsSince1900 - 2208988800UL - 3 * 3600;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
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
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
