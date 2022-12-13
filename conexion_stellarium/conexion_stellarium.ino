/*Librerias utilizadas en el programa*/
#include <Ethernet.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <RTClib.h>

/*Declaraciones de pines ethernet*/
#define PINSS 4
#define PINRESET 3

/*Creacion del objeto servidor*/
#define PORT_STELLARIUM 10000
EthernetServer stell(PORT_STELLARIUM);

/*Coordenadas geograficas del IAR*/
#define LONGITUD -58.139302070143444
#define LATITUD  -34.866431 // sur
#define DEC_STAR_TEST -69.723071 // f1 tau 

unsigned long int RA12HS = 0x80000000;
long int DEC10 = 0x40000000;
long int dec;
unsigned long int RA = 0;


typedef struct
{
  uint16_t length;
  uint16_t type;
  uint64_t unix_time;
  uint32_t ra;
  int32_t dec;
  uint32_t status;
} stellarium_t;

stellarium_t st;

uint8_t data[24];  // vector datos recibidos desde stellarium

unsigned int localPort = 8888;  // local port to listen for UDP packets
const char timeServer[] = "time.nist.gov";  // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

RTC_Millis rtc;
DateTime now;

void setup()
{

  /*Inicializacion de ethernet*/
  Serial.begin(57600);
  Ethernet.init(PINSS);
  byte mac[] = { 0xBC, 0xCB, 0xAF, 0xAB, 0xAA, 0x01 };  // dirmac

  /*Obtencion de IP por DHCP*/
  if (Ethernet.begin(mac) == 0) {

    Serial.print(F("Fallo DHCP"));

  } else {
    Serial.println(Ethernet.localIP());
  }

  /*Obtencion de tiempo unix desde la red*/
  Udp.begin(localPort);
  sendNTPpacket(timeServer);  // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket())
  {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.print("Seconds since Jan 1 1900 = ");
    //Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    //Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    //Serial.println(epoch);

    /*Inicio el rtc con el tiempo unix*/
    rtc.begin(DateTime(epoch));
    // wait ten seconds before asking for the time again
    delay(10000);
    Ethernet.maintain();
  }
  DateTime now = rtc.now();
  st.unix_time = now.unixtime();
}

void loop() {
  /*Comprobacion de conexion con el cliente*/
  EthernetClient cliente_s = stell.available();
  int flag_datos = 0;
  if (cliente_s) {

    while (cliente_s.connected()) {
      int i = 0;
      while (cliente_s.available()) {
        uint8_t code_received = cliente_s.read();
        data[i] = code_received;
        i = i + 1;
        flag_datos = 1;
      }
      if (flag_datos == 1) {
        Serial.println("COORDENADAS RECIBIDAS");

        /*Reorganizo los datos (BigEndian LitleEndian)*/
        dec = 0x00000000 | (long (data[19]) << 24) | (long (data[18]) << 16) | (long (data[17]) << 8) | (long (data[16]) << 0);
        RA  = 0x00000000 | (long (data[15]) << 24) | (long (data[14]) << 16) | (long (data[13]) << 8) | (long (data[12]) << 0);
        // declinacion en angulo
        //ra en angulo
        // RA12HS -- 12hs
        unsigned long int hsra = (((RA * 12.0) / RA12HS) * 10000) * 15   ;
        float hsrf  = ((RA * 12.0) / RA12HS) * 15 ;
        float declinacion = (90.0 / DEC10) * dec ;
        Serial.print("\tdeclinacion: ") ; Serial.println(declinacion, 6) ;
        Serial.print("\tascencion recta: ") ; Serial.println(hsrf, 6) ;
        //Serial.print(" ascencion recta entera: ") ; Serial.println(hsra) ;
        st.length = sizeof(st);
        st.unix_time = (int64_t)now.unixtime();
        st.dec = (90.0 / DEC10) * dec ;
        //Serial.print("dec:");
        //Serial.print(st.dec);
        st.ra  = ((RA * 12.0) / RA12HS) * 15   ;
        //Serial.print("\nra:");
        //Serial.print(st.ra);
        st.status = 0;
        cliente_s.write((byte*)&st, sizeof(st));  //Envio al stellarium la posicion de la antena
        delay(1000);
        /*Obtengo el tiempo GMT*/
        DateTime now = rtc.now();
        uint16_t anio = now.year();
        uint8_t mes  = now.month();
        uint8_t dia  = now.day();
        uint8_t hora = now.hour();
        uint8_t minuto = now.minute();
        uint8_t segundo = now.second() ;
        formulaLibroOrb(anio, mes, dia, hora, minuto, segundo) ;
        flag_datos = 0;
      }
    }
  }
}
void sendNTPpacket(const char* address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123);  // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


void formulaLibroOrb(uint16_t year, uint8_t mes, uint8_t dia, uint8_t hora, uint8_t min, uint8_t seg)
{
  //JD = J0 + UT/24 (FRACCIÓN DEL DIA )

  int hh, mm   ;
  float ss, ha ;
  float az, alt ;
  double sidereal_time ;
  const double k_calc_jd = 1721013.5 ;
  double jul_cent ;
  double j0 = 367.0 * year + (float )int((275 * mes) / 9.0) + (float )dia + k_calc_jd ;
  double ut_24 = (float) hora +  (float)min / 60.0  + (float)seg / 3600.0 ;
  //segundo término
  double term_neg = (float )int( ((float )7.0 * ((float )year  + int ((mes + 9) / 12.0))) / 4.0 );
  j0 = j0 - term_neg ;
  double jd = j0  + ut_24 / 24.0; //+(double) ut_24/24.0 ;
  jul_cent = (j0 - 2451545) / 36525 ;
  sidereal_time = 100.4606184 + 36000.7700 * jul_cent + 0.000387933 * pow(jul_cent, 2)
                  - 2.583E-8 * pow(jul_cent, 3) ;
  if (sidereal_time > 360.0) {
    sidereal_time = sidereal_time - (double) int(sidereal_time / 360.0) * 360.0 ;
  }
  sidereal_time = sidereal_time + 360.98564724 * (ut_24 / 24.0);
  if (sidereal_time > 360.0) {
    sidereal_time = sidereal_time - (double) int(sidereal_time / 360.0) * 360.0 ;
  }
  sidereal_time = sidereal_time + LONGITUD ;
  if (sidereal_time > 360.0) {
    sidereal_time = sidereal_time - (double) int(sidereal_time / 360.0) * 360.0 ;
  } else if (sidereal_time < 0) {
    sidereal_time = sidereal_time + 360.0 ;
  }
  // obtener angulo horario HA
  sidereal_time = sidereal_time - 138.297458  ;
  ha = sidereal_time ;
  if (sidereal_time > 360.0) {
    sidereal_time = sidereal_time - (double) int(sidereal_time / 360.0) * 360.0 ;
  } else if (sidereal_time < 0) {
    sidereal_time = sidereal_time + 360.000000 ;
  }
  // local sidereal time in angle
  //Serial.print("  st_g:  ") ; Serial.print(sidereal_time,6) ;
  //hh, mm y float ss
  // (sidereal_time*24)/360.0 --> convierte a horas !
  hh = int ((sidereal_time * 24) / 360.0 ) ; // parte entera de hora
  // calcula los minutos
  mm = (int)  ((((sidereal_time * 24) / 360.0) - (float)hh) * 60.0)      ;
  ss = ((((sidereal_time * 24) / 360.0) - hh) * 60.0 - (float) mm) * 60.0  ;
  //  Serial.println(year);
  //  Serial.println(mes);
  //  Serial.println(dia);
  //  Serial.println(hora);
  //  Serial.println(min);
  //  Serial.println(seg);
  Serial.print("TRANSFORMACION COORDENADAS\n");
  Serial.print("\tha: ") ;
  Serial.print(hh)   ;  Serial.print(":") ;
  Serial.print(mm)   ;  Serial.print(":") ;
  Serial.println(ss, 6) ;
  transformEc2H(ha, dec, &az, &alt ) ;
}

/*Transformacion de coordenadas ecuatoriales locales a horizontales*/
void transformEc2H(float ha , float dec, float *az, float *alt)
{
  //cos(z) = sin(phi)
  float angle_offset = 0 ;
  // K1 IS SIGN OF COS, K2 IS SIGN OF K2

  *alt = sin(dec * DEG_TO_RAD) * sin(LATITUD * DEG_TO_RAD)
         + cos(LATITUD * DEG_TO_RAD) * cos(dec * DEG_TO_RAD) * cos(ha * DEG_TO_RAD) ;
  *alt = acos(*alt) * RAD_TO_DEG ;
  // elige cuadrante de azimut
  float k1 = -cos(LATITUD * DEG_TO_RAD) * sin(dec * DEG_TO_RAD)
             + sin(LATITUD * DEG_TO_RAD) * cos(dec * DEG_TO_RAD) * cos(ha * DEG_TO_RAD) ;
  k1 = k1 / sin(*alt * DEG_TO_RAD) ;
  float k2 = cos(dec * DEG_TO_RAD) * sin(ha * DEG_TO_RAD) ;
  k2 = k2 / sin(*alt * DEG_TO_RAD) ;

  *alt = 90.0 - *alt ;
  Serial.print("\tcuadrante: ") ;
  Serial.print(k1)  ; Serial.print("  ") ;
  Serial.print(k2) ;

  if (k1 > 0 && k2 > 0) { // primer cuadrante
    Serial.println("1") ;
    angle_offset = 0 ;
  } else if (k1 < 0 && k2 > 0) { // segundo cuadrante
    Serial.println("2") ;

    angle_offset = 180.0 ;
  } else if (k1 < 0 && k2 < 0) { // tercer cuadrante
    Serial.println("3") ;

    angle_offset = 270.0 ;
  } else if (k1 > 0 && k2 < 0) { // cuarto cuadrante
    Serial.println("  4") ;
    angle_offset = 360.0 ;
  }
  *az = atan(k2 / k1) * RAD_TO_DEG  + angle_offset;
  *az = -180.0 +  *az;
  Serial.print("\taz/alt ") ; Serial.print(*az) ; Serial.print("/") ; Serial.println(*alt) ;
  Serial.print("------------------------------------\n");



}
