#include <Ethernet.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <RTClib.h>

#define PINSS 4
#define PINRESET 3
#define PORT_STELLARIUM 10000
#define PORT_GPREDICT 1024
EthernetServer stell(PORT_STELLARIUM);

unsigned long int RA12HS = 0x80000000;
long int DEC10 = 0x40000000;
long int dec;
unsigned long int RA = 0;

typedef struct {
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

void setup() {

  Serial.begin(57600);
  Ethernet.init(PINSS);
  byte mac[] = { 0xBC, 0xCB, 0xAF, 0xAB, 0xAA, 0x01 };  // dirmac
  if (Ethernet.begin(mac) == 0) {

    Serial.print(F("Fallo DHCP"));

  } else {
    Serial.println(Ethernet.localIP());
  }

  Udp.begin(localPort);

  sendNTPpacket(timeServer);  // send an NTP packet to a time server

  // wait to see if a reply is available
  delay(1000);
  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    rtc.begin(DateTime(epoch));
    // wait ten seconds before asking for the time again
    delay(10000);
    Ethernet.maintain();
  }
  DateTime now = rtc.now();
  st.unix_time = now.unixtime();
}

void loop() {
  // put your main code here, to run repeatedly:
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
        //Serial.print(code_received);Serial.println(code_received,HEX);
      }
      if (flag_datos == 1) {
        Serial.println("llegaron los datos!");
        //Serial.println(unixtime());
        dec = 0x00000000 | (long (data[19])<<24) | (long (data[18])<<16) | (long (data[17])<<8) | (long (data[16])<<0);
        RA  = 0x00000000 | (long (data[15])<<24) | (long (data[14])<<16) | (long (data[13])<<8) | (long (data[12])<<0);
        // declinacion en angulo 
        //ra en angulo 
        // RA12HS -- 12hs 
        unsigned long int hsra = (((RA*12.0)/RA12HS) * 10000)*15   ;
        float hsrf  = ((RA*12.0)/RA12HS)*15 ;      
        float declinacion = (90.0/DEC10)*dec ;  
        Serial.print("declinacion: ") ; Serial.println(declinacion,6) ; 
        Serial.print("ascencion recta: ") ; Serial.println(hsrf,6) ; 
        Serial.print(" ascencion recta entera: ") ; Serial.println(hsra) ;        
        st.length = sizeof(st);
        st.unix_time = (int64_t)now.unixtime();
        st.dec = (90.0/DEC10)*dec ;
        Serial.print("dec:");
        Serial.print(st.dec);
        //st.dec = 0x40000000;
        st.ra  = ((RA*12.0)/RA12HS) *15   ;
        Serial.print("\nra:");
        Serial.print(st.ra);
        //st.ra = 0x80000000;
        st.status = 0;
        cliente_s.write((byte*)&st, sizeof(st));
        delay(1000);
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
