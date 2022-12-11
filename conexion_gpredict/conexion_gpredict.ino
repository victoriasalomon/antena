#include <Ethernet.h>
#include <SPI.h>
#include <Servo.h>

#define PINSS 4
#define PINRESET 3
#define PORT_GPREDICT 1024
EthernetServer gpr(PORT_GPREDICT);

Servo altitud;
Servo azimut;
int posicion_az = 90;
int posicion_alt = 90;
int new_az;
int new_alt;
char data[50];
int i = 0;
String data_str, data_az, data_alt;

void setup() {

  //Configuracion de pines servos
  altitud.attach(9);
  azimut.attach(10);

  //Inicia en posicion cenit
  altitud.write(90);
  azimut.write(90);

  Serial.begin(9600);
  Ethernet.init(PINSS);
  byte mac[] = { 0xBC, 0xCB, 0xAF, 0xAB, 0xAA, 0x01 };  // dirmac
  if (Ethernet.begin(mac) == 0) {

    Serial.print(F("Fallo DHCP"));

  } else {
    Serial.println(Ethernet.localIP());
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  EthernetClient cliente_g = gpr.available();
  if (cliente_g) {


    while (cliente_g.connected()) {

      if (cliente_g.available()) {
        {
          char c = cliente_g.read() ;
          Serial.println(c);
          if (c == 'p') {
            String alt = String (posicion_alt);
            String az = String (posicion_az);
            String pos = alt + "\n" + az + "°\n";
            //cliente_g.print("130.0\n 39°\n");
            cliente_g.print(pos);
            cliente_g.flush();
          }
          else if (c == 'P') {
            i = 0;
            while (i < 12) {
              data[i] = (char)cliente_g.read();
              Serial.print(data[i]);
              i++;
            }
            Serial.println("---");
            data_str = data;
            data_az = data_str.substring(1, 6);
            new_az = data_az.toInt();
            Serial.println(new_az);
            data_alt = data_str.substring(8, 12);
            new_alt = data_alt.toInt();
            Serial.println(new_alt);
            posicion_alt = new_alt;
            posicion_az = new_az;
            
          }
        }
      }
    }
  }
}
