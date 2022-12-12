/*Librerias utilizadas en el programa*/
#include <Ethernet.h>
#include <SPI.h>
#include <Servo.h>

/*Declaracion de pines del modulo Ethernet*/
#define PINSS 4
#define PINRESET 3

/*Creacion del objeto servidor */
#define PORT_GPREDICT 1024
EthernetServer gpr(PORT_GPREDICT);

/*Motores para cada eje de la antena*/
Servo altitud;
Servo azimut;
int posicion_az = 90;
int posicion_alt = 90;
int new_az;
int new_alt;

char data[50];  //Vector para los datos recibidos
int i = 0;
String data_str, data_az, data_alt;

void setup() {

  /*Configuracion de pines servos*/
  altitud.attach(9);
  azimut.attach(10);

  /*Inicia en posicion cenit*/
  altitud.write(90);
  azimut.write(90);
  /*Inicializacion de ethernet*/
  Serial.begin(9600);
  Ethernet.init(PINSS);
  byte mac[] = { 0xBC, 0xCB, 0xAF, 0xAB, 0xAA, 0x01 };  // dirmac
  /*Obtencion de IP por DHCP*/
  if (Ethernet.begin(mac) == 0) {

    Serial.print(F("Fallo DHCP"));

  } else {
    Serial.println(Ethernet.localIP());
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  /*Comprobacion de conexion con el cliente*/
  EthernetClient cliente_g = gpr.available();
  if (cliente_g) {


    while (cliente_g.connected()) {

      while (cliente_g.available() > 0)
      {
        char c = cliente_g.read() ;
        Serial.print(c);
        /*Comando 'p' para obtener posicion de la antena*/
        if (c == 'p') {
          String alt = String (posicion_alt);
          String az = String (posicion_az);
          String pos = az + "\n" + alt + "°\n";
          //cliente_g.print("130.0\n 39°\n");
          cliente_g.print(pos);
          cliente_g.flush();
        }
        /*Comando 'P' para enviar posicion a la antena*/
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
          data_alt = data_str.substring(7, 12);
          new_alt = data_alt.toInt();
          Serial.println(new_alt);
          posicion_alt = new_alt;
          posicion_az = new_az;
          altitud.write(posicion_alt);
          azimut.write(posicion_az);
          cliente_g.print("RPRT 0");    //Lectura correcta
        }
        /*Comando 'q' para cerrar la conexion*/
        else if (c == 'q') {
          cliente_g.flush();
          cliente_g.stop();
        }
        /*Comando 'S' para parar el rotador*/
        else if (c == 'S') {
          cliente_g.flush();
          cliente_g.stop();
        }

      }
    }
    cliente_g.stop();
    delay(10);
    /*Muevo la antena a la posicion cenit*/
    altitud.write(90);
    azimut.write(90);
  }
}
