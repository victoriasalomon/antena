/*Librerias utilizadas en el programa*/
#include <Ethernet.h>
#include <SPI.h>
#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <Servo.h>

/*Declaracion de pines ethernet*/
#define PINSS 4
#define PINRESET 3

/*Creacion del objeto servidor*/
#define PORT_INFO 15000
EthernetServer pagina(PORT_INFO);

/*Motores para cada eje de la antena*/
Servo altitud;
Servo azimut;

int az = 90;
int alt = 90;

char jsonp[30];   //Vector de datos recibidos
int i = 0;



void setup()
{
  /*Inicializacion de ethernet*/
  Serial.begin(9600);
  Ethernet.init(PINSS);
  byte mac[] = { 0x10, 0x01, 0x12, 0x01, 0x08, 0xDC };  // dirmac

  /*Obtencion de IP por DHCP*/
  if (Ethernet.begin(mac) == 0) {

    Serial.print(F("Fallo DHCP"));

  } else {
    Serial.println(Ethernet.localIP());
  }

}

void loop()
{
  /*Compruebo la conexion con el cliente*/

  EthernetClient client = pagina.available();
  if (client)
  {

    while (client.connected())
    {

      while (client.available() > 0)
      {
        jsonp [i] = ((char)client.read());
        i++;
      }
      client.write("Recibido");
      //client.write("{\"x\": 55, \"y\": 60}\"",1024)  ;
      i = 0 ;
      String json = jsonp;
      Serial.print("json: "); Serial.println(jsonp);

      StaticJsonDocument<100> doc;
      DeserializationError error = deserializeJson(doc, json);
      if (error)
      {
        Serial.println("error ") ;
        continue ;
      }
      /*Guardo las coordenadas en su respectiva variable*/
      az = doc["x"];
      alt = doc["y"];
      /*Muevo la antena a la posicion recibida*/
      altitud.write(alt);
      azimut.write(az);
      Serial.print("azimut: " ) ;
      Serial.print(az) ;
      Serial.print(" altitud: " ) ;
      Serial.println(alt) ;

    }
    Serial.println("Cliente desconectado");
    client.stop()  ;
    /*Muevo la antena a la posicion cenit*/
    //alt = 90;
    //az = 90;
    //altitud.write(alt);
    //azimut.write(az);
  }
}
