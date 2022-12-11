#include <Ethernet.h>
#include <SPI.h>
#include <ArduinoJson.hpp>
#include <ArduinoJson.h>

#define PINSS 4
#define PINRESET 3
#define PORT_INFO 15000
char jsonp[30];
int i = 0;
EthernetServer pagina(PORT_INFO);


void setup()
{
  Serial.begin(9600);
  Ethernet.init(PINSS);
  byte mac[] = { 0x00, 0xCD, 0xEF, 0xEE, 0xAA, 0xBC };  // dirmac
  if (Ethernet.begin(mac) == 0) {

    Serial.print(F("Fallo DHCP"));

  } else {
    Serial.println(Ethernet.localIP());
  }

}

void loop()
{
  EthernetClient client = pagina.available();
  if (client) {

    while (client.connected()) {

      if (client.available()) {
        {
          //Serial.print((char)client.read());
          jsonp [i] = ((char)client.read());
          i++;
          String json = jsonp;
          StaticJsonDocument<300> doc;
          DeserializationError error = deserializeJson(doc, json);
          if (error) {
            return;
          }
          Serial.println(jsonp);

          int azimuth = doc["x"];
          int altitud = doc["y"];

          Serial.println(azimuth);
          Serial.println(altitud);

        }

      }
    }
    Serial.println("Cliente desconectado");
  }
}
