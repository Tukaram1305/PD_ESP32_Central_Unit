#ifndef WIFICREDENTIALS_H
#define WIFICREDENTIALS_H
//  Parametry lokalnej sieci WWIFI
const char* ssid = "ssid";
const char* password =  "pass";
// spobuje ustawic statyczne IP bazy(ESP32)
IPAddress local_IP(192, 168, 1, 35);
IPAddress gateway(192, 168, 1 ,1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4);

//  Dane uwierzytelnienia kanalu na serwerze ThingSpeak
unsigned long myChannelNumber = 1896531;
const char * myWriteAPIKey = "xxkeyxx";

//  Adres lokalnego serwera WWW na maszynie stacjonarnej
String ServerAddress = "192.168.1.55/smartesp";

#endif
