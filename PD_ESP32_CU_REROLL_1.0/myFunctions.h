#ifndef MYFUNCTIONS_H
#define MYFUNCTIONS_H
void startWIFI()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Lacze z WiFi..");
    }
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {Serial.println("Nie udalo sie skonfigurowac WiFi STA");}

  String IP = WiFi.localIP().toString().c_str();
  int RSI = WiFi.RSSI();
  Serial.println("IP ESP: "+IP+" , sygnal: "+String(RSI));
}

void startRadio()
{
  // Radio prestart
  delay(100);
  if (radio.begin()) { Serial.println("RADIO ZGLASZA SIE"); RADIO_OK = true;}
  else { Serial.println("TOS NIE TAK Z RADIEM !!!"); RADIO_OK = false;}
  radio.openWritingPipe(addrLed); delay(5); // 3xLEDy RF
  radio.setPALevel(RF24_PA_LOW); delay(5);
  radio.stopListening(); delay(5);
  radio.setRetries(15, 15); delay(5);
  radio.powerUp(); delay(5);
  radio.setDataRate(RF24_250KBPS); delay(5);
  radio.enableDynamicPayloads();
}

String getNTPTime()
{
  // Czas z NTP
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime();
  //Get a time structure
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;
  //konkatenacja daty i czasu (SQL format)
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);
  //Serial.println("CzasNTP: "+currentDate+" "+formattedTime);
  String dateTime = currentDate+" "+formattedTime;
  return dateTime;
}

void alarm(int sigOnMs, int sigOffMs)
{
  if (sigOnMs==0 && sigOffMs==0) digitalWrite(BUZZER_PIN,HIGH);
  else{
    bool state = digitalRead(BUZZER_PIN);
    if (state == true && (millis() > alarmTimerMs+sigOffMs) )
    {
      digitalWrite(BUZZER_PIN,LOW);
      alarmTimerMs=millis();
    }
    else if (state == false && (millis() > alarmTimerMs+sigOnMs) )
    {
      digitalWrite(BUZZER_PIN,HIGH);
      alarmTimerMs=millis();
    }
  }
}

bool validateData()
{
  bool valid = true;
  if (sensorsData.BMP_TEMP < T_L_LIM || sensorsData.BMP_TEMP > T_U_LIM) valid = false;
  if (sensorsData.BMP_PRESS < P_L_LIM || sensorsData.BMP_PRESS > P_U_LIM) valid = false;
  if (sensorsData.AHT_TEMP < T_L_LIM || sensorsData.AHT_TEMP > T_U_LIM) valid = false;
  if (sensorsData.AHT_HUM < H_L_LIM || sensorsData.AHT_HUM > H_U_LIM) valid = false;
  return valid;
}

void getSensorsData()
{
  // Zbieranie danych z czujników
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print("Temp.BMP280 = ");
  sensorsData.BMP_TEMP = temp_event.temperature;
  Serial.println(String(sensorsData.BMP_TEMP)+" *C");

  Serial.print("Press.BMP280 = ");
  sensorsData.BMP_PRESS = pressure_event.pressure;
  Serial.println(String(sensorsData.BMP_PRESS)+" hPa");

  sensorsData.AHT_TEMP = aht10.readTemperature();
  sensorsData.AHT_HUM = aht10.readHumidity();
  Serial.println("AHT Temp: "+String(sensorsData.AHT_TEMP,2));
  Serial.println("AHT Hum: "+String(sensorsData.AHT_HUM,2));  

  if (!validateData())
  {
    Serial.println("BLEDNE ODCZYTY! Sprawdzam ponownie...");
    delay(100);
    getSensorsData();
  }
}

bool ESPNOW_INIT()
{
  if (esp_now_init() != ESP_OK) { return false; }
  esp_now_register_send_cb(OnDataSent);
  // Zarejestroj peer'y
  memcpy(m1switch220PeerInf.peer_addr, m1switch220, 6);
  memcpy(m2oneLEDstripPeerInf.peer_addr, m2oneLEDstrip, 6);
  memcpy(m3RGBLEDstripPeerInf.peer_addr, m3RGBLEDstrip, 6);
  m1switch220PeerInf.channel = 0;  
  m1switch220PeerInf.encrypt = false;
  m2oneLEDstripPeerInf.channel = 0;  
  m2oneLEDstripPeerInf.encrypt = false;
  m3RGBLEDstripPeerInf.channel = 0;
  m3RGBLEDstripPeerInf.encrypt = false;
    // dodaj peer 1       
  if (esp_now_add_peer(&m1switch220PeerInf) != ESP_OK){
    Serial.println("Nie udalo sie dodac peer'a!"); return false; }
  // dodaj peer 2       
  if (esp_now_add_peer(&m2oneLEDstripPeerInf) != ESP_OK){
    Serial.println("Nie udalo sie dodac peer'a!"); return false; }
  // dodaj peer 3       
  if (esp_now_add_peer(&m3RGBLEDstripPeerInf) != ESP_OK){
    Serial.println("Nie udalo sie dodac peer'a!"); return false; }
  return true;
}

void GATHER_THP(const uint16_t DELAY)
{
  if (millis() > TCW_timer+(DELAY*MSnaMin)) {
  TCW_timer=millis();
  String dateTime = getNTPTime();
  getSensorsData();
  HTTPClient http;
  form = "{\"T\":"+String(sensorsData.AHT_TEMP,2)+
          ",\"C\":"+String(sensorsData.BMP_PRESS,2)+
          ",\"H\":"+String(sensorsData.AHT_HUM,2)+
          ",\"D\":"+"\""+dateTime+"\""+"}"+"\n";
  
  http.begin("http://"+ServerAddress+"/api/espPostJSONdata.php"); //HTTP
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(200);
  int httpCode = http.POST(form);
  String payload;

  if(httpCode > 0) {
    if(httpCode == HTTP_CODE_OK) {
      payload = http.getString();
      Serial.println("Wyslano, odpowiedz: "+String(payload));
      readAndSend(SPIFFS, "/dane.txt");
    } else {
      Serial.printf("[HTTP] GET: %d, zapisuje w SPIFFS...\n", httpCode);
      appendFile(SPIFFS, "/dane.txt", form.c_str());
    }
  }
  else {
    Serial.printf("Blad[HTTP]GET, opis: %s, zapisuje w SPIFFS...\n", http.errorToString(httpCode).c_str());
    appendFile(SPIFFS, "/dane.txt", form.c_str());
    }
  http.end(); } }

void POST_THINGSPEAK(const uint16_t DELAY)
{
  if (millis()>ThgSpk_timer+(DELAY*MSnaMin))
  {
  ThgSpk_timer = millis();
  getSensorsData();
  
  // ustaw pola
  ThingSpeak.setField(1, sensorsData.AHT_TEMP);
  ThingSpeak.setField(2, sensorsData.BMP_PRESS);
  ThingSpeak.setField(3, sensorsData.AHT_HUM);
  ThingSpeak.setField(4, getDataNums(SPIFFS, "/dane.txt"));
  ThingSpeak.setField(5, BRsensorsData.TempCO);
  
  int code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(code == 200){ Serial.println("Zaaktualizowano kanal ThingSpeak"); }
  else{ Serial.println("Blad podczas aktualizicja kanalu ThingSpeak!"); }
  } // minol zadany czas
}

void GAS_SAFETY_HANDLER()
{
GAS_LEVEL gas_level = SENSOR_PROBLEM;

if (millis() > CO_timer+CO_FAILSAFE)
{
  BRsensorsData.TempCO = 0;
  BRsensorsData.GasAnalog = 0;
  Serial.println("Brak odpowiedzi od czujnika CO!");
  CO_timer = millis();
  gas_level = SENSOR_PROBLEM;
}
if (BRsensorsData.GasAnalog > 0 && BRsensorsData.GasAnalog <= GAS_TRESHOLD) gas_level = GL_OK;
else if (BRsensorsData.GasAnalog > GAS_TRESHOLD && BRsensorsData.GasAnalog <= GAS_TRESHOLD+40) gas_level = GL_LOW;
else if (BRsensorsData.GasAnalog > GAS_TRESHOLD+40 && BRsensorsData.GasAnalog <= GAS_TRESHOLD+80) gas_level = GL_MEDIUM;
else if (BRsensorsData.GasAnalog > GAS_TRESHOLD+80) gas_level = GL_HIGH;

switch(gas_level)
{
  case GL_OK:           {alarm(0,0);      break;}
  case SENSOR_PROBLEM:  {alarm(100,1900); break;}
  case GL_LOW:          {alarm(250,750);  break;}
  case GL_MEDIUM:       {alarm(200,400);  break;}
  case GL_HIGH:         {alarm(200,200);  break;}
  default: break;
}

}

void RFTest(byte sig)
{
  byte percent = 0;
  Serial.println("RADIO STRESS TEST--------");
  
  radio.openWritingPipe(addrLed); delay(5); // 3xLEDy RF
  switch(sig)
  {
    case 0:
    {
      radio.setPALevel(RF24_PA_MIN); delay(5);
      Serial.println("PA LEVEL: MIN");
      break;
    }
    case 1:
    {
      radio.setPALevel(RF24_PA_LOW); delay(5);
      Serial.println("PA LEVEL: LOW");
      break;
    }
    case 2:
    {
      radio.setPALevel(RF24_PA_HIGH); delay(5);
      Serial.println("PA LEVEL: HIGH");
      break;
    }
    case 3:
    {
      radio.setPALevel(RF24_PA_MAX); delay(5);
      Serial.println("PA LEVEL: MAX");
      break;
    }
    default:
    {
      radio.setPALevel(RF24_PA_LOW); delay(5);
      Serial.println("PA LEVEL: LOW (default)");
      break;
    }
  }
  
  radio.stopListening(); delay(5);
  radio.setRetries(5, 0); delay(5);
  radio.powerUp(); delay(5);
  radio.setDataRate(RF24_250KBPS); delay(5);
  radio.enableDynamicPayloads();
  if(radio.isPVariant()) Serial.println("NRF+");
  else Serial.println("NRF bez PA GAIN");
  bool report;
  for (byte i=0; i<9; i++) RadioLED[i]=0xAA;

  Serial.print("[");
  for (byte i=0; i<100; i++)
  {
    report = radio.write(&RadioLED, sizeof(RadioLED));
    delay(5);
    if (report==true) {percent++; Serial.print("+");}
    else Serial.print("-");
  }
  Serial.println("]");
  Serial.println("Sila sygnalu: "+String(percent)+" %");
  delay(1000);
  startRadio();
}
#endif
