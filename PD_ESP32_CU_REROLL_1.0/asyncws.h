#ifndef ASYNCWS_H
#define ASYNCWS_H
// Asynchroniczny WEBSERWER
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
    Serial.println("GET: podaje index.html");
  });

  server.on("/styl.css", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/styl.css","text/css");
  Serial.println("GET: podaje styl.css");
  });

    // Ilosc zmagazynowanych danych
  server.on("/getDataNums", HTTP_GET, [] (AsyncWebServerRequest *request) {
    uint16_t DNums = getDataNums(SPIFFS, "/dane.txt");
    request->send(200, "text/plain", String(DNums).c_str());
    //Serial.println("GET: podaje Sile sygnalu WiFi");
  });
  
  // Sprawdz status (czy ESP online) i sprobuj wyslac dane (jezeli sa)
  server.on("/espStatus", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String("ESP-ONLINE").c_str());
    // sprawdz czy sa dane do wyslania - ustaw flage
    dataReadyToPOST = true;
    Serial.println("GET: podaje STATUS_ESP32");
  });

  // Pobierz aktualne dane
  server.on("/getCurrData", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String dateTime = getNTPTime();
    getSensorsData();
    form = "{\"T\":"+String(sensorsData.AHT_TEMP,2)+
            ",\"C\":"+String(sensorsData.BMP_PRESS,2)+
            ",\"H\":"+String(sensorsData.AHT_HUM,2)+
            ",\"D\":"+"\""+dateTime+"\""+"}"+"\n";
            
    request->send(200, "application/json",form.c_str());
    Serial.println("GET: podaje aktualne dane T/C/W");
  });

  // Pobierz aktualne dane CO (kotlownia)
  server.on("/getCOData", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("BR_T")) 
    {
      String buf = request->getParam("BR_T")->value();
      Serial.println("GET Param -> "+buf);
      BRsensorsData.TempCO = buf.toFloat();
    }
    if (request->hasParam("BR_G")) 
    {
      String buf = request->getParam("BR_G")->value();
      Serial.println("GET Param -> "+buf);
      BRsensorsData.GasAnalog = buf.toInt();
      // Modul CO odezwal sie, resetuj czas alarmu powiadomienia
      CO_timer = millis();
    }
    String dateTime = getNTPTime();
    String dJSON = "{\"TCO\":"+String(BRsensorsData.TempCO,1)+
                    ",\"GAS\":"+String(BRsensorsData.GasAnalog)+
                    ",\"D\":"+"\""+dateTime+"\""+"}"+"\n";
    request->send(200, "application/json",dJSON.c_str());
  });
  
// Switch 1 / przekaznik/kontrola
  server.on("/getSW1", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("input1")) 
    {
      inputMessage = request->getParam("input1")->value();
      (inputMessage=="true") ? SWITCH_1_STATE=1 : SWITCH_1_STATE=0;
      Serial.println("GET: "+String(inputMessage)+" Przelacznik_1: "+String(SWITCH_1_STATE));
      esp_err_t result = esp_now_send(m1switch220, (uint8_t *) &SWITCH_1_STATE, sizeof(SWITCH_1_STATE));
      if (result == ESP_OK) {request->send(200, "text/plain", String("200").c_str());} // OK - modul odpowiada
      else {request->send(200, "text/plain", String("400").c_str());}  
    }
    else request->send(200, "text/plain", String(!SWITCH_1_STATE).c_str());
    });

// Tasma LED - 1 kolor White
  server.on("/getONE_LED_W1", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("input1")) 
    {
      inputMessage = request->getParam("input1")->value();
      OneWayLED_W1 = inputMessage.toInt();
      Serial.println("GET: "+String(inputMessage)+" Tasma LED ciepla: "+String(OneWayLED_W1));
      ////NOWE 11.2022
      esp_err_t result = esp_now_send(m2oneLEDstrip, (uint8_t *) &OneWayLED_W1, sizeof(OneWayLED_W1));
      if (result == ESP_OK) {request->send(200, "text/plain", String("200_ONELED_OK").c_str());} // OK - modul odpowiada
      else {request->send(200, "text/plain", String("404").c_str());}                  // BLAD - modul nie odpowiada
   }
    });

// Czas odswierzania (do bazy danych) / modyfikacja odswiezania
  server.on("/getSetRefresh", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("input1")) 
    {
      inputMessage = request->getParam("input1")->value();
      TCW_delay = inputMessage.toInt();
      Serial.println("Zmodyfikowano czas odswierzania do DB: "+inputMessage+" min.");
    }
    request->send(200, "text/plain", String(TCW_delay).c_str());
    });

// Czas odswierzania (do serw.ThingSpeak) / modyfikacja odswiezania
  server.on("/getSetTSRefresh", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("input1")) 
    {
      inputMessage = request->getParam("input1")->value();
      ThgSpk_delay = inputMessage.toInt();
      Serial.println("Zmodyfikowano czas wysylania do TS: "+inputMessage+" min.");
    }
    request->send(200, "text/plain", String(ThgSpk_delay).c_str());
    });
    
// Oswietlenie LED #1 ( Tasma RGB - witty )
  server.on("/getLED1", HTTP_GET, [] (AsyncWebServerRequest *request) {

    if (request->hasParam("Red")) 
    {
      inputMessage = request->getParam("Red")->value();
      rgbLedVals.r = inputMessage.toInt();
    }
    if (request->hasParam("Green")) 
    {
      inputMessage = request->getParam("Green")->value();
      rgbLedVals.g = inputMessage.toInt();
    }
    if (request->hasParam("Blue")) 
    {
      inputMessage = request->getParam("Blue")->value();
      rgbLedVals.b = inputMessage.toInt();
    }
      Serial.println("GET: RGB LED#1: R:"+String(rgbLedVals.r)+" G:"+String(rgbLedVals.g)+" B:"+String(rgbLedVals.b));
      esp_err_t result = esp_now_send(m3RGBLEDstrip, (uint8_t *) &rgbLedVals, sizeof(rgbLedVals));
      if (result == ESP_OK) {request->send(200, "text/plain", String("200_RGB_OK").c_str());} // OK - modul odpowiada
      else {request->send(200, "text/plain", String("404").c_str());} 
    });

// Oswietlenie TASMA #1
  server.on("/getLEDs1", HTTP_GET, [] (AsyncWebServerRequest *request) {

    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("Red")) 
    {
      inputMessage = request->getParam("Red")->value();
      LED_1_R = inputMessage.toInt();
    }
    if (request->hasParam("Green")) 
    {
      inputMessage = request->getParam("Green")->value();
      LED_1_G = inputMessage.toInt();
    }
    if (request->hasParam("Blue")) 
    {
      inputMessage = request->getParam("Blue")->value();
      LED_1_B = inputMessage.toInt();
    }
      Serial.println(" TASMA#1/2/3: R:"+String(LED_1_R)+" G:"+String(LED_1_G)+" B:"+String(LED_1_B));
      // input na radio
      RadioLED[0] = static_cast<byte>(LED_1_R);
      RadioLED[1] = static_cast<byte>(LED_1_G);
      RadioLED[2] = static_cast<byte>(LED_1_B);
      bool report = radio.write(&RadioLED, sizeof(RadioLED));

      if (report) 
      {
        request->send(200, "text/plain", String("RGB-R [OK]").c_str());
      }
      else request->send(200, "text/plain", String("RADIO [ERR]").c_str());
    });

// Oswietlenie TASMA #2
  server.on("/getLEDs2", HTTP_GET, [] (AsyncWebServerRequest *request) {

    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("Red")) 
    {
      inputMessage = request->getParam("Red")->value();
      LED_1_R = inputMessage.toInt();
    }
    if (request->hasParam("Green")) 
    {
      inputMessage = request->getParam("Green")->value();
      LED_1_G = inputMessage.toInt();
    }
    if (request->hasParam("Blue")) 
    {
      inputMessage = request->getParam("Blue")->value();
      LED_1_B = inputMessage.toInt();
    }
      Serial.println(" TASMA#1/2/3: R:"+String(LED_1_R)+" G:"+String(LED_1_G)+" B:"+String(LED_1_B));
      // input na radio
      RadioLED[3] = static_cast<byte>(LED_1_R);
      RadioLED[4] = static_cast<byte>(LED_1_G);
      RadioLED[5] = static_cast<byte>(LED_1_B);
      bool report = radio.write(&RadioLED, sizeof(RadioLED));

      if (report) 
      {
        request->send(200, "text/plain", String("RGB-R [OK]").c_str());
      }
      else request->send(200, "text/plain", String("RADIO [ERR]").c_str());
    });

// Oswietlenie TASMA #3
  server.on("/getLEDs3", HTTP_GET, [] (AsyncWebServerRequest *request) {

    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam("Red")) 
    {
      inputMessage = request->getParam("Red")->value();
      LED_1_R = inputMessage.toInt();
    }
    if (request->hasParam("Green")) 
    {
      inputMessage = request->getParam("Green")->value();
      LED_1_G = inputMessage.toInt();
    }
    if (request->hasParam("Blue")) 
    {
      inputMessage = request->getParam("Blue")->value();
      LED_1_B = inputMessage.toInt();
    }
      Serial.println(" TASMA#1/2/3: R:"+String(LED_1_R)+" G:"+String(LED_1_G)+" B:"+String(LED_1_B));
      // input na radio
      RadioLED[6] = static_cast<byte>(LED_1_R);
      RadioLED[7] = static_cast<byte>(LED_1_G);
      RadioLED[8] = static_cast<byte>(LED_1_B);
      bool report = radio.write(&RadioLED, sizeof(RadioLED));

      if (report) 
      {
        request->send(200, "text/plain", String("RGB-R [OK]").c_str());
      }
      else request->send(200, "text/plain", String("RADIO [ERR]").c_str());
    });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();

#endif
