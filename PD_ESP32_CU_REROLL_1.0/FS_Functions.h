#ifndef FS_FUNCTIONS_H
#define FS_FUNCTIONS_H

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Czytam plik: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- Blad przy otwieraniu pliku do odczytu!");
        return;
    }

    Serial.println("- czytam z pliku:");
    String buff = "";
    uint16_t iter = 0;
    while(file.available()){
        //Serial.write(file.read());
        buff = file.readStringUntil('\n');
        iter++;
        Serial.print(String(iter)+"------");
        Serial.println(buff);
        //if (buff=="\n") break;
        //else if(buff=="") break;
    }
    file.close();
}

uint16_t getDataNums(fs::FS &fs, const char * path){
    Serial.printf("Czytam plik %s i licze dane...\r\n", path);
    File file = fs.open(path);
    if(!file || file.isDirectory()){ Serial.println("- blad przy otwieraniu pliku!"); return 0; }
    uint16_t iter = 0;
    String buff = {};
    while(file.available())
    {
        file.readStringUntil('\n');
        iter++;
    }
    file.close();
    return iter;
}

void readAndSend(fs::FS &fs, const char * path){
    Serial.printf("Czytam plik %s i wysylam dane...\r\n", path);
    bool sended = false;
    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- blad przy otwieraniu pliku!");
        return;
    }
    String buff = "";
    uint16_t iter = 0;
    if (file.available())
    { Serial.println("Znalazlem dane do wyslania...");
    HTTPClient http;
    while(file.available()){
        // czytaj do bufora jedna linie pliku z danymi+czas i wysylaj do serwera
        buff = file.readStringUntil('\n');
        iter++;
        Serial.print("<-wysylam["+String(iter)+"]-");
        Serial.println(buff);
        // http

        http.begin("http://"+ServerAddress+"/api/espPostJSONdata.php"); //HTTP
        http.addHeader("Content-Type", "application/json");
        //http.setConnectTimeout(300);
        int httpCode = http.POST(buff);
        
        // httpCODE bedzie ujemny przy bledzie
        if(httpCode > 0) {
            // file found at server
            if(httpCode == HTTP_CODE_OK) {
              sended = true;
              String payload = http.getString();
              Serial.println("Odpowiedz: "+String(payload));
            } else {
              // HTTP header has been send and Server response header has been handled
              Serial.printf("[HTTP] GET... code: %d\n", httpCode);
            }
          } // blad HTTP
          else if(httpCode == HTTPC_ERROR_CONNECTION_REFUSED) {Serial.println("Host odrzucil polaczenie!");}
          else {Serial.printf("Blad[HTTP]GET, opis: %s\n", http.errorToString(httpCode).c_str());}

    } // while
          http.end();
    } // sa dane do wyslania
    else  Serial.println("Brak danych do wyslania");
    
    file.close();
      
    if (sended==true) 
    {
      writeFile(SPIFFS, path, "");
      String logtx = "Delete DATA file - "+getNTPTime()+"\n";
      appendFile(SPIFFS, "/log.txt", logtx.c_str());
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Zapisuje/nadpisuje plik: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- blad przy otwierniu pliku do zapisu!");
        return;
    }
    if(file.print(message)){
        Serial.println("- zapisano plik");
    } else {
        Serial.println("Plik wyczyszczony");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Dopisuje do pliku: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- blad przy otwieraniu pliku!");
        return;
    }
    if(file.print(message)){
        Serial.println("- dane dolaczone");
    } else {
        Serial.println("- blad przy dolaczaniu do pliku!");
    }
    file.close();
}

#endif
