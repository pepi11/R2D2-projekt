#include "dane_wifi.h"  // pass i hasło wifi
#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include "Arduino.h"
#include <Wire.h>
#include <ESP32Time.h>
#include <WiFi.h>
#include "Audio.h"  // usunieto w biliotece Sd.h
#include <Preferences.h>
#include <WebSocketsServer.h>
// wi fi dane - bez pliku #include "dane_wifi.h" odremować
//const char* ssid = "nazwa sieci";
//onst char* password = "hasło";

//----------------------------------

// audio blok
//-----------------------------------
#define I2S_DOUT 25
#define I2S_BCLK 33
#define I2S_LRC 26
Audio audio;
byte volume = 14;
byte koniec_mp3 = 0;
String gramy_mp3;
int nr_mp3 = 1;
String buffer = "";
//-----------------------------------
//----------------------------
//blok karty SD
//----------------------------
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCLK 18
#define SD_CS 5
File plik;
//----------------------------
// lan socket
WebSocketsServer webSocket = WebSocketsServer(81);  // port 81
// init sd card
bool initSDCard(int maxRetries = 6, int retryDelay = 500) {
  for (int i = 0; i < maxRetries; i++) {
    if (SD.begin(SD_CS)) {
      Serial.println("SD card mounted successfully.");
      return true;
    }
    Serial.println("SD mount failed. Retrying...");
    delay(retryDelay);
  }
  return false;
}
// ewenty na wifi
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WebSocket] Połączono z klientem #%u\n", num);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WebSocket] Rozłączono klienta #%u\n", num);
      break;

    case WStype_TEXT:
      Serial.printf("[WebSocket] Otrzymano dane: %s\n", payload);

      // Odpowiedź (echo)
      webSocket.sendTXT(num, "OK: " + String((char*)payload));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

  // --- Start WiFi ---
  Serial.println("Start WiFi...");
  WiFi.begin(ssid, password);

  int x = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    x++;
    Serial.println(x);
    if (x > 20) {
      Serial.println("WiFi fail — restart ESP32");
      ESP.restart();  // restart po 10 sekundach
    }
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket serwer uruchomiony na porcie 81");
  Serial2.println("#SYST:WIFI_OK;");
  // SD start
  Serial.println("SD start");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  delay(100);
  if (!initSDCard()) {
    Serial.println("SD Card MOUNT FAIL after retries. Restarting ESP32...");
    delay(1000);
    ESP.restart();  // automatyczny restart
  }
  // audio  start
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(volume);  // default 0...21
  Serial.println("OK-RADIO");
  //  or alternative
  //audio.setVolumeSteps(100);  // max 255
  // audio.setVolume(volume);
  delay(100);
  gramy_mp3 = "/mp3/1.mp3";
  audio.connecttoFS(SD, gramy_mp3.c_str());
}

void loop() {
  webSocket.loop();
  audio.loop();  // obsługa dźwięku – musi być wysoko

  // Bufor znaków
  static String serialBuffer = "";

  while (Serial2.available()) {
    char ch = (char)Serial2.read();

    if (ch == '\n') {
      serialBuffer.trim();

      if (serialBuffer.length() > 0) {
        Serial.println("⮩ ODEBRANE: " + serialBuffer);

        if (serialBuffer.startsWith("#")) {
          Serial.println("📡 Otrzymano dane: " + serialBuffer);
          webSocket.broadcastTXT(serialBuffer);

          if (serialBuffer.startsWith("#R2D2:") || serialBuffer.startsWith("#SYST:")) {
            handleCommand(serialBuffer.substring(1));
          }
        } else {
          Serial.println("⚠️ Pominięto nieznaną ramkę: " + serialBuffer);
        }
      }

      serialBuffer = "";  // wyczyść bufor po każdej linii
    } else {
      // Dopisuj tylko, jeśli nie przepełniamy RAMu
      if (serialBuffer.length() < 256) {
        serialBuffer += ch;
      } else {
        Serial.println("❌ Przepełnienie bufora Serial2! Resetuję bufor.");
        serialBuffer = "";
      }
    }
  }
}





void handleCommand(String cmd) {
  // R2D2:PLAY:4:15
  int first = cmd.indexOf(':');
  int second = cmd.indexOf(':', first + 1);
  int third = cmd.indexOf(':', second + 1);

  String folder = cmd.substring(0, first);
  String action = cmd.substring(first + 1, second);
  String param1 = cmd.substring(second + 1, third == -1 ? cmd.length() : third);
  String param2 = third == -1 ? "" : cmd.substring(third + 1);

  param1.trim();
  param2.trim();

  if (action == "PLAY") {
    String path = "/mp3/" + folder + "/" + param1 + ".mp3";
    gramy_mp3 = path;
    if (param2.length() > 0) {
      int vol = param2.toInt();
      vol = constrain(vol, 0, 21);
      audio.setVolume(vol);
    }
    koniec_mp3 = 0;
    audio.connecttoFS(SD, gramy_mp3.c_str());
  }

  else if (action == "STOP") {
    audio.stopSong();
    koniec_mp3 = 1;
  }

  else if (action == "VOL") {
    int vol = param1.toInt();
    vol = constrain(vol, 0, 21);
    audio.setVolume(vol);
  }

  Serial.println("Wykonano: " + cmd);
}


void play_mp3(const String& path) {
  if (audio.isRunning()) {
    audio.stopSong();  // zatrzymaj jeśli coś gra
  }

  koniec_mp3 = 0;
  gramy_mp3 = path;
  audio.connecttoFS(SD, gramy_mp3.c_str());
}
// koniec pliku mp3
void audio_eof_mp3(const char* info) {  //end of file
  Serial.print("eof_mp3     ");
  Serial2.println("#SYST:DONE;");
  koniec_mp3 = 1;  // 1 odegrane 0 gramy
}
