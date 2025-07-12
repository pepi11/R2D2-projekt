#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include "Arduino.h"
#include <Wire.h>
#include <ESP32Time.h>
#include <WiFi.h>
#include <WiFiUdp.h>
//#include <ESPping.h>
//include <Ping.h>
#include "Audio.h"  // usunieto w biliotece Sd.h
#include <Preferences.h>



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


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  // SD start
  Serial.println("SD start");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  delay(100);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card MOUNT FAIL");
    return;
  }
  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  String str = "SDCard Size: " + String(cardSize) + "MB";
  Serial.println(str);
  // audio  start
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(volume); // default 0...21
  Serial.println("OK-RADIO");
  //  or alternative
  //audio.setVolumeSteps(100);  // max 255
 // audio.setVolume(volume);
  delay(100);
  gramy_mp3 = "/mp3/1.mp3";
  audio.connecttoFS(SD, gramy_mp3.c_str());
}

void loop() {
    if (Serial2.available()) {
    String cmd = Serial2.readStringUntil(';'); // czytamy do znaku końca
    Serial2.println(cmd);
    cmd.trim();
    if (cmd.startsWith("#")) {
      handleCommand(cmd.substring(1)); // usuń #
    }
  }

  audio.loop(); // ważne – ciągła obsługa audio
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
