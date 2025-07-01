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
  audio.setVolume(21); // default 0...21
  Serial.println("OK-RADIO");
  //  or alternative
  //audio.setVolumeSteps(100);  // max 255
 // audio.setVolume(volume);
  delay(100);
  gramy_mp3 = "/mp3/1.mp3";
  audio.connecttoFS(SD, gramy_mp3.c_str());
}

void loop() {
 // gramy_mp3 = "/mp3/001/" + String(nr_mp3) + ".mp3";
   delay(2000);
  gramy_mp3 = "/mp3/5.mp3";
  play_mp3();
  delay(1000);
  gramy_mp3 = "/mp3/4.mp3";
  play_mp3();
  delay(1000);
  gramy_mp3 = "/mp3/2.mp3";
  play_mp3();
  gramy_mp3 = "/mp3/1.mp3";
  play_mp3();
  delay(5000);
}

void play_mp3() {

  //gramy_mp3 = "/mp3/010/118.mp3";
  
  audio.connecttoFS(SD, gramy_mp3.c_str());
  do {
    audio.loop();
  } while (koniec_mp3 != 1 );
  koniec_mp3 = 0;  // zerowanie petli grania
  audio.stopSong();

}
// koniec pliku mp3
void audio_eof_mp3(const char* info) {  //end of file
  Serial.print("eof_mp3     ");
  Serial.println(info);
  koniec_mp3 = 1;  // 1 odegrane 0 gramy
}
