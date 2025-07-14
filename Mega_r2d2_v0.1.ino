#include <AccelStepper.h>
#include <PS2X_lib.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


// 3 silniki: lewy i prawy, kopułka
AccelStepper motorL(AccelStepper::HALF4WIRE, 9, 7, 8, 6);      // lewy
AccelStepper motorR(AccelStepper::HALF4WIRE, 5, 3, 4, 2);      // prawy
AccelStepper motorK(AccelStepper::HALF4WIRE, 28, 26, 27, 25);  // kopułka

// Pad
PS2X ps2x;
int error;
unsigned long lastRead = 0;
// Zmienne stanu przycisków
bool wasCirclePressed = false;
bool wasSquarePressed = false;
bool wasTrianglePressed = false;
bool wasCrossPressed = false;
long pozycja_k = 0;

#define PIN_A12 66          // zasilanie off
const int czujnikPin = 34;  // szczelinowy


void setup() {
  Serial.begin(115200);
  delay(20);
  Serial2.begin(115200);
  delay(20);

  pinMode(PIN_A12, OUTPUT);
  digitalWrite(PIN_A12, HIGH);  // podtrzymanie zasilania

  pinMode(czujnikPin, INPUT);  // czujnik

  motorL.setMaxSpeed(1000);
  motorL.setAcceleration(200);

  motorR.setMaxSpeed(1000);
  motorR.setAcceleration(200);

  motorK.setMaxSpeed(1000);
  motorK.setAcceleration(400);
  motorK.disableOutputs();
  // niepewny start pada
  //-----------------------------------------------
  const int MAX_PROBY = 6;
  int proba = 0;
  bool padOK = false;

  while (proba < MAX_PROBY) {
    error = ps2x.config_gamepad(15, 11, 10, 12, true, true);
    if (error == 0) {
      padOK = true;
      break;
    }
    proba++;
    Serial.print("Próba połączenia z padem: ");
    Serial.println(proba);
    delay(100);
  }

  if (padOK) {
    Serial.println("Pad OK");
  } else {
    Serial.println("Nie udało się połączyć z padem!");
    delay(20);
    // reset:
    asm volatile("  jmp 0");  // dla AVR
  }

  Serial.println("Adafruit VL53L0X INIT.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  lox.startRangeContinuous();
  // procedury startowe
  autostart();
  ps2xFlush();
}

void loop() {

  if (millis() - lastRead >= 10) {
    ps2x.read_gamepad(false, 0);
    lastRead = millis();
  }

  // Sprawdź przyciski
  // Trójkąt – uruchom tryb RC po puszczeniu przycisku
  if (ps2x.Button(PSB_TRIANGLE)) {
    wasTrianglePressed = true;
  } else if (wasTrianglePressed) {
    rc_pilot();
    wasTrianglePressed = false;
  }

  // Kółko – wypisz po puszczeniu
  if (ps2x.Button(PSB_CIRCLE)) {
    wasCirclePressed = true;
  } else if (wasCirclePressed) {
    Serial.println("Koło");
    wasCirclePressed = false;
  }

  // Kwadrat – wypisz po puszczeniu
  if (ps2x.Button(PSB_SQUARE)) {
    wasSquarePressed = true;
  } else if (wasSquarePressed) {
    Serial.println("Kwadrat");
    Serial2.println("#R2D2:PLAY:4:15;");
    wasSquarePressed = false;
  }

  // Krzyżyk – po puszczeniu ustaw pin A12 na LOW // power off
  if (ps2x.Button(PSB_CROSS)) {
    wasCrossPressed = true;
  } else if (wasCrossPressed) {
    Serial.println("OFF");
    digitalWrite(PIN_A12, LOW);
    wasCrossPressed = false;
  }
  // pomiar odleglosci
  
}
// oczekiwanie na koniec odtwarzania - blokada programu - tymczasowe
void czekajNaDone() {
  while (true) {
    if (Serial2.available()) {
      String wiadomosc = Serial2.readStringUntil(';');
      if (wiadomosc == "#SYST:DONE") {
        wiadomosc = "";
        break;  // kończymy pętlę, kontynuujemy program
      }
    }
  }
}
// -----------Blok autostartu------------
void autostart() {
  kopulka_zero();
  Serial2.println("#SYST:PLAY:1:21;");
  czekajNaDone();
  Serial2.println("#R2D2:PLAY:5:12;");
}
// ======== FUNKCJA RC PILOT ============
void rc_pilot() {
  Serial.println("Tryb RC: START");

  bool obiektBlisko = false;

  // Stabilizacja – zapełnienie bufora filtrowanego
  uint32_t start = millis();
  while (millis() - start < 300) {
    odleglosc();  // kilka odczytów, żeby bufor nie był pusty
    motorL.runSpeed();
    motorR.runSpeed();
    motorK.runSpeed();
  }

  while (true) {
    //ps2x.read_gamepad(false, 0);
    if (millis() - lastRead >= 100) {
      ps2x.read_gamepad(false, 0);
      lastRead = millis();}
    // Detekcja odległości
    uint16_t dystans = odleglosc();

    // Histereza: tylko raz przy zbliżeniu, reset przy oddaleniu
    if (dystans < 250 && !obiektBlisko) {
      Serial2.println("#SYST:PLAY:2:21;");
      Serial.print("📏 Zbliżenie (");
      Serial.print(dystans);
      Serial.println(" mm) – komunikat wysłany");
      obiektBlisko = true;
    }

    if (dystans > 350 && obiektBlisko) {
      Serial.print("📏 Oddalenie (");
      Serial.print(dystans);
      Serial.println(" mm) – reset zatrzasku");
      obiektBlisko = false;
    }

    // Wyjście z trybu RC
    if (ps2x.Button(PSB_CROSS)) {
      wasCrossPressed = true;
    }
    if (wasCrossPressed && !ps2x.Button(PSB_CROSS)) {
      wasCrossPressed = false;
      break;
    }

    // Sterowanie – joystick
    int joyY = ps2x.Analog(PSS_LY);
    int joyX = ps2x.Analog(PSS_LX);
    int joyZ = ps2x.Analog(PSS_RX);
    pozycja_k = motorK.currentPosition();

    bool neutralY = abs(joyY - 128) < 10;
    bool neutralX = abs(joyX - 128) < 10;
    bool neutralZ = abs(joyZ - 128) < 10;

    if (neutralY && neutralX) {
      motorL.disableOutputs();
      motorR.disableOutputs();
    } else {
      motorL.enableOutputs();
      motorR.enableOutputs();
      int speed = map(joyY, 0, 255, 1000, -1000);
      int turn = map(joyX, 0, 255, -1000, 1000);
      motorL.setSpeed(speed + turn);
      motorR.setSpeed(speed - turn);
    }

    if (neutralZ) {
      motorK.disableOutputs();
      motorK.setSpeed(0);
    } else {
      int speed_k = map(joyZ, 0, 255, 1000, -1000);
      long aktualnaPozycja = motorK.currentPosition();
      bool dozwolonyRuch =
          (speed_k > 0 && aktualnaPozycja < 2048) ||
          (speed_k < 0 && aktualnaPozycja > -2048);

      if (dozwolonyRuch) {
        motorK.enableOutputs();
        motorK.setSpeed(speed_k);
      } else {
        motorK.disableOutputs();
        motorK.setSpeed(0);
      }
    }

    // 👇 runSpeed() dla płynności
    motorL.runSpeed();
    motorR.runSpeed();
    motorK.runSpeed();
  }

  // Po wyjściu:
  motorL.disableOutputs();
  motorR.disableOutputs();
  motorK.disableOutputs();
  Serial.println("Tryb RC: STOP");
}

//---------------------------------------
// ustaw kopulke na zero
void kopulka_zero() {

  Serial.println("🔁 Szukanie przedniej krawędzi szczeliny...");

  motorK.setSpeed(300);  // obroty w lewo – dostosuj w razie potrzeby
  motorK.enableOutputs();

  bool krawedzZlapana = false;
  int ostatniStan = digitalRead(czujnikPin);

  while (!krawedzZlapana) {
    motorK.runSpeed();

    int aktualnyStan = digitalRead(czujnikPin);

    if (ostatniStan == HIGH && aktualnyStan == LOW) {
      // Złapano przednią krawędź
      long pos = motorK.currentPosition();
      Serial.print("✅ Krawędź wykryta na pozycji: ");
      Serial.println(pos);

      // Cofnij o 1690 kroków
      motorK.moveTo(pos - 1690);
      while (motorK.distanceToGo() != 0) {
        motorK.run();
      }

      // Ustaw nowe zero
      motorK.setCurrentPosition(0);
      motorK.disableOutputs();
      Serial.println("🎯 Pozycja zerowa ustawiona.");
      // Wyczyść śmieci z pada – 30 szybkich odczytów
      ps2xFlush();
      krawedzZlapana = true;
    }

    ostatniStan = aktualnyStan;
  }
}
// czyszczenie bufora pada bo CHRL
void ps2xFlush() {
  for (int i = 0; i < 30; i++) {
    ps2x.read_gamepad(false, 0);
    delay(2);
  }
}
// test vl54lox
#define FILTR_OKNO 5

uint16_t odleglosc() {
  static uint16_t pomiary[FILTR_OKNO] = {9999};
  static int index = 0;
  static uint32_t ostatniPomiar = 0;

  // Odstęp między pomiarami (w ms)
  if (millis() - ostatniPomiar >= 60 && lox.isRangeComplete()) {
    uint16_t nowy = lox.readRange();
    pomiary[index] = nowy;
    index = (index + 1) % FILTR_OKNO;
    ostatniPomiar = millis();
  }

  // Średnia
  uint32_t suma = 0;
  for (int i = 0; i < FILTR_OKNO; i++) {
    suma += pomiary[i];
  }
  return suma / FILTR_OKNO;
}