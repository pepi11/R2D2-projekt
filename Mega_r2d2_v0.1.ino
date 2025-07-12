#include <AccelStepper.h>
#include <PS2X_lib.h>

// 2 silniki: lewy i prawy, kopułka
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

#define PIN_A12 66  // zasilanie off
const int czujnikPin = 34;  // szczelinowy

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial2.begin(115200);
  delay(20);

  pinMode(PIN_A12, OUTPUT);
  digitalWrite(PIN_A12, HIGH);  // podtrzymanie zasilania
  
  pinMode(czujnikPin, INPUT); // czujnik

  motorL.setMaxSpeed(1000);
  motorL.setAcceleration(500);

  motorR.setMaxSpeed(1000);
  motorR.setAcceleration(500);

  motorK.setMaxSpeed(1000);
  motorK.setAcceleration(500);
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
  // procedury startowe
  autostart();
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
  Serial2.println("#SYST:PLAY:1:18;");
  czekajNaDone();
  Serial2.println("#R2D2:PLAY:5:12;");
}
// ======== FUNKCJA RC PILOT ============
void rc_pilot() {
  Serial.println("Tryb RC: START");

  while (true) {
    if (millis() - lastRead >= 10) {
      ps2x.read_gamepad(false, 0);
      lastRead = millis();
    }
    // Jeśli krzyżyk wciśnięt i puszcony wychodzimy do loop
    if (ps2x.Button(PSB_CROSS)) {
      wasCrossPressed = true;
    }
    if (wasCrossPressed && !ps2x.Button(PSB_CROSS)) {
      wasCrossPressed = false;
      break;
    }

    int joyY = ps2x.Analog(PSS_LY);
    int joyX = ps2x.Analog(PSS_LX);
    int joyZ = ps2x.Analog(PSS_RX);
    pozycja_k = motorK.currentPosition();

    // Martwa strefa
    bool neutralY = abs(joyY - 128) < 10;
    bool neutralX = abs(joyX - 128) < 10;
    bool neutralZ = abs(joyZ - 128) < 10;


    // Napęd
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

      motorL.runSpeed();
      motorR.runSpeed();
    }

    // kopulka osobno
    if (neutralZ) {
      motorK.disableOutputs();
      motorK.setSpeed(0);
    } else {
      int speed_k = map(joyZ, 0, 255, 1000, -1000);
      long aktualnaPozycja = motorK.currentPosition();
      bool dozwolonyRuch = false;


      if (speed_k > 0 && aktualnaPozycja < 2048) {
        dozwolonyRuch = true;
      } else if (speed_k < 0 && aktualnaPozycja > -2048) {
        dozwolonyRuch = true;
      }

      if (dozwolonyRuch) {
        motorK.enableOutputs();
        motorK.setSpeed(speed_k);
        motorK.runSpeed();
      } else {
        motorK.disableOutputs();
        motorK.setSpeed(0);
      }
    }
  }

  // Po wyjściu z trybu RC:
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
