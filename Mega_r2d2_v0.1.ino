#include <AccelStepper.h>
#include <PS2X_lib.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_INA219.h>
#include <Adafruit_BNO08x.h>


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_INA219 ina219;
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

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
// logika
String pendingSerial2Command = "";
bool waitForDone = false;
int losowanyNumer = 0;

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial2.begin(115200);
  delay(20);

  pinMode(PIN_A12, OUTPUT);
  digitalWrite(PIN_A12, HIGH);  // podtrzymanie zasilania

  pinMode(czujnikPin, INPUT);  // czujnik

  motorL.setMaxSpeed(1200);
  motorL.setAcceleration(200);

  motorR.setMaxSpeed(1200);
  motorR.setAcceleration(200);

  motorK.setMaxSpeed(1200);
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
  // init vlox
  Serial.println("Adafruit VL53L0X INIT.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  lox.startRangeContinuous();
  // init INA
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1)
      ;
  }
  //bBNO085
  if (!bno08x.begin_I2C()) {
    Serial.println("Nie znaleziono BNO08x");
    while (1)
      ;
  }
  Serial.println("BNO08x OK");

  // Rotation vector = yaw/pitch/roll względem północy (z magnetometrem)
  bno08x.enableReport(SH2_ROTATION_VECTOR);
  delay(100);

  Serial.println("⏳ Oczekiwanie na gotowość ESP32 (WiFi)...");

  while (true) {
    if (Serial2.available()) {
      String msg = Serial2.readStringUntil(';');
      msg.trim();
      if (msg == "#SYST:WIFI_OK") {
        Serial.println("✅ ESP32 potwierdził połączenie z WiFi.");
        break;
      }
    }
    delay(100);  // krótka pauza, by nie zamulić loopa
  }


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

  // Kwadrat – wypisz po puszczeniu
  if (ps2x.Button(PSB_SQUARE)) {
    wasSquarePressed = true;
  } else if (wasSquarePressed) {
    Serial.println("Kwadrat");

    // Jeśli nie trwa odtwarzanie – wysyłaj od razu
    if (!waitForDone) {
      Serial2.println("#R2D2:PLAY:4:15;");
      waitForDone = true;
    } else {
      // Jeśli trwa – zapamiętaj do późniejszego wysłania
      pendingSerial2Command = "#R2D2:PLAY:4:15;";
      Serial.println("🎵 Polecenie zapamiętane – zostanie wysłane później");
    }

    wasSquarePressed = false;
  }

  // Kółko – wypisz po puszczeniu
  if (ps2x.Button(PSB_CIRCLE)) {
    wasCirclePressed = true;
  } else if (wasCirclePressed) {
    Serial.println("Koło");
    digitalWrite(PIN_A12, LOW);
    wasCirclePressed = false;
  }


  // Krzyżyk – po puszczeniu ustaw pin A12 na LOW // power off
  if (ps2x.Button(PSB_CROSS)) {
    wasCrossPressed = true;
  } else if (wasCrossPressed) {
    Serial.println("OFF");
    digitalWrite(PIN_A12, LOW);
    wasCrossPressed = false;
  }
  // pomiar zasilania i sprawdzenie mp3
  static uint32_t ostatniCheck = 0;

  if (millis() - ostatniCheck >= 1000) {
    ostatniCheck = millis();

    if (!czekajNaDone()) {
      Serial.println("✅ Odebrano #SYST:DONE");
      waitForDone = false;

      // Jeśli mamy polecenie oczekujące – wyślij je teraz
      if (pendingSerial2Command.length() > 0) {
        Serial2.println(pendingSerial2Command);
        Serial.println("📤 Wysłano zaległe polecenie:");
        Serial.println(pendingSerial2Command);
        pendingSerial2Command = "";
        waitForDone = true;
      }
    }

    zasilanie();
  }
}

// Sprawdzenie czy koniec utworu
byte czekajNaDone() {
  static bool done = false;

  if (Serial2.available()) {
    String wiadomosc = Serial2.readStringUntil(';');
    if (wiadomosc == "#SYST:DONE") {
      Serial.println("✅ Odebrano #SYST:DONE");
      done = true;
    }
  }

  if (done) {
    done = false;
    return 0;
  }

  return 1;
}
// -----------Blok autostartu------------
//  --------- mozliwe użycie delay -----
void autostart() {
  kopulka_zero();

  Serial2.println("#SYST:PLAY:1:21;");
  Serial.println("⏳ Czekam na #SYST:DONE (1/12)...");
  while (czekajNaDone()) {
    delay(100);
  }

  Serial2.println("#R2D2:PLAY:5:12;");
  Serial.println("⏳ Czekam na #SYST:DONE (9/12)...");
  while (czekajNaDone()) {
    delay(100);
  }
}

// ======== FUNKCJA RC PILOT ============
void rc_pilot() {
  Serial.println("Tryb RC: START");

  bool obiektBlisko = false;
  unsigned long lastDistSend = 0;

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
      lastRead = millis();
    }
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

if (millis() - lastDistSend >= 500) {
  lastDistSend = millis();

  float yaw = 0.0;
  if (bno08x.getSensorEvent(&sensorValue) &&
      sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;
    yaw = atan2(2.0f * (qw * qz + qx * qy),
                1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / PI;
    if (yaw < 0) yaw += 360;
  }

  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);

 String msg = "#DATA:DIST=" + String(dystans) +
             ";YAW=" + String(yaw, 1) +
             ";V=" + String(loadvoltage, 2) + ";";
Serial2.println(msg);
Serial.print("Wysyłam pakiet: ");
Serial.println(msg);
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
    // losowe dzwieki r2d2
    if (ps2x.Button(PSB_SQUARE)) {
      wasSquarePressed = true;
    } else if (wasSquarePressed) {
      losowanyNumer = random(1, 13);  // 1–12 włącznie
      String komenda = "#R2D2:PLAY:" + String(losowanyNumer) + ":12;";
      Serial2.println(komenda);
      Serial.print("🔊 Wysłano polecenie: ");
      Serial.println(komenda);
      wasSquarePressed = false;
    }


    // Sterowanie – joystick
    int joyY = ps2x.Analog(PSS_LY);
    int joyX = ps2x.Analog(PSS_LX);
    int joyZ = ps2x.Analog(PSS_RX);
    pozycja_k = motorK.currentPosition();

    bool neutralY = abs(joyY - 128) < 15;
    bool neutralX = abs(joyX - 128) < 15;
    bool neutralZ = abs(joyZ - 128) < 15;

    if (neutralY && neutralX) {
      motorL.disableOutputs();
      motorR.disableOutputs();
      motorL.setSpeed(0);
      motorR.setSpeed(0);
    } else {
      motorL.enableOutputs();
      motorR.enableOutputs();
      int speed = map(joyY, 0, 255, -1000, 1000);
      int turn = map(joyX, 0, 255, -1000, 1000);
      motorL.setSpeed(speed + turn);
      motorR.setSpeed(speed - turn);
      motorL.runSpeed();
      motorR.runSpeed();
    }

    if (neutralZ) {
      motorK.disableOutputs();
      motorK.setSpeed(0);
    } else {
      int speed_k = map(joyZ, 0, 255, 1000, -1000);
      long aktualnaPozycja = motorK.currentPosition();
      bool dozwolonyRuch =
        (speed_k > 0 && aktualnaPozycja < 2048) || (speed_k < 0 && aktualnaPozycja > -2048);

      if (dozwolonyRuch) {
        motorK.enableOutputs();
        motorK.setSpeed(speed_k);
        motorK.runSpeed();
        motorL.runSpeed();
        motorR.runSpeed();
      } else {
        motorK.disableOutputs();
        motorK.setSpeed(0);
        motorL.runSpeed();
        motorR.runSpeed();
      }
    }

    // 👇 runSpeed() dla płynności
    motorL.runSpeed();
    motorR.runSpeed();
    motorK.runSpeed();
    zasilanie();
    //motorL.runSpeed();
    //motorR.runSpeed();
    //motorK.runSpeed();
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
  static uint16_t pomiary[FILTR_OKNO] = { 9999 };
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
// odczyt zasilania
void zasilanie() {
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  static uint32_t ostatniPomiar = 0;

  // Odstęp między pomiarami (w ms)
  if (millis() - ostatniPomiar >= 10000) {


    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print("Bus Voltage:   ");
    Serial.print(busvoltage);
    Serial.println(" V");
    Serial.print("Shunt Voltage: ");
    Serial.print(shuntvoltage);
    Serial.println(" mV");
    Serial.print("Load Voltage:  ");
    Serial.print(loadvoltage);
    Serial.println(" V");
    Serial.print("Current:       ");
    Serial.print(current_mA);
    Serial.println(" mA");
    Serial.print("Power:         ");
    Serial.print(power_mW);
    Serial.println(" mW");
    Serial.println("");
    ostatniPomiar = millis();
  }
}
void pozycja() {
  if (!bno08x.getSensorEvent(&sensorValue)) return;

  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;

    // Przeliczanie quaternion → yaw/pitch/roll
    float yaw = atan2(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz))
                * 180.0f / PI;
    float pitch = asin(2.0f * (qw * qy - qz * qx)) * 180.0f / PI;
    float roll = atan2(2.0f * (qw * qx + qy * qz),
                       1.0f - 2.0f * (qx * qx + qy * qy))
                 * 180.0f / PI;

    if (yaw < 0) yaw += 360;

    Serial.print("Yaw: ");
    Serial.print(yaw, 1);
    Serial.print("°, Pitch: ");
    Serial.print(pitch, 1);
    Serial.print("°, Roll: ");
    Serial.println(roll, 1);
  }
}