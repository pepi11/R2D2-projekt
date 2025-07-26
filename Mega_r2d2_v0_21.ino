#include <AccelStepper.h>
#include <PS2X_lib.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_INA219.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>

#define PIN A15  // WS2812B na A15
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// blok 2 vl53
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
// set the pins to shutdown
#define SHT_LOX1 A13
#define SHT_LOX2 33

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_k = Adafruit_VL53L0X();
// this holds the measurement


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
static bool wasPadUp = false;
static bool wasPadDown = false;
static bool wasPadLeft = false;
static bool wasPadRight = false;
long pozycja_k = 0;

#define PIN_A12 66          // zasilanie off
const int czujnikPin = 34;  // szczelinowy
// logika
String pendingSerial2Command = "";
bool waitForDone = false;
int losowanyNumer = 0;
float yawOffset = 0.0;
// deklaracje funkcji

void L_prosta(bool doTylu = false);
void kat_90(bool wLewo = false);
void kwadrat_demo();
void kurs(float odleglosc_cm, float kursDocelowy, bool tyl = false);
float yawRelative();
void setID();

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial2.begin(57600);
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
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));
  Serial.println(F("Starting..."));
  setID();

  lox.startRangeContinuous();
  Serial.println("kopulka start OK");
  lox_k.startRangeContinuous();
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
  pixels.begin();
  pixels.show();  // start: wyczyszczenie diody

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

  // Góra – jazda do przodu
  if (ps2x.Button(PSB_PAD_UP)) {
    wasPadUp = true;
  } else if (wasPadUp) {
    L_prosta(false);  // przód
    wasPadUp = false;
  }

  // Dół – jazda do tyłu
  if (ps2x.Button(PSB_PAD_DOWN)) {
    wasPadDown = true;
  } else if (wasPadDown) {
    L_prosta(true);  // tył
    wasPadDown = false;
  }

  // Lewo – obrót 90° w lewo
  if (ps2x.Button(PSB_PAD_LEFT)) {
    wasPadLeft = true;
  } else if (wasPadLeft) {
    kat_90(true);  // w lewo
    wasPadLeft = false;
  }

  // Prawo – obrót 90° w prawo
  if (ps2x.Button(PSB_PAD_RIGHT)) {
    wasPadRight = true;
  } else if (wasPadRight) {
    kat_90(false);  // w prawo
    wasPadRight = false;
  }

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
    kurs(60.0, 110.0, false);
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
  led_neo(1);
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

  // 🔄 Automatyczne zerowanie yaw – ustaw aktualny kierunek jako 0
  float suma = 0;
  int proby = 0;

  for (int i = 0; i < 5; i++) {
    if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;

      float yaw = atan2(2.0f * (qw * qz + qx * qy),
                        1.0f - 2.0f * (qy * qy + qz * qz))
                  * 180.0f / PI;
      if (yaw < 0) yaw += 360;

      suma += yaw;
      proby++;
    }
    delay(50);
  }

  if (proby > 0) {
    yawOffset = suma / proby;
    Serial.print("🎯 yawOffset ustawiony (średnia z 5): ");
    Serial.println(yawOffset, 1);
  } else {
    Serial.println("❌ Nie udało się ustawić yawOffset – brak danych z BNO");
  }
}

float yawRelative() {
  if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    float qw = sensorValue.un.rotationVector.real;
    float qx = sensorValue.un.rotationVector.i;
    float qy = sensorValue.un.rotationVector.j;
    float qz = sensorValue.un.rotationVector.k;

    float yaw = atan2(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz))
                * 180.0f / PI;
    if (yaw < 0) yaw += 360;

    float rel = yaw - yawOffset;
    if (rel < 0) rel += 360;
    return rel;
  }
  return 0;
}

// ======== FUNKCJA RC PILOT ============
void rc_pilot() {
  Serial.println("Tryb RC: START");

  bool obiektBlisko = false;
  unsigned long lastDistSend = 0;

  // Stabilizacja – zapełnienie bufora filtrowanego
  uint32_t start = millis();
  while (millis() - start < 300) {
    odleglosc_k();
    motorL.runSpeed();
    motorR.runSpeed();
    motorK.runSpeed();
  }

  while (true) {
    if (millis() - lastRead >= 100) {
      ps2x.read_gamepad(false, 0);
      lastRead = millis();
    }

    uint16_t dystans_k = odleglosc_k(); // górny kapsulka
    uint16_t dystans = odleglosc();    // dolny gondola

    if (dystans < 250 && !obiektBlisko) {
      Serial2.println("#SYST:PLAY:2:21;");
      Serial.print("📏 Zbliżenie (");
      Serial.print(dystans);
      Serial.println(" mm) – komunikat wysłany");
      obiektBlisko = true;
    }

    if (millis() - lastDistSend >= 500) {
      lastDistSend = millis();

      float yaw = yawRelative();

      float shuntvoltage = ina219.getShuntVoltage_mV();
      float busvoltage = ina219.getBusVoltage_V();
      float loadvoltage = busvoltage + (shuntvoltage / 1000);

      String msg = "#DATA:DIST_K=" + String(dystans_k) + ";DIST=" + String(dystans) + ";YAW=" + String(yaw, 1) + ";V=" + String(loadvoltage, 2) + ";";

      if (Serial2.availableForWrite() > msg.length() + 2) {
        Serial2.println(msg);
      } else {
        Serial.println("⚠️ Serial2 bufor pełny – pominięto pakiet");
      }
      Serial.print("Wysyłam pakiet: ");
      Serial.println(msg);
    }

    if (dystans > 350 && obiektBlisko) {
      Serial.print("📏 Oddalenie (");
      Serial.print(dystans);
      Serial.println(" mm) – reset zatrzasku");
      obiektBlisko = false;
    }

    if (ps2x.Button(PSB_CROSS)) {
      wasCrossPressed = true;
    }
    if (wasCrossPressed && !ps2x.Button(PSB_CROSS)) {
      wasCrossPressed = false;
      break;
    }

    if (ps2x.Button(PSB_SQUARE)) {
      wasSquarePressed = true;
    } else if (wasSquarePressed) {
      losowanyNumer = random(1, 13);
      String komenda = "#R2D2:PLAY:" + String(losowanyNumer) + ":12;";
      Serial2.println(komenda);
      Serial.print("🔊 Wysłano polecenie: ");
      Serial.println(komenda);
      wasSquarePressed = false;
    }

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
      bool dozwolonyRuch = (speed_k > 0 && aktualnaPozycja < 2048) || (speed_k < 0 && aktualnaPozycja > -2048);

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

    motorL.runSpeed();
    motorR.runSpeed();
    motorK.runSpeed();
    zasilanie();
    led_neo(5);
  }

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
// czytnik dolny vl54lox
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
// czytnik dolny vl54lox
#define FILTR_OKNO_k 5

uint16_t odleglosc_k() {
  static uint16_t pomiary[FILTR_OKNO_k] = { 9999 };
  static int index = 0;
  static uint32_t ostatniPomiar = 0;

  // Odstęp między pomiarami (w ms)
  if (millis() - ostatniPomiar >= 63 && lox_k.isRangeComplete()) {
    uint16_t nowy = lox_k.readRange();
    pomiary[index] = nowy;
    index = (index + 1) % FILTR_OKNO_k;
    ostatniPomiar = millis();
  }

  // Średnia
  uint32_t suma = 0;
  for (int i = 0; i < FILTR_OKNO_k; i++) {
    suma += pomiary[i];
  }
  return suma / FILTR_OKNO_k;
}
// odczyt zasilania
void zasilanie() {
  static uint32_t ostatniPomiar = 0;
  static bool niskieNapiecieZgloszone = false;
  static bool krytyczneNapiecieZgloszone = false;

  if (millis() - ostatniPomiar >= 1000) {
    ostatniPomiar = millis();

    float busvoltage = ina219.getBusVoltage_V();
    float shuntvoltage = ina219.getShuntVoltage_mV();
    float loadvoltage = busvoltage + (shuntvoltage / 1000.0);

    Serial.print("🔋 Napięcie: ");
    Serial.print(loadvoltage, 2);
    Serial.println(" V");

    if (loadvoltage < 6.6 && !krytyczneNapiecieZgloszone) {
      Serial2.println("#SYST:PLAY:4:21;");
      Serial.println("❗ Krytyczne napięcie – zatrzymanie!");

      // Zatrzymanie silników
      motorL.setSpeed(0);
      motorR.setSpeed(0);
      motorK.setSpeed(0);

      motorL.disableOutputs();
      motorR.disableOutputs();
      motorK.disableOutputs();

      krytyczneNapiecieZgloszone = true;
    } else if (loadvoltage < 7.0 && !niskieNapiecieZgloszone) {
      Serial2.println("#SYST:PLAY:3:21;");
      Serial.println("⚠️ Niskie napięcie – ostrzeżenie");
      niskieNapiecieZgloszone = true;
    }

    // Reset flag, jeśli napięcie wróci do normy
    if (loadvoltage >= 7.0) {
      niskieNapiecieZgloszone = false;
      krytyczneNapiecieZgloszone = false;
    }
  }
}



// 90 stopni prawo lewo
//-------------------------------------------------------
void kat_90(bool wLewo = false) {
  float aktualnyYaw = yawRelative();
  float offset = wLewo ? 88.0 : -88.0;
  float celYaw = fmod((aktualnyYaw + offset + 360.0), 360.0);

  Serial.print(wLewo ? "↩️ Obrót w LEWO do yaw = " : "↪️ Obrót w PRAWO do yaw = ");
  Serial.println(celYaw, 1);

  motorL.enableOutputs();
  motorR.enableOutputs();

  int predkosc = 1100;
  motorL.setSpeed(wLewo ? -predkosc : predkosc);
  motorR.setSpeed(wLewo ? predkosc : -predkosc);

  while (true) {
    float aktualny = yawRelative();
    float roznica = celYaw - aktualny;
    if (roznica < 0) roznica += 360;

    motorL.runSpeed();
    motorR.runSpeed();

    if (roznica <= 1.0 || roznica >= 359.0) break;
  }

  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorL.disableOutputs();
  motorR.disableOutputs();
  ps2xFlush();
  Serial.println("✅ Obrót 90° zakończony");
}


// 20 cm do tyłu i przodu
void L_prosta(bool doTylu = false) {
  Serial.println(doTylu ? "⬅️ Jazda do tyłu 20 cm..." : "➡️ Jazda do przodu 20 cm...");

  const long krokiDoPrzejazdu = 8530;  // dla 20 cm
  const float histereza = 1.0;
  const int predkosc = 1000;
  const int korekta = 50;

  float startYaw = yawRelative();

  motorL.setCurrentPosition(0);
  motorR.setCurrentPosition(0);
  motorL.enableOutputs();
  motorR.enableOutputs();

  while (abs(motorL.currentPosition()) < krokiDoPrzejazdu) {
    float yawNow = yawRelative();

    float blad = yawNow - startYaw;
    if (blad > 180) blad -= 360;
    if (blad < -180) blad += 360;

    int baza = doTylu ? predkosc : -predkosc;
    int speedL = baza;
    int speedR = baza;

    if (blad > histereza) {
      speedL -= korekta;
      speedR += korekta;
    } else if (blad < -histereza) {
      speedL += korekta;
      speedR -= korekta;
    }

    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
    motorL.runSpeed();
    motorR.runSpeed();
  }

  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorL.disableOutputs();
  motorR.disableOutputs();
  ps2xFlush();
  Serial.println("✅ Jazda zakończona.");
}

// jazda po kursie
void kurs(float odleglosc_cm, float kursDocelowy, bool tyl = false) {
  Serial.print("🚀 Start jazdy na kurs ");
  Serial.print(kursDocelowy, 1);
  Serial.print("°, ");
  Serial.print(odleglosc_cm, 1);
  Serial.println(" cm");

  const float histereza = 1.0;
  const float tolerancjaObrotu = 1.0;
  const int predkosc = 1000;
  const int korekta = 50;

  // Krok 1: obróć się do kursDocelowy
  float roznica = kursDocelowy - yawRelative();
  if (roznica < -180) roznica += 360;
  if (roznica > 180) roznica -= 360;

  if (abs(roznica) > tolerancjaObrotu) {
    Serial.print("↪️ Obrót do kursu... różnica = ");
    Serial.println(roznica, 1);

    int obrSpeed = 1100;
    motorL.enableOutputs();
    motorR.enableOutputs();
    motorL.setSpeed(roznica > 0 ? -obrSpeed : obrSpeed);
    motorR.setSpeed(roznica > 0 ? obrSpeed : -obrSpeed);

    while (true) {
      float aktualny = yawRelative();
      float blad = kursDocelowy - aktualny;
      if (blad < -180) blad += 360;
      if (blad > 180) blad -= 360;

      motorL.runSpeed();
      motorR.runSpeed();

      if (abs(blad) <= tolerancjaObrotu) break;
    }

    motorL.setSpeed(0);
    motorR.setSpeed(0);
    motorL.disableOutputs();
    motorR.disableOutputs();
    Serial.println("✅ Obrót zakończony");
    delay(100);
  }

  // Krok 2: jazda
  long krokiDocelowe = odleglosc_cm * 418;
  motorL.setCurrentPosition(0);
  motorR.setCurrentPosition(0);
  motorL.enableOutputs();
  motorR.enableOutputs();

  while (abs(motorL.currentPosition()) < krokiDocelowe) {
    float aktualny = yawRelative();
    float blad = kursDocelowy - aktualny;
    if (blad > 180) blad -= 360;
    if (blad < -180) blad += 360;

    int baza = tyl ? predkosc : -predkosc;
    int speedL = baza;
    int speedR = baza;

    if (blad > histereza) {
      speedL -= korekta;
      speedR += korekta;
    } else if (blad < -histereza) {
      speedL += korekta;
      speedR -= korekta;
    }

    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
    motorL.runSpeed();
    motorR.runSpeed();
  }

  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorL.disableOutputs();
  motorR.disableOutputs();
  ps2xFlush();
  Serial.println("🏁 Kurs zakończony");
}
// zmiana adresu vlox
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox_k.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot gondola VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot kopulka VL53L0X"));
    while (1)
      ;
  }
}
// obsluga ledki
void led_neo(uint8_t effect) {
  static uint8_t hue = 0;
  static int brightness = 0;
  static int fadeDirection = 1;
  static bool on = false;
  static unsigned long lastUpdate = 0;
  unsigned long currentMillis = millis();

  switch (effect) {

    case 1: // 🌈 Tęczowy gradient
      if (currentMillis - lastUpdate >= 40) {
        lastUpdate = currentMillis;
        uint32_t color = pixels.ColorHSV(hue * 256);
        pixels.setPixelColor(0, color);
        pixels.show();
        hue++;
      }
      break;

    case 2: // 🚨 Miganie czerwone
      if (currentMillis - lastUpdate >= 300) {
        lastUpdate = currentMillis;
        on = !on;
        pixels.setPixelColor(0, on ? pixels.Color(255, 0, 0) : 0);
        pixels.show();
      }
      break;

    case 3: // 💖 Różowy fade
      if (currentMillis - lastUpdate >= 30) {
        lastUpdate = currentMillis;
        brightness += fadeDirection * 5;
        if (brightness <= 0 || brightness >= 255) fadeDirection *= -1;
        pixels.setPixelColor(0, pixels.Color(brightness, 0, brightness));
        pixels.show();
      }
      break;

    case 4: // ⚡ Stroboskop niebieski
      if (currentMillis - lastUpdate >= 100) {
        lastUpdate = currentMillis;
        on = !on;
        pixels.setPixelColor(0, on ? pixels.Color(0, 0, 255) : 0);
        pixels.show();
      }
      break;

    case 5: // 🎲 Losowe kolory
      if (currentMillis - lastUpdate >= 300) {
        lastUpdate = currentMillis;
        uint8_t r = random(0, 256);
        uint8_t g = random(0, 256);
        uint8_t b = random(0, 256);
        pixels.setPixelColor(0, pixels.Color(r, g, b));
        pixels.show();
      }
      break;

    case 6: // 🌊 Pulsujący niebieski
      if (currentMillis - lastUpdate >= 25) {
        lastUpdate = currentMillis;
        brightness += fadeDirection * 4;
        if (brightness <= 0 || brightness >= 255) fadeDirection *= -1;
        pixels.setPixelColor(0, pixels.Color(0, 0, brightness));
        pixels.show();
      }
      break;

    case 7: // 🍃 Oddech zielony
      if (currentMillis - lastUpdate >= 10) {
        lastUpdate = currentMillis;
        float breathe = (sin(millis() / 1000.0 * 2 * PI) + 1) / 2; // 0–1
        int val = (int)(breathe * 255);
        pixels.setPixelColor(0, pixels.Color(0, val, 0));
        pixels.show();
      }
      break;

    default:
      pixels.setPixelColor(0, 0); // Wyłącz
      pixels.show();
      break;
  }
}
