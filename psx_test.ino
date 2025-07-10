#include <AccelStepper.h>
#include <PS2X_lib.h>

// 2 silniki: lewy i prawy, kopulka
AccelStepper motorL(AccelStepper::HALF4WIRE, 9, 7, 8, 6);   // silnik lewy
AccelStepper motorR(AccelStepper::HALF4WIRE, 5, 3, 4, 2);   // silnik prawy
AccelStepper motorK(AccelStepper::HALF4WIRE, 28, 26, 27, 25);   // silnik kopulka

// Pad
PS2X ps2x;
int error;
static unsigned long lastRead = 0;

void setup() {
  Serial.begin(115200);

  motorL.setMaxSpeed(1000);
  motorL.setAcceleration(500);

  motorR.setMaxSpeed(1000);
  motorR.setAcceleration(500);

  motorK.setMaxSpeed(1000);
  motorK.setAcceleration(500);
  motorK.disableOutputs();

  error = ps2x.config_gamepad(15, 11, 10, 12, true, true);

  if (error == 0) {
    Serial.println("Pad OK");
  } else {
    Serial.println("Błąd pada");
  }
  Serial.println("OK");
}

void loop() {
  if (millis() - lastRead >= 10) {
    ps2x.read_gamepad(false, 0);
    lastRead = millis();
  }

  int joyY = ps2x.Analog(PSS_LY);
  int joyX = ps2x.Analog(PSS_LX);
  int joyZ = ps2x.Analog(PSS_RX);
  // Martwa strefa ±10 wokół środka
  bool neutralY = abs(joyY - 128) < 10;
  bool neutralX = abs(joyX - 128) < 10;
  bool neutralZ = abs(joyZ - 128) < 10;

  if (neutralY && neutralX) {
    // Joystick w spoczynku – odłącz silniki
    motorL.disableOutputs();
    motorR.disableOutputs();
  } else {
    // Aktywne sterowanie – włącz silniki i jedź
    motorL.enableOutputs();
    motorR.enableOutputs();  

    int speed = map(joyY, 0, 255, 1000, -1000);
    int turn  = map(joyX, 0, 255, -1000, 1000);

    int leftSpeed  = speed + turn;
    int rightSpeed = speed - turn;

    motorL.setSpeed(leftSpeed);
    motorR.setSpeed(rightSpeed);

    motorL.runSpeed();
    motorR.runSpeed();
  }
  // kopulka osobno
  if (neutralZ) {
    // Joystick w spoczynku – odłącz silnik
    motorK.disableOutputs();
  } else {
    // Aktywne sterowanie – włącz silnik i obracaj
    motorL.enableOutputs(); 

    int speed_k = map(joyZ, 0, 255, 1000, -1000);

    int KSpeed  = speed_k;

    motorK.setSpeed(KSpeed);

    motorK.runSpeed();
  }
}
