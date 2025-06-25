
 #include <AccelStepper.h>

#define STEPS_PER_REV 6096  // pełny obrót wyjściowego wału silnika 28BYJ-48

// Prawy silnik: IN1=9, IN2=7, IN3=8, IN4=6
AccelStepper prawy(AccelStepper::HALF4WIRE, 9, 7, 8, 6);

// Lewy silnik: IN1=5, IN2=3, IN3=4, IN4=2
AccelStepper lewy(AccelStepper::HALF4WIRE, 5, 3, 4, 2);

// Etapy działania
enum State { FORWARD, PAUSE1, BACKWARD, PAUSE2 };
State state = FORWARD;

unsigned long pauseStart = 0;

void setup() {
  prawy.setMaxSpeed(1100);       // maksymalna prędkość (kroki/s)
  prawy.setAcceleration(500);   // przyspieszenie (kroki/s^2)

  lewy.setMaxSpeed(1100);
  lewy.setAcceleration(500);

  // Rozpocznij ruch do przodu
  prawy.moveTo(STEPS_PER_REV);
  lewy.moveTo(STEPS_PER_REV);
}

void loop() {
  switch (state) {

    case FORWARD:
      prawy.run();
      lewy.run();
      if (prawy.distanceToGo() == 0 && lewy.distanceToGo() == 0) {
        prawy.disableOutputs();
        lewy.disableOutputs();
        pauseStart = millis();
        state = PAUSE1;
      }
      break;

    case PAUSE1:
      if (millis() - pauseStart >= 4000) {
        prawy.enableOutputs();
        lewy.enableOutputs();
        prawy.moveTo(0);
        lewy.moveTo(0);
        state = BACKWARD;
      }
      break;

    case BACKWARD:
      prawy.run();
      lewy.run();
      if (prawy.distanceToGo() == 0 && lewy.distanceToGo() == 0) {
        prawy.disableOutputs();
        lewy.disableOutputs();
        pauseStart = millis();
        state = PAUSE2;
      }
      break;

    case PAUSE2:
      if (millis() - pauseStart >= 4000) {
        // Restartujemy cały cykl (opcjonalnie)
        prawy.enableOutputs();
        lewy.enableOutputs();
        prawy.moveTo(STEPS_PER_REV);
        lewy.moveTo(STEPS_PER_REV);
        state = FORWARD;
      }
      break;
  }
}
