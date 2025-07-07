#include <Adafruit_NeoPixel.h>

#define PIN        2
#define NUMPIXELS  1
#define MAX_BRIGHTNESS 100  // maksymalna jasność (0–255)

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

unsigned long previousMillis = 0;
int currentEffect = 0;

void setup() {
  pixels.begin();
}

void loop() {
  // Zmień numer efektu tutaj
  led_neo(1);
}

// Funkcja pomocnicza – ogranicz jasność RGB
uint32_t limitBrightness(uint8_t r, uint8_t g, uint8_t b) {
  r = min(r, MAX_BRIGHTNESS);
  g = min(g, MAX_BRIGHTNESS);
  b = min(b, MAX_BRIGHTNESS);
  return pixels.Color(r, g, b);
}

void led_neo(uint8_t effect) {
  static uint8_t hue = 0;
  static int brightness = 0;
  static int fadeDirection = 1;
  static bool on = false;
  static unsigned long lastUpdate = 0;
  unsigned long currentMillis = millis();

  switch (effect) {
    case 1: // Tęcza
      if (currentMillis - lastUpdate >= 20) {
        lastUpdate = currentMillis;
        uint32_t color = pixels.ColorHSV(hue * 256, 255, MAX_BRIGHTNESS);
        pixels.setPixelColor(0, color);
        pixels.show();
        hue++;
      }
      break;

    case 2: // Miganie czerwone
      if (currentMillis - lastUpdate >= 500) {
        lastUpdate = currentMillis;
        on = !on;
        pixels.setPixelColor(0, on ? limitBrightness(255, 0, 0) : 0);
        pixels.show();
      }
      break;

    case 3: // Fade różowy
      if (currentMillis - lastUpdate >= 30) {
        lastUpdate = currentMillis;
        brightness += fadeDirection * 5;
        if (brightness <= 0 || brightness >= MAX_BRIGHTNESS) {
          fadeDirection *= -1;
        }
        pixels.setPixelColor(0, limitBrightness(brightness, 0, brightness));
        pixels.show();
      }
      break;

    case 4: // Stroboskop biały
      if (currentMillis - lastUpdate >= 100) {
        lastUpdate = currentMillis;
        on = !on;
        pixels.setPixelColor(0, on ? limitBrightness(255, 255, 255) : 0);
        pixels.show();
      }
      break;

    case 5: // Losowe kolory
      if (currentMillis - lastUpdate >= 300) {
        lastUpdate = currentMillis;
        uint8_t r = random(0, MAX_BRIGHTNESS + 1);
        uint8_t g = random(0, MAX_BRIGHTNESS + 1);
        uint8_t b = random(0, MAX_BRIGHTNESS + 1);
        pixels.setPixelColor(0, limitBrightness(r, g, b));
        pixels.show();
      }
      break;

    case 6: // Pulsujący niebieski
      if (currentMillis - lastUpdate >= 25) {
        lastUpdate = currentMillis;
        brightness += fadeDirection * 4;
        if (brightness <= 0 || brightness >= MAX_BRIGHTNESS) {
          fadeDirection *= -1;
        }
        pixels.setPixelColor(0, limitBrightness(0, 0, brightness));
        pixels.show();
      }
      break;

    case 7: // Zielony oddech
      if (currentMillis - lastUpdate >= 20) {
        lastUpdate = currentMillis;
        float breathe = (sin(millis() / 1000.0 * 2 * PI) + 1) / 2;
        int val = (int)(breathe * MAX_BRIGHTNESS);
        pixels.setPixelColor(0, limitBrightness(0, val, 0));
        pixels.show();
      }
      break;

    default:
      pixels.setPixelColor(0, 0);
      pixels.show();
      break;
  }
}
