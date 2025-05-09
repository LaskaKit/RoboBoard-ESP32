#include <Adafruit_NeoPixel.h>

struct Beacon {
  Adafruit_NeoPixel leds;
  uint16_t leds_state;
  
  Beacon(uint8_t pin) {
    this->leds = Adafruit_NeoPixel(pin, 16, NEO_GRB + NEO_KHZ800);
    this->leds.setBrightness(100);
    this->leds.begin();
  }

  void write() {
    for (int i = 0; i < 16; i++) {
      uint32_t color = this->leds.Color(0, 0, 255);
      if (i >= 4 && i < 12) {
        color = this->leds.Color(255, 0, 0);
      }
      if (this->leds_state & (1 << i)) {
        this->leds.setPixelColor(i, color);
      } else {
        this->leds.setPixelColor(i, this->leds.Color(0, 0, 0));
      }
    }
    this->leds.show();
  }
};


class LeftRightBeaconPattern {
  private:
    Beacon* beacon;
    bool direction;
    uint8_t state;

  public:
    LeftRightBeaconPattern(Beacon* beacon)
      : beacon(beacon)
    {}

    void init() {
      this->beacon->leds_state = 3 | (3 << (8 + 6));
      this->direction = false;
      this->state = 0;
    }

    void step() {
      if (this->state >= 6) {
        this->direction = !this->direction;
        this->state = 0;
      }
      if (!this->direction) {
        this->beacon->leds_state = (this->beacon->leds_state & 0x00FF) << 1 |
                                  (this->beacon->leds_state & 0xFF00) >> 1;
      } else {
        this->beacon->leds_state = (this->beacon->leds_state & 0x00FF) >> 1 |
                                  (this->beacon->leds_state & 0xFF00) << 1;
      }
      this->state++;
    }
};
