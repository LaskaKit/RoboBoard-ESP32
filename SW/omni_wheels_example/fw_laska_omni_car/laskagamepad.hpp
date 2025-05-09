#ifndef LASKAGAMEPAD_HPP
#define LASKAGAMEPAD_HPP


#define GET_BIT(REG, BIT) ((REG) & (BIT))
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))


enum LaskaGamepadButton {
  CROSS = 0x1 << 4,
  CIRCLE = 0x1 << 3,
  TRIANGLE = 0x1 << 2,
  SQUARE = 0x1 << 5,
  UP = 0x1 << 12,
  DOWN = 0x1 << 10,
  RIGHT = 0x1 << 9,
  LEFT = 0x1 << 11,
  SELECT = 0x1 << 7,
  START = 0x1 << 6,
  L1 = 0x1 << 13,
  L2 = 0x1 << 14,
  L3 = 0x1 << 8,
  R1 = 0x1 << 1,
  R2 = 0x1,
  R3 = 0x1 << 15
};


struct LaskaGamepad {
  int16_t left_stick_x;
  int16_t left_stick_y;
  int16_t right_stick_x;
  int16_t right_stick_y;
  uint16_t buttons;

  bool get_button(LaskaGamepadButton button) {
    return GET_BIT(this->buttons, button);
  }
  void set_button(LaskaGamepadButton button, bool value) {
    this->set_clear_button_bit(value, button);
  }

private:
  void set_clear_button_bit(bool value, uint8_t bit) {
    value ? SET_BIT(this->buttons, 1) : CLEAR_BIT(this->buttons, 1);
  }
};

#endif  // LASKAGAMEPAD_HPP
