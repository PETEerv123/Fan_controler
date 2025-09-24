#include "74HC595_cus.h"

byte Ma7doan[14] = {
  0b11000000,  // 0
  0b11111001,  // 1
  0b10100100,  // 2
  0b10110000,  // 3
  0b10011001,  // 4
  0b10010010,  // 5
  0b10000010,  // 6
  0b11111000,  // 7
  0b10000000,  // 8
  0b10010000,  // 9
  0b10100011,  // o.
  0b00101011,  // n.
  0b00001110,  // F.
  0b11111111,  // turnoff
};
// byte Mucnuoc[4] = {
//   0b11110111,  // 1-
//   0b10110111,  // 2-
//   0b10110110,  // 3-
//   0b10110110,  // 3-
// };
void _74HC595::Ma7doan_Display(byte LED1, int LED2) {
  // int hang_donvi_LED1 = LED1 % 10;
  int hang_donvi_LED2 = LED2 % 10;
  digitalWrite(pin_LOAD, LOW);
  // if (hang_chuc < 10) hang_chuc = 13; // off
  SPI.transfer(Ma7doan[hang_donvi_LED2]);                // LED 2 // hang don vi
  
  // SPI.transfer(Ma7doan[hang_donvi_LED1] & 0b01111111);  // LED 1 // hang chuc
  SPI.transfer(LED1);
  digitalWrite(pin_LOAD, HIGH);
}
void _74HC595::Ma7doan_Display_Power_on(void) {
  digitalWrite(pin_LOAD, LOW);
  SPI.transfer(Ma7doan[11]);  // LED 2 // hang don vi
  SPI.transfer(Ma7doan[10]);  // LED 1 // hang chuc
  digitalWrite(pin_LOAD, HIGH);
}
void _74HC595::Ma7doan_Display_Power_off(void) {
  digitalWrite(pin_LOAD, LOW);
  SPI.transfer(Ma7doan[12]);  // LED 2 // hang don vi
  SPI.transfer(Ma7doan[10]);  // LED 1 // hang chuc
  digitalWrite(pin_LOAD, HIGH);
}
void _74HC595::Ma7doan_Display_OFF(void) {
  digitalWrite(pin_LOAD, LOW);
  SPI.transfer(Ma7doan[13]);  // LED 2 // hang don vi
  SPI.transfer(Ma7doan[13]);  // LED 1 // hang chuc
  digitalWrite(pin_LOAD, HIGH);
}
void _74HC595::khoitao(void) {
  // gpio_config_t load_config = {
  //     .pin_bit_mask = (1ULL << pin_LOAD),
  //     .mode = GPIO_MODE_OUTPUT,
  // };

  // gpio_config(&load_config);
  // gpio_set_level((gpio_num_t)pin_LOAD,0);
  // gpio_set_level((gpio_num_t)pin_LOAD,1);
  pinMode(pin_LOAD, OUTPUT);
  digitalWrite(pin_LOAD, HIGH);

  SPI.begin(pin_SCK, -1, pin_MOSI);
}