#ifndef HC595_H
#define HC595_H


// #include "driver/gpio.h"
#include "SPI.h"
#include "Arduino.h"

class _74HC595 {
public:
  int pin_SCK, pin_MOSI, pin_LOAD;

  //dua cac chan can vao 
  _74HC595(int pin_SCK,int pin_MOSI ,int pin_LOAD):
  pin_SCK(pin_SCK),pin_MOSI(pin_MOSI),pin_LOAD(pin_LOAD){}
  void Ma7doan_Display_Power_off(void);
  void Ma7doan_Display_Power_on(void);
  void Ma7doan_Display(byte LED1,int LED2);
  void khoitao(void);
  void Ma7doan_Display_OFF(void);
private:
};

#endif
