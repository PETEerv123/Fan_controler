#ifndef __ChuongTrinhChinh_h
#define __ChuongTrinhChinh_h


#include "01_WIFI.h"
#include "07_triac_devices.h"
#include "11_PWM.h"
#include "03_Flags.h"

#define Buzz        18
#define NutBoot     0
#define BTN_MOTOR   13   // D0 - Bật/tắt motor
#define BTN_UP      15   // D1 - Tăng tốc độ (+10%)
#define BTN_RELAY   2   // D2 - Bật/tắt relay
#define BTN_DOWN    4   // D3 - Giảm tốc độ (-10%)
#define Relay1      14
#define Relay2      12
#define LOAD        27
#define SCLK        26
#define SDI         25
#define LED         23
#define Key         35 //19
#define S2          32
#define S1          33
#define BOT         19  //21
#define TOP         23 //22
#define Store_currentlevel 599
#define Store_CurrrentRelay 603
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================== Begin: KHAI BÁO THƯ VIỆN ===========================================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#include <Arduino.h>
#include <Arduino_JSON.h>  // Thư viện xử lý dữ liệu kiểu JSON
#include <EEPROM.h>        // Thư viện để lưu dữ liệu thông số board vào bộ nhớ ROM
#include <Wire.h>          // Để kết nối I2C với mô-đun RTC (thời gian thực),
                           // mô-đun mở rộng port PCF8574, mô-đun đọc cảm biến nhiệt độ & độ ẩm SHT3x
#include <TimeLib.h>       // Thư viện xử lý các tính toán liên quan thời gian.
#include <74HC595_cus.h>
#include "driver/ledc.h"
#include "esp_timer.h"
#include "ACS712.h"
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================== End: KHAI BÁO THƯ VIỆN =============================================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
void ChuongTrinhChinh(void);  //
void KhoiTao(void);        
void ThucThiTacVuTheoFLAG(void);
void buzzer_beep(int BUZZER_GPIO, uint32_t freq, uint32_t duration_ms);
void stop_buzzer(void* arg);
void calibrateACS(void);
void Relay_Control(uint8_t Level_Relay);
void Encoder(void);
void buzzer_off(void);
void IRAM_ATTR handleKeyInterrupt(void);
#endif