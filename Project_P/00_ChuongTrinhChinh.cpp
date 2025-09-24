#include "00_ChuongTrinhChinh.h"

#define BLYNK_VIRTUAL_PIN_LED1 V1
#define BLYNK_VIRTUAL_PIN_LED2 V2
#define BLYNK_VIRTUAL_PIN_LED3 V3
#define BLYNK_VIRTUAL_PIN_LED4 V4
#define BLYNK_VIRTUAL_PIN_UP V3
#define BLYNK_VIRTUAL_PIN_DOWN V4
#define BLYNK_VIRTUAL_PIN_LEVEL V9
#define BLYNK_VIRTUAL_PIN_LED7 V7
#define BLYNK_VIRTUAL_PIN_Level_A V8

#define BLYNK_TEMPLATE_ID "TMPL6m28KhiYp"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "yaHae-goHilOqAF0VosJTSHLq8v1KCjF"

#include <BlynkSimpleEsp32.h>

_74HC595 _74HC595(SCLK, SDI, LOAD);
int PowerT[6] = {6000, 7000, 6300, 5500, 5000, 2700 };

#define HoldTime 3000
volatile uint8_t Water_Level;
int currentLevel;
int maxLevel = 5;
int minLevel = 1;
uint8_t level_Relay;
bool Flag_Relay1 = false;
bool Flag_Relay2 = false;
bool blynkConnected = false;
bool Flag_Power = false;
uint8_t _GiuBootDePhatAP;
bool _kiemtraketnoiwifi;
// float zeroVoltage = 0;
byte Mucnuoc[5] = {
  0b11110111,  // 1-
  0b10110111,  // 2-
  0b10110110,  // 3-
  0b10110110,  // 3-
  0b11111111,
};
Flags _Flags;
WIFI _WiFi;
String ID_Mac_Board;  // Số ID của ESP32, đây là số IMEI của board.
TRIAC_DEVICES _Triac_Devices;
PWM_LEDC PWM_Quat(GPIO_NUM_5, LEDC_CHANNEL_1, LEDC_TIMER_0, 4000);
ACS712 ACS(35, 3.3, 4095, 160.9);

TaskHandle_t _Button_Handle;
TaskHandle_t _control_system;
SemaphoreHandle_t EventHandle;

int lastState;
const int btnPins[] = { BTN_MOTOR, BTN_UP, BTN_RELAY, BTN_DOWN };


volatile bool KeyPress;
volatile unsigned long Presstime;
//(chương trình) Event Handle (ITR) -> call HandleKeyInternupt( check key was press ? )// when release button ->  call HandlekeyInterrupt (check key is High give key press dowwn)
// void IRAM_ATTR handleKeyInterrupt() {
//   if (digitalRead(Key) == LOW) {
//     Presstime = millis();
//     KeyPress = true;
//   } else {
//     // lúc thả ?
//     KeyPress = false;
//   }
// }
void IRAM_ATTR handleKeyInterrupt(void* arg) {
  // int level = gpio_get_level((gpio_num_t)Key);
  if (gpio_get_level((gpio_num_t)Key) == 0) {  // nhấn xuống
    Presstime = xTaskGetTickCountFromISR();    // an toàn hơn millis()
    KeyPress = true;
  } else {  // thả ra
    // releaseTime = xTaskGetTickCountFromISR();
    KeyPress = false;
  }
}
BLYNK_CONNECTED() {
#ifdef debug
  Serial.println("Connected to Blynk Cloud!");
#endif
  Blynk.syncAll();
}
BLYNK_WRITE(BLYNK_VIRTUAL_PIN_LED1) {
  // digitalWrite(led1, p);
  if (param.asInt()) {  // phát hiện nhấn nút
    Flag_Power = !Flag_Power;
#ifdef debug
    Serial.println(Flag_Power ? "Power ON" : "Power OFF");
#endif

    _Triac_Devices.DieuKhienFan(map(currentLevel, 1, 10, 10000, 0));
    _Triac_Devices.Bat_Quat();

    buzzer_beep(Buzz, 2000, 50);
  }
}

BLYNK_WRITE(BLYNK_VIRTUAL_PIN_LED2) {
  if (param.asInt() && Flag_Power == 1 && Water_Level != 0) {
    Flag_Relay1 = !Flag_Relay1;
    // Relay_Control(level_Relay);
    buzzer_beep(Buzz, 2000, 50);
  }
}
BLYNK_WRITE(BLYNK_VIRTUAL_PIN_LED7) {
  if (param.asInt() && Flag_Power == 1) {
    Flag_Relay2 = !Flag_Relay2;
    // Relay_Control(level_Relay);
    buzzer_beep(Buzz, 2000, 50);
  }
}
BLYNK_WRITE(BLYNK_VIRTUAL_PIN_UP) {
  if (param.asInt() == 1 && Flag_Power == 1) {
    if (currentLevel < maxLevel) {
      currentLevel++;


      _Triac_Devices.DieuKhienFan(map(currentLevel, 1, 10, 10000, 0));
      _Triac_Devices.Bat_Quat();


#ifdef debug
      Serial.print("Level Increased to: ");
      Serial.println(currentLevel);
#endif
      EEPROM.write(Store_currentlevel, currentLevel);
      EEPROM.commit();
      // #ifdef debug
      //       Serial.print("Đã lưu: ");
      //       Serial.println(currentLevel);
      // #endif
      buzzer_beep(Buzz, 2000, 50);
    }
  }
}

BLYNK_WRITE(BLYNK_VIRTUAL_PIN_DOWN) {
  if (param.asInt() == 1 && Flag_Power == 1) {
    if (currentLevel > 4) {
      currentLevel--;


      _Triac_Devices.DieuKhienFan(map(currentLevel, 1, 10, 10000, 0));
      _Triac_Devices.Bat_Quat();

#ifdef debug
      Serial.print("Level Decreased to: ");
      Serial.println(currentLevel);
#endif
      EEPROM.write(Store_currentlevel, currentLevel);
      EEPROM.commit();
      // #ifdef debug
      //       Serial.print("Đã lưu: ");
      //       Serial.println(currentLevel);
      // #endif
      buzzer_beep(Buzz, 2000, 50);
    }
  }
}
#pragma region Dual_Task
#pragma region handleButton
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ Begin: Hàm xử lý các tác vụ task =============================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
void Button_Handle(void* pvParameters) {
  static unsigned long PressingTime = 0;
  static bool longPresshold = false;
  static unsigned long lastReleaseTime = 0;
  static uint8_t clickCount;
  for (;;) {
    if (digitalRead(BTN_MOTOR)) {
      while (digitalRead(BTN_MOTOR)) { vTaskDelay(pdMS_TO_TICKS(1)); };
      Flag_Power = !Flag_Power;
#ifdef debug
      Serial.println(Flag_Power ? "Power ON" : "Power OFF");
#endif
      if (Flag_Power) {
        _Triac_Devices.DieuKhienFan(PowerT[currentLevel]);
#ifdef debug
        Serial.printf("Speed :%d\n", PowerT[currentLevel]);
#endif
        _Triac_Devices.Bat_Quat();
      }
      buzzer_beep(Buzz, 2000, 50);
    } else if (digitalRead(BTN_RELAY) && Flag_Power == 1) {
      while (digitalRead(BTN_RELAY)) { vTaskDelay(pdMS_TO_TICKS(1)); };
      // level_Relay++;
      // Flag_Relay1 = (level_Relay >> 0) & 0x01;  // lay bit cao
      // Flag_Relay2 = (level_Relay >> 1) & 0x01;
      // EEPROM.write(Store_CurrrentRelay, Flag_Relay1);
      // EEPROM.commit();
      buzzer_beep(Buzz, 2000, 50);
      goto Relay;
      // #ifdef debug
      //       Serial.printf("level Relay  %d\n", level_Relay);
      // #endif
      if (level_Relay >= 4) level_Relay = 0;
    } else if (digitalRead(BTN_UP) && Flag_Power == 1) {
      while (digitalRead(BTN_UP)) { vTaskDelay(pdMS_TO_TICKS(1)); };
      if (currentLevel < maxLevel) {
        currentLevel++;
        _Triac_Devices.DieuKhienFan(PowerT[currentLevel]);
#ifdef debug
        Serial.printf("Speed :%d\n", PowerT[currentLevel]);
#endif
        _Triac_Devices.Bat_Quat();
#ifdef debug
        Serial.print("Level Increased to: ");
        Serial.println(currentLevel);
#endif
        EEPROM.write(Store_currentlevel, currentLevel);
        EEPROM.commit();
        // #ifdef debug
        //       Serial.print("Đã lưu: ");
        //       Serial.println(currentLevel);
        // #endif
        buzzer_beep(Buzz, 2000, 50);
      }
    } else if (digitalRead(BTN_DOWN) && Flag_Power == 1) {
      while (digitalRead(BTN_DOWN)) { vTaskDelay(pdMS_TO_TICKS(1)); };
      if (currentLevel > minLevel) {
        currentLevel--;
        _Triac_Devices.DieuKhienFan(PowerT[currentLevel]);
#ifdef debug
        Serial.printf("Speed :%d\n", PowerT[currentLevel]);
#endif
        _Triac_Devices.Bat_Quat();

#ifdef debug
        Serial.print("Level Decreased to: ");
        Serial.println(currentLevel);
#endif
        EEPROM.write(Store_currentlevel, currentLevel);
        EEPROM.commit();
        buzzer_beep(Buzz, 2000, 50);
      }
    } else if (Flag_Power) {
      int currentState = digitalRead(S1);
      static int pulseCount = 0;  // đếm xung
      if (currentState != lastState) {
        pulseCount++;           // tăng mỗi khi có cạnh
        if (pulseCount >= 4) {  // đủ 2 xung mới xử lý
          pulseCount = 0;       // reset bộ đếm
          if (currentState != lastState) {
            if (digitalRead(S2) != currentState) {
              // Xoay phải
              if (currentLevel < maxLevel) {
                currentLevel++;
                _Triac_Devices.DieuKhienFan(PowerT[currentLevel]);
#ifdef debug
                Serial.printf("Speed :%d\n", PowerT[currentLevel]);
#endif
                _Triac_Devices.Bat_Quat();
                EEPROM.write(Store_currentlevel, currentLevel);
                EEPROM.commit();
                buzzer_beep(Buzz, 2000, 50);
              }
            } else {
              // Xoay trái
              if (currentLevel > minLevel) {
                currentLevel--;
                _Triac_Devices.DieuKhienFan(PowerT[currentLevel]);
#ifdef debug
                Serial.printf("Speed :%d\n", PowerT[currentLevel]);
#endif
                _Triac_Devices.Bat_Quat();
                EEPROM.write(Store_currentlevel, currentLevel);
                EEPROM.commit();
                buzzer_beep(Buzz, 2000, 50);
              }
            }
          }
        }
#ifdef debug
        Serial.printf("Current Level = %d\n", currentLevel);
#endif
      }
      lastState = currentState;
    }
//Encoder Region
#pragma region Encoder
    if (KeyPress) {
      if (millis() - Presstime >= HoldTime && !longPresshold) {
        Flag_Power = !Flag_Power;
#ifdef debug
        Serial.println(Flag_Power ? "Power ON" : "Power OFF");
#endif
        longPresshold = true;  // đánh dấu đã nhấn giữ tránh rơi bặt tắt liên tục
        buzzer_beep(Buzz, 2000, 50);
        if (Flag_Power) {

          _Triac_Devices.DieuKhienFan(PowerT[currentLevel]);
#ifdef debug
          Serial.printf("Speed :%d\n", PowerT[currentLevel]);
#endif
          _Triac_Devices.Bat_Quat();
        }
      }
    } else {
      if (Presstime > 0 && !longPresshold) {
        // level_Relay++;
        buzzer_beep(Buzz, 2000, 50);
Relay:
        if (Flag_Power) {
          clickCount++;
          lastReleaseTime = millis();
        }
        // if (level_Relay >= 4) level_Relay = 0;
      }
      Presstime = 0;  // reset lại thời gian nhấn , chờ thời gian nhấn tiếp theo
      longPresshold = false;
      // clickCount = 0;
    }  // khong luu trong lenh if cho khi nhan tha
    if (clickCount > 0 && millis() - lastReleaseTime >= 500) {
      if (clickCount == 1) Flag_Relay1 = !Flag_Relay1;
      else if (clickCount == 2) Flag_Relay2 = !Flag_Relay2;
      level_Relay = (Flag_Relay1 | Flag_Relay2 << 1);
      EEPROM.write(Store_CurrrentRelay, Flag_Relay1);
      EEPROM.commit();
      lastReleaseTime = 0;
      clickCount = 0;
    }
    // #ifdef debug
    //   Serial.printf("Current Relay1 :%d\n", level_Relay);
    // //    Serial.printf("click count:%d\n", clickCount);
    // #endif
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms
  }
}
#pragma endregion Encoder
#pragma endregion handleButton
#pragma region handle_App_Blynk
void Blynk_App(void* pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!blynkConnected) {
#ifdef debug
        Serial.println("Connecting to Blynk Cloud...!");
#endif
        if (Blynk.connect(5000)) {
          blynkConnected = true;
        } else {
#ifdef debug
          Serial.println("Connection failed. Try again later.");
#endif
        }
      }
      if (Blynk.connected()) {
        Blynk.run();
      } else {
        blynkConnected = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms
  }
}
void Wifi_Connected(void* pvParameters) {
  for (;;) {
    if (_GiuBootDePhatAP >= 10) {
      _WiFi.NgatKetNoiWiFi();
      _kiemtraketnoiwifi = true;
    }
    if (_WiFi.DaBatAP == true) {
      _kiemtraketnoiwifi = false;
    }
    _Flags.TurnONFlags();
    ThucThiTacVuTheoFLAG();
    _Flags.TurnOFFFlags();
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms
  }
}
#pragma endregion handle_App_Blynk
#pragma region handle_ACS712_Reading
void Water_Reading(void* pvParameters) {
  static unsigned long TimeWarn = 0;
  static bool Warning = false;
  static bool Timer_Warning = 0;
  for (;;) {
    Water_Level = (!digitalRead(TOP) << 1) | digitalRead(BOT);
    // #ifdef debug
    // Serial.printf("Logic Water %d\n", Water_Level);
    // #endif
    if (Flag_Power) {
      if ((Water_Level == 0 || Water_Level >= 2) && Warning == 0) {
        // TimeWarn = millis();
        Warning = true;
        buzzer_beep(Buzz, 2500, 5000);
      } else if (Water_Level == 0 && Warning) Relay_Control(0 | Flag_Relay2 << 1);  // xoa Relay 1
                                                                                    // if (TimeWarn > 0 && millis() - TimeWarn >= 10000 && Timer_Warning == false) {
                                                                                    //   TimeWarn = 0;
                                                                                    //   Warning = false;
                                                                                    //   Timer_Warning = true;
                                                                                    // }
      if (Water_Level == 1 && Warning) {
        Warning = false;
        // Timer_Warning = false;
        buzzer_off();
      }
      // else if (!Timer_Warning && Warning) buzzer_beep(Buzz, 1000, 50);
      // #ifdef debug
      //       Serial.println(Warning ? "Warnning on" : "Warning off");
      // #endif
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
#pragma endregion handle_ACS712_Reading
void Control_System(void* pvParameters) {
  static unsigned long powerOnTime = 0;
  static unsigned long _TimeBlink = 0;
  static bool _blink;
  byte digital;
  for (;;) {
    // if (xSemaphoreTake(EventHandle, portMAX_DELAY) == pdPASS) {
    if (Flag_Power) {
      // Nếu mới bật Power thì lưu thời điểm
      // if (powerOnTime == 0) {
      //   powerOnTime = millis();
      // }
      // if (millis() - powerOnTime <= 2000) {
      //   // Hiển thị logo "Power On" trong 2 giây
      //   _74HC595.Ma7doan_Display_Power_on();
      // } else {
      // Sau đó hiển thị mức relay
      powerOnTime = 0;
      if (Water_Level != 0) {
        Relay_Control(level_Relay);
      }
      // #ifdef debug
      //       Serial.printf("Currrent Relay :%d\n", level_Relay);
      //       Serial.printf("Toc Do Quat :%d\n", currentLevel);
      // #endif
      if (Flag_Relay1 == 1 && Water_Level != 0) {  // hiển thị mực nước 2/3
        digital = Mucnuoc[Water_Level % 10];
        // #ifdef debug
        //         Serial.println("Đã kích hoạt  hiển thị mực nước 2/3 Relay 1 on");
        // #endif
      } else if (!Flag_Relay1 && Water_Level != 0) {
        // #ifdef debug
        //         Serial.println("Đã kích hoạt  hiển thị  Relay 2 on , Relay 1 Off");
        // #endif
        digital = Mucnuoc[4];
      }  // hiển thị Relay2 nếu Relay 1 off
      else if (Water_Level == 0) {
        static unsigned long _TimeBlink = 0;
        if (millis() - _TimeBlink >= 400) {
          _TimeBlink = millis();
          _blink = !_blink;
          if (_blink) {
            digital = Mucnuoc[4];
          } else {
            digital = Mucnuoc[0];
          }
        }
      }
      _74HC595.Ma7doan_Display(digital & ~(Flag_Relay2 << 7), currentLevel);
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_LEVEL, map(currentLevel, minLevel, maxLevel, 1, 6));

      // else {
      // _Triac_Devices.DieuKhienFan(map(currentLevel, 1, 9, 10000, 0));
      // _Triac_Devices.Bat_Quat();
      // #ifdef debug
      //         Serial.printf("current Level:%d\n", currentLevel);
      // #endif
      // }
      // }
    } else {
      // Nếu Power tắt thì reset timer
      if (powerOnTime <= 0) {  // nếu power < = 0 thì Reset
        powerOnTime = millis();
      }
      Relay_Control(0);
      Flag_Relay2 = 0;
      level_Relay = Flag_Relay1;
      if (millis() - powerOnTime <= 3000) {
        _74HC595.Ma7doan_Display_Power_off();
      } else {
        _74HC595.Ma7doan_Display_OFF();
      }
      _Triac_Devices.Tat_Quat();
      // #ifdef debug
      //       Serial.printf("Flag Relay :%d\n", Flag_Relay1);
      // #endif
    }
    // }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ End: Hàm xử lý các tác vụ task =============================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#pragma endregion Dual_Task
#pragma region khoitao
void KhoiTao(void) {
  //----------------------------------------------------------------------------------------
  // Khởi tạo giao tiếp SERIAL
  Serial.begin(115200);
  Serial.println("");
  EEPROM.begin(605);
  delay(10);
  //----------------------------------------------------
#pragma region Khởi tạo WIFI
  //======================================================================
  //------ Begin: Khởi tạo để có thể cấu hình kết nối WiFi tự động -----//
  //======================================================================
  // Nếu muốn xóa thông tin WIFI đã lưu trong EEPROM thì mở dòng code này.
  // _WiFi.XoaWiFiDaLuuTrongEEPROM();
  // _ThongSoBoard.XoaEEPROM(0, 512);
  // Đoạn code này phải được gọi ở cuối cùng ở hàm setup().
  _WiFi.DocWiFiDaLuuTrongEEPROM();
  // Dành 10s để kết nối WiFI
  // Lưu ý: Phải có thời gian chờ cho việc kết nối WIFI nếu không sẽ
  // gây ra tình trạng board bị reset và không thể phát access point (AP).
  // EEPROM.write(Store_CurrrentRelay,0);

  // _WiFi.KetNoiWiFi(10);
  //======================================================================
  //------ End: Khởi tạo để có thể cấu hình kết nối WiFi tự động -------//
  //======================================================================
  ID_Mac_Board = _WiFi.LaySoMAC();
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  digitalWrite(Relay1, LOW);
  digitalWrite(Relay2, LOW);
  pinMode(LED, OUTPUT);
  // pinMode(Key, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S1, INPUT_PULLUP);
  pinMode(TOP, INPUT_PULLUP);
  pinMode(BOT, INPUT_PULLUP);
  gpio_config_t Key_pin_conf = {
    .pin_bit_mask = (uint64_t)(1ULL << (gpio_num_t)Key),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_ANYEDGE,
  };
  gpio_config(&Key_pin_conf);
  _74HC595.khoitao();
  // _74HC595.Ma7doan_Display_Power_off();
  ACS.autoMidPoint(50, 50);
  Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 80);
  _Triac_Devices.triac_devices_KhoiTao();
  currentLevel = constrain(EEPROM.read(Store_currentlevel), minLevel, maxLevel);
  Flag_Relay1 = constrain(EEPROM.read(Store_CurrrentRelay), 0, 1);
#ifdef debug
  Serial.printf("Level hiện tại : %d\n", currentLevel);
  Serial.printf("Relay1 Flag hiện tại : %d\n", level_Relay);
#endif
  // Serial.println(map(currentLevel, 1, 10, 8000, 4000));
  // _Triac_Devices.Tat_Quat();
  // Tat Khi vua bat dien
  // PWM_Quat.PWM_set_duty(1024);
  for (int i = 0; i < 4; i++) {
    pinMode(btnPins[i], INPUT_PULLUP);
  }
  EventHandle = xSemaphoreCreateBinary();
  // attachInterrupt(digitalPinToInterrupt(Key), handleKeyInterrupt, CHANGE);
  gpio_isr_handler_add((gpio_num_t)Key, handleKeyInterrupt, NULL);
  gpio_intr_enable((gpio_num_t)Key);
  // calibrateACS();
  xTaskCreatePinnedToCore(
    Button_Handle,   /* Task function. */
    "Button_Handle", /* name of task. */
    8000,            /* Stack size of task */
    NULL,            /* parameter of the task */
    2,               /* priority of the task */
    &_Button_Handle, /* Task handle to keep track of created task */
    0);              /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  // xTaskCreatePinnedToCore(
  //   Blynk_App,   /* Task function. */
  //   "Blynk_App", /* name of task. */
  //   6000,        /* Stack size of task */
  //   NULL,        /* parameter of the task */
  //   1,           /* priority of the task */
  //   &_Blynk_App, /* Task handle to keep track of created task */
  //   1);          /* pin task to core 1 */
  // xTaskCreate(
  //   Wifi_Connected,   /* Task function. */
  //   "Wifi Connected", /* name of task. */
  //   4096,             /* Stack size of task */
  //   NULL,             /* parameter of the task */
  //   3,                /* priority of the task */
  //   NULL);
  xTaskCreate(
    Water_Reading,    /* Task function. */
    "Water Reading ", /* name of task. */
    6000,             /* Stack size of task */
    NULL,             /* parameter of the task */
    2,                /* priority of the task */
    NULL);
  xTaskCreatePinnedToCore(
    Control_System,   /* Task function. */
    "control system", /* name of task. */
    8192,             /* Stack size of task */
    NULL,             /* parameter of the task */
    1,                /* priority of the task */
    &_control_system,
    1);
}
#pragma endregion khoitao
void ChuongTrinhChinh(void) {
  vTaskDelay(pdMS_TO_TICKS(1));  // 1ms
}
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ Begin: CÁC HÀM THỰC THI TÁC VỤ THEO FLAG =============================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#pragma region CÁC HÀM THỰC THI TÁC VỤ THEO FLAG
void ThucThiTacVuTheoFLAG(void) {
#pragma region ThucThiTacVuTheoFLAG
//------------------------------------------------------------------------------
#pragma region Flag100ms
#ifdef _Flag_100ms
  if (_Flags.Flag.t100ms) {
    // _WiFi.KiemTraKetNoiWiFi();
    //kiem tra ket noi wifi hoac tu thiet lap ket noi
    //wifi thong qua viec nhan nut boot
    //de giu nut boot se giup esp32 phat wifi
    //thiet lap wifi va luu vao eeproom
    //va tu dong Reset khi da luu thanh cong
    if (digitalRead(NutBoot) == LOW) {
      _GiuBootDePhatAP++;
#ifdef debug
      Serial.print("NHAN GIU LAN THU: ");
      Serial.println(_GiuBootDePhatAP);
#endif
    }
  }
#endif
#pragma endregion Flag100ms
//------------------------------------------------------------------------------
#pragma region Flag500ms
#ifdef _Flag_500ms
  if (_Flags.Flag.t500ms) {
    // int mA = ACS.mA_AC(50, 1);
    // Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_Level_A, (double)mA);
    // #ifdef debug
    //     Serial.printf("Current Load AC : %d\n", mA);
    // #endif
    _WiFi._KiemTraKetNoiWiFi();
  }
#endif
#pragma endregion Flag500ms
#pragma endregion ThucThiTacVuTheoFLAG
}
#pragma endregion CÁC HÀM THỰC THI TÁC VỤ THEO FLAG
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ End: CÁC HÀM THỰC THI TÁC VỤ THEO FLAG ===============================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ Begin: CÁC HÀM THỰC THI TÁC BĂM XUNG ChO CÒI =========================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#pragma region tạo_PWM_không_chiếm_dụng_CPU
static esp_timer_handle_t buzzer_timer;
// Hàm tắt buzzer
void stop_buzzer(void* arg) {
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

// Hàm phát 1 beep bằng PWM
void buzzer_beep(int BUZZER_GPIO, uint32_t freq, uint32_t duration_ms) {
  // Cấu hình PWM
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
  ledc_timer.freq_hz = freq;
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {};
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.duty = 512;  // duty 50%
  ledc_channel.gpio_num = BUZZER_GPIO;
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_channel.hpoint = 0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_channel);

  // Dùng esp_timer non-blocking để tắt sau duration_ms
  if (buzzer_timer) {
    esp_timer_stop(buzzer_timer);
    esp_timer_delete(buzzer_timer);
  }

  const esp_timer_create_args_t buzzer_timer_args = {
    .callback = &stop_buzzer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "buzzer_timer"
  };
  esp_timer_create(&buzzer_timer_args, &buzzer_timer);
  esp_timer_start_once(buzzer_timer, duration_ms * 1000);
}
void buzzer_off(void) {
  // Ngừng PWM trên channel
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  // Nếu timer vẫn còn tồn tại thì xóa đi để tránh tự gọi stop_buzzer() lại
  if (buzzer_timer) {
    esp_timer_stop(buzzer_timer);
    esp_timer_delete(buzzer_timer);
    buzzer_timer = NULL;
  }
}

#pragma endregion tạo_PWM_không_chiếm_dụng_CPU

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ END: CÁC HÀM THỰC THI TÁC BĂM XUNG ChO CÒI =========================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

// void calibrateACS(void) {
//   long sum = 0;
//   for (int i = 0; i < 1000; i++) {
//     sum += analogRead(35);
//     delayMicroseconds(200);
//   }
//   int avg = sum / 1000;
//   zeroVoltage = avg * (3.3 / 4095.0);  // Convert ADC to voltage
// }

void Relay_Control(uint8_t Level_) {
  bool _Flag_Relay1_ = (Level_ >> 0) & 0x01;
  bool _Flag_Relay2_ = (Level_ >> 1) & 0x01;  // 10
  digitalWrite(Relay1, _Flag_Relay1_);
  digitalWrite(Relay2, _Flag_Relay2_);
  // #ifdef debug
  //   Serial.print(Flag_Relay1 ? "Relay1 On, " : "Relay1 Off, ");
  //   Serial.println(Flag_Relay2 ? "Relay2 On" : "Relay2 Off");
  // #endif
}
// void Encoder(void) {
//   static bool longPresshold = false;
//   static unsigned long lastReleaseTime = 0;
//   static uint8_t clickCount;
//   if (KeyPress) {
//     if (millis() - Presstime >= HoldTime && !longPresshold) {
//       Flag_Power = !Flag_Power;
// #ifdef debug
//       Serial.println(Flag_Power ? "Power ON" : "Power OFF");
// #endif
//       longPresshold = true;  // đánh dấu đã nhấn giữ tránh rơi bặt tắt liên tục
//       buzzer_beep(Buzz, 2000, 50);
//       if (Flag_Power) {
//         if (currentLevel != 10) {
//           _Triac_Devices.DieuKhienFan(map(currentLevel, 1, 9, 10000, 1500));
//         } else gpio_set_level(GPIO_NUM_5, 1);
//         _Triac_Devices.Bat_Quat();
//       }
//     }
//   } else {
//     if (Presstime > 0 && !longPresshold) {
//       // level_Relay++;
//       buzzer_beep(Buzz, 2000, 50);
//       if (Flag_Power) {
//         clickCount++;
//         lastReleaseTime = millis();
//       }
//       // if (level_Relay >= 4) level_Relay = 0;
//     }
//     Presstime = 0;  // reset lại thời gian nhấn , chờ thời gian nhấn tiếp theo
//     longPresshold = false;
//     // clickCount = 0;
//   }  // khong luu trong lenh if cho khi nhan tha
//   if (clickCount > 0 && millis() - lastReleaseTime >= 500) {
//     if (clickCount == 1) Flag_Relay1 = !Flag_Relay1;
//     else if (clickCount == 2) Flag_Relay2 = !Flag_Relay2;
//     level_Relay = (Flag_Relay1 | Flag_Relay2 << 1);
//     EEPROM.write(Store_CurrrentRelay, Flag_Relay1);
//     EEPROM.commit();
//     lastReleaseTime = 0;
//     clickCount = 0;
//   }
//   // #ifdef debug
//   //   Serial.printf("Current Relay1 :%d\n", level_Relay);
//   // //    Serial.printf("click count:%d\n", clickCount);
//   // #endif
// }
