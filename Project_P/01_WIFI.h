#ifndef _WiFi_h
#define _WiFi_h

#include "Project_P.h"
#include <Arduino.h>    // Để khai báo kiểu String

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//===================== Begin: classWiFi ==================================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#pragma region classWiFi
class WIFI {
public:
  bool DaBatAP;
  String LaySoMAC(void);
  void KetNoiWiFi(int ThoiGianChoKetNoi);  // Kết nối WiFi.
  int TinhDoManhCuaWiFi(void);

  void DocWiFiDaLuuTrongEEPROM(void);
  void _KiemTraKetNoiWiFi(void);
  void NgatKetNoiWiFi(void);
private:
  static String TenWiFi;
  static String MatKhauWiFi;
  String AP_SSID = "IoTvision";
  String AP_PASS = "IoTvision@2023";
  ///
  static String rssiToIcon(int rssi);
  static void ServerON(void);
  static void QuetTimWiFi(void);
  static void TrangChu(void);
  static void StyleCSS(void);
  static void CodeJS(void);
  static void LuuWiFiVaoEEPROM(void);
  void ThietLapAP(void);
};
#pragma endregion classWiFi
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//===================== End: classWiFi ====================================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

#endif
