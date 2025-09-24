#ifndef __TRIAC_DEVICES_H__
#define __TRIAC_DEVICES_H__

#include "12_triac.h"

#define     FAN_PIN     5
#define     ACDET_PIN   34


class TRIAC_DEVICES : public triac
{
    public:
    triac Fan;
    triac Light;
    triac Uv;

    /* Thời gian kích của từng thiết bị */
    uint16_t powerFan = TRIAC_HIGH_LIMIT;
    uint16_t powerLight = TRIAC_HIGH_LIMIT;
    uint16_t powerUV = TRIAC_HIGH_LIMIT;    

    TRIAC_DEVICES();

    void triac_devices_KhoiTao(void);
    void Bat_Quat(void);
    void Tat_Quat(void);
    void DieuKhienFan(uint16_t power_fan);
    // void Bat_Den(void);
    // void Tat_Den(void);
    // void Bat_UV(void);
    // void Tat_UV(void);
    // void DieuKhienDen(uint16_t power_light);
    // void DieuKhienUV(uint16_t power_uv);

    // void DieuKhienLight(uint16_t power_light);
    // void DieuKhienUV(uint16_t power_UV);

};





#endif /* __TRIAC_DEVICES_H__ */