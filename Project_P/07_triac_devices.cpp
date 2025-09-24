#include "07_triac_devices.h"


TRIAC_DEVICES::TRIAC_DEVICES()
:   Fan((gpio_num_t)FAN_PIN, TIMER_GROUP_0, TIMER_0)
    // Light((gpio_num_t)LIGHT_PIN,TIMER_GROUP_0,TIMER_1),
    // Uv((gpio_num_t)UV_PIN,TIMER_GROUP_1,TIMER_0)
{

}

void TRIAC_DEVICES::triac_devices_KhoiTao(void)
{
    
    /* Khởi tạo ACDET */
    triac::configACDETPIN((gpio_num_t)ACDET_PIN);

    /* Khởi tạo thiết bị triac */
    Fan.init();
    // Light.init();
    // Uv.init();

    /* Cài đặt góc kích lớn nhất để tắt các thiết bị triac khi mới khởi tạo */
    this->powerFan = TRIAC_HIGH_LIMIT;
    this->powerLight = TRIAC_HIGH_LIMIT;
    this->powerUV = TRIAC_HIGH_LIMIT;

    Fan.TurnOffTriac();
    // Light.TurnOffTriac();
    // Uv.TurnOffTriac();

}

void TRIAC_DEVICES::Bat_Quat(void)
{
    Fan.TurnOnTriac();
}

void TRIAC_DEVICES::Tat_Quat(void)
{
    Fan.TurnOffTriac();
}

void TRIAC_DEVICES::DieuKhienFan(uint16_t power_fan)
{
    this->powerFan = power_fan;
    Fan.SetTimeOverFlow(power_fan);
}


// void TRIAC_DEVICES::Bat_Den(void)
// {
//     Light.TurnOnTriac();
// }

// void TRIAC_DEVICES::Tat_Den(void)
// {
//     Light.TurnOffTriac();
// }

// void TRIAC_DEVICES::Bat_UV(void)
// {

//     Uv.TurnOnTriac();
// }

// void TRIAC_DEVICES::Tat_UV(void)
// {
//     Uv.TurnOffTriac();
// }

// void TRIAC_DEVICES::DieuKhienDen(uint16_t power_light)
// {
//     this->powerLight = power_light;
//     Light.SetTimeOverFlow(power_light);
// }

// void TRIAC_DEVICES::DieuKhienUV(uint16_t power_uv)
// {
//     this->powerUV = power_uv;
//     Uv.SetTimeOverFlow(power_uv);
// }

// void TRIAC_DEVICES::DieuKhienLight(uint16_t power_light)
// {
//     this->powerLight = power_light;
//     Light.SetTimeOverFlow(power_light);
// }

// void TRIAC_DEVICES::DieuKhienUV(uint16_t power_UV)
// {
//     this->powerUV = power_UV;
//     UV.SetTimeOverFlow(this->powerUV);
// }



