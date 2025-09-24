#include "hal/ledc_types.h"
#include "driver/ledc.h"
#include "hal/gpio_types.h"
#ifndef __PWM_H__
#define __PWM_H__

class PWM_LEDC
{
    public:

    /* Configuration parameters of LEDC */
    int pin;                                                    /* All GPIO pins except input-only pins */
    ledc_timer_bit_t duty_resolution = LEDC_TIMER_10_BIT;       /* the range of duty setting is [0, (2**duty_resolution)] : 10 bit : 1024 */  
    ledc_intr_type_t intr_type = LEDC_INTR_DISABLE;             /* Default : OFF */
    ledc_timer_t timer_sel = LEDC_TIMER_0;                      /* Default : Timer_0 -- Max Timer : 4  */
    ledc_channel_t channel;                                     /* 16 channels : 8 channels in high speed mode/8 channles in low speed mode */                            
    uint32_t duty = 0;
    ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;               /* Default : Low_Speed */   
    int hpoint = 0;
    uint32_t freq_hz;
    ledc_clk_cfg_t clk_cfg = LEDC_AUTO_CLK ;                    /* Automatically select the source clock(REF_TICK or APB) */

    PWM_LEDC( gpio_num_t pin,ledc_channel_t channel, ledc_timer_t timer_sel, uint32_t freq_hz );
    void PWM_Chanel_Config( gpio_num_t pin, ledc_channel_t channel, uint32_t duty, ledc_intr_type_t intr_state, ledc_timer_t timer_sel, ledc_mode_t speed_mode, int hpoint);
    void PWM_Timer_Config( ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, ledc_timer_t timer_num, uint32_t freq_hz, ledc_clk_cfg_t clk_cfg);

    void PWM_init(void);

    void PWM_set_duty(uint32_t duty);
    void PWM_set_duty_hpoint(uint32_t duty, uint32_t hpoint);
    void PWM_set_freq(uint32_t freq);
    
    void PWM_pause(void);
    void PWM_resume(void);
};

#endif /* __PWM_H__ */


