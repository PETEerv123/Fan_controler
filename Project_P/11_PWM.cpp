#include "11_PWM.h"


  PWM_LEDC::PWM_LEDC( gpio_num_t pin,ledc_channel_t channel, ledc_timer_t timer_sel, uint32_t freq_hz )
{
    this->pin = pin;
    this->channel = channel;
    this->timer_sel = timer_sel;
    this->freq_hz = freq_hz;
}

void PWM_LEDC::PWM_Chanel_Config( gpio_num_t pin, ledc_channel_t channel, uint32_t duty, ledc_intr_type_t intr_type, ledc_timer_t timer_sel, ledc_mode_t speed_mode, int hpoint)
{
    this->pin = pin;
    this->channel = channel;
    this->duty = duty;
    this->intr_type = intr_type;
    this->timer_sel = timer_sel;
    this->speed_mode = speed_mode; 
    this->hpoint = hpoint;
}   

void PWM_LEDC::PWM_Timer_Config( ledc_mode_t speed_mode, ledc_timer_bit_t duty_resolution, ledc_timer_t timer_num, uint32_t freq_hz, ledc_clk_cfg_t clk_cfg)
{
    this->speed_mode = speed_mode;
    this->duty_resolution = duty_resolution;
    this->timer_sel = timer_num;
    this->freq_hz = freq_hz;
    this->clk_cfg = clk_cfg;
}

void PWM_LEDC::PWM_init(void)
{
    ledc_channel_config_t PWM_channel_t = {
        .gpio_num = (int)this->pin,
        .speed_mode = this->speed_mode,
        .channel = this->channel,
        .intr_type = this->intr_type,
        .timer_sel = this->timer_sel,
        .duty = this->duty,
        .hpoint = this->hpoint,
    };

    ledc_channel_config( &PWM_channel_t );
    PWM_Timer_Config( this->speed_mode, this->duty_resolution, this->timer_sel, this->freq_hz, this->clk_cfg );

    ledc_timer_config_t PWM_timer_t = {
        .speed_mode = this->speed_mode,
        .duty_resolution = this->duty_resolution,
        .timer_num = this->timer_sel,
        .freq_hz = this->freq_hz,
        .clk_cfg = this->clk_cfg,
    };

    ledc_timer_config( &PWM_timer_t );
}

void PWM_LEDC::PWM_set_duty(uint32_t duty)
{
  this->duty = duty;
  ledc_set_duty(this->speed_mode, this->channel, this->duty);
  ledc_update_duty(this->speed_mode, this->channel);
}

void PWM_LEDC::PWM_set_duty_hpoint(uint32_t duty, uint32_t hpoint)
{
    this->hpoint = hpoint;
    this->duty = duty;
    ledc_set_duty_with_hpoint(this->speed_mode, this->channel, this->duty, this->hpoint);
    ledc_update_duty(this->speed_mode, this->channel);
}

void PWM_LEDC::PWM_set_freq(uint32_t freq)
{
    ledc_set_freq(this->speed_mode, this->timer_sel, this->freq_hz);
}

/**
 * @brief Pause LEDC timer counter.
 */
void PWM_LEDC::PWM_pause(void)
{
    ledc_set_duty(this->speed_mode, this->channel,0);
    ledc_update_duty(this->speed_mode, this->channel);
}

/**
 * @brief Resume LEDC timer.
 */
void PWM_LEDC::PWM_resume(void)
{
    PWM_set_duty(this->duty);
}