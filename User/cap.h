#ifndef __CAP_H
#define __CAP_H

#include "stdint.h"
#include "main.h"

#define aver_sample_len 10

typedef struct _adc_data_t{
    uint16_t adc_list_aver[6];
    uint32_t adc_list_sum[6];
    uint16_t adc1_list_record[aver_sample_len][3];
    uint16_t adc2_list_record[aver_sample_len][3];
    float bat_v;
    float bat_i;
    float cap_v;
    float cap_i;
    float chas_v;
    float chas_i;
    float bat_v_m;
    float bat_i_m;
    float cap_v_m;
    float cap_i_m;
    float chas_v_m;
    float chas_i_m;
    float bat_v_a;
    float bat_i_a;
    float cap_v_a;
    float cap_i_a;
    float chas_v_a;
    float chas_i_a;
}adc_data_t;

typedef struct
{
    uint16_t chassis_power_buffer;  //底盘功率缓冲
    uint16_t chassis_power_limit;   //机器人底盘功率限制上限
    int16_t output_power_limit;     //电容放电功率限制
    uint16_t input_power_limit;     //电容充电功率限制
    union{
        uint16_t all;
        struct
        {
            uint16_t cap_switch : 1;    //电容开关
            uint16_t cap_record : 1;    //记录功能开关
        }bit;
    }cap_control;
}receive_data_t;

typedef union
{
    uint16_t state;
    struct
    {
        uint16_t warning : 1;   //报警
        uint16_t cap_v_over : 1;    //电容过压
        uint16_t cap_i_over : 1;    //电容过流
        uint16_t cap_v_low : 1;     //电容欠压
        uint16_t bat_v_low : 1;     //裁判系统欠压
        uint16_t can_receive_miss : 1;    //未读到CAN通信数据
    }bit;
}cap_state_t;

typedef volatile struct _pid_cap_i{
    float SetI;
    float ActualI;
    float err;
    float err_last;
    float kp,ki,kd;
    float pout,iout,dout;
    float integral;
    float OutDutyCycle;
    float LastOutDutyCycle;
    float integralmax;
    float integralmin;
    float deadarea;
    float OutDutyCycleMax;
    float OutDutyCycleMin;
    uint8_t PidSwitch;
    float Filter_fac;
}pid_cap_i_t;

typedef volatile struct _pid_bat_power{
    float SetPower;
    float ActualPower;
    float err;
    float err_last;
    float kp,ki,kd;
    float pout,iout,dout;
    float integral;
    float OutCurrent;
    float LastOutCurrent;
    float integralmax;
    float integralmin;
    float deadarea;
    float OutCurrentMax;
    float OutCurrentMin;
    uint8_t PidSwitch;
    float Filter_fac;
}pid_bat_power_t;

typedef volatile struct _pid_powerbuffer{
    float SetBuffer;
    float ActualBuffer;
    float err;
    float err_last;
    float kp,ki,kd;
    float pout,iout,dout;
    float integral;
    float OutPower;
    float LastOutPower;
    float integralmax;
    float integralmin;
    float deadarea;
    float OutPowerMax;
    float OutPowerMin;
    uint8_t PidSwitch;
    float Filter_fac;
}pid_powerbuffer_t;

extern adc_data_t adc_data;

#define LED_RED_ON()		(HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET))
#define LED_RED_OFF()		(HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET))
#define LED_RED_TOGGLE()	(HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin))

#define LED_BLUE_ON()		(HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET))
#define LED_BLUE_OFF()		(HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET))
#define LED_BLUE_TOGGLE()	(HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin))

void cap_pid_i(void);
void system_init(void);
void cap_loop(void);
void pwm_output(void);
void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN_Init(void);
void can_send(void);
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
void led_blue_set(void);
void led_red_set(void);
void oled_display(void);

#endif

