#include "cap.h"
#include "adc.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "hrtim.h"
#include "tim.h"
#include "opamp.h"
#include "gpio.h"
#include "can.h"
#include "oled.h"

#define abs(x) 					((x)>0? (x):(-(x)))
#define max(a,b)                ((a)>(b) ? (a):(b))
#define min(a,b)                ((a)<(b) ? (a):(b))

#define cap_vol_max_limit 24.0f
#define cap_vol_min_limit 3.5f

float bat_power = 0;
float cap_power = 0;
float chas_power = 0;
uint32_t count = 0;
float efficiency = 0;
int16_t can_send_data[4] = {0};
uint32_t outtime[5] = {0,0,0,0,1000};
uint8_t pwm_switch = 1;     //可以在debug中控制pwm开关
uint8_t adjust_switch = 1;     //校准模式开关
uint16_t test11;
adc_data_t adc_data = {
    .bat_v_m = 1.0f,
    .bat_i_m = 1.0038f,
    .cap_v_m = 0.9851f,
    .cap_i_m = 0.8541f,
    .chas_v_m = 1.0f,
    .chas_i_m = 0.9841f,
    .bat_v_a = 0.0f,
    .bat_i_a = 0.0265f,
    .cap_v_a = 0.0154f,
    .cap_i_a = -0.1461f,
    .chas_v_a = 0.0f,
    .chas_i_a = -0.2886f,
};
receive_data_t receive_data;
cap_state_t cap_state;

//最大占空比和最小占空比可以一定程度上防止过充过放
pid_cap_i_t pid_cap_i = {
    .kp = 0,
    .ki = 0.001,
    .kd = 0,
    .integralmax = 1000,
    .integralmin = 150,
    .deadarea = 0,
    .OutDutyCycleMax = 1,
    .OutDutyCycleMin = 0.15,
    .PidSwitch = 0,
    .Filter_fac = 0,
};

//这里限制了最大充电电流和最大放电电流，注意积分上限和电流限制的倍数关系，要改同时改
pid_bat_power_t pid_bat_power = {
    .kp = 0.05,
    .ki = 0.005,
    .kd = 0,
    .integralmax = 2000,
    .integralmin = -2600,
    .deadarea = 0,
    .OutCurrentMax = 10,
    .OutCurrentMin = -13,
    .PidSwitch = 0,
    .Filter_fac = 0,
};

pid_powerbuffer_t pid_powerbuffer = {
    .kp = 1,
    .ki = 0.004,
    .kd = 0,
    .SetBuffer = 20,//20,
    .deadarea = 0,
    .PidSwitch = 0,
    .Filter_fac = 0,
};

//* 3.adc数据处理
float cap_v;
float cap_i;
void adc_solve()
{
	//* 电池电压
    adc_data.bat_v = (float)adc_data.adc_list_aver[0] / 4096 * 3.3f * 15 * adc_data.bat_v_m + adc_data.bat_v_a;
    //* 电池电流
	adc_data.bat_i = ((float)adc_data.adc_list_aver[3] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.bat_i_m + adc_data.bat_i_a;
    //* 电容电压
	adc_data.cap_v = (float)adc_data.adc_list_aver[1] / 4096 * 3.3f * 15 * adc_data.cap_v_m + adc_data.cap_v_a;
	//* 电容电流(TIM2_IRQHandler)
	//   adc_data.cap_i//在定时器中断里计算
	//* 底盘电压
    adc_data.chas_v = (float)adc_data.adc_list_aver[5] / 4096 * 3.3f * 15 * adc_data.chas_v_m + adc_data.chas_v_a;
    //* 底盘电流
	adc_data.chas_i = -((float)adc_data.adc_list_aver[2] / 4096 * 3.3f - 1.65f) / 0.062f * adc_data.chas_i_m + adc_data.chas_i_a;
    
	cap_v=(adc_data.cap_v)/0.333*5;
	cap_i=((adc_data.cap_i)-1.65)/0.075;
	
    //清零
    for(uint8_t i = 0; i < 4; i++)
    {
        adc_data.adc_list_sum[i] = 0;
    }
    adc_data.adc_list_sum[5] = 0;
    
    //求和
    for(uint8_t i = 0; i < aver_sample_len; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            adc_data.adc_list_sum[j] += adc_data.adc1_list_record[i][j];
        }
        adc_data.adc_list_sum[3] += adc_data.adc2_list_record[i][0];
        adc_data.adc_list_sum[5] += adc_data.adc2_list_record[i][2];
    }
    
    //取平均
	//* 用于上面的电压电流计算
    for(uint8_t i = 0; i < 4; i++)
    {
        adc_data.adc_list_aver[i] = adc_data.adc_list_sum[i] / aver_sample_len;
    }
    adc_data.adc_list_aver[5] = adc_data.adc_list_sum[5] / aver_sample_len;
}

//* 7.超电电流环pid
//* 目标:电容电流目标值(功率环环pid输出)  
//* 当前:当前电容电流
//* 输出:pwm占空比
void cap_pid_i()
{
    //判断是否运行该pid
    if(pid_cap_i.PidSwitch == 0){
		//* 大概积分值 = 电容电压/(电池电压*ki)
        pid_cap_i.integral = adc_data.cap_v / (adc_data.bat_v * pid_cap_i.ki);  //计算大概的积分累计值，避免过大的电流脉冲
        return;
    }
    
	//* 校准模式
    if(adjust_switch == 1)
    {
		//* 目标值赋值为电容充放电流目标值
        pid_cap_i.SetI = pid_bat_power.OutCurrent;  
    }
	//* 当前值为电容当前电流值
    pid_cap_i.ActualI = adc_data.cap_i;
    
    //计算偏差
    pid_cap_i.err = pid_cap_i.SetI - pid_cap_i.ActualI;

    //死区处理
//    if(abs(pid_cap_i.err) < pid_cap_i.deadarea)
//    {
//        return;
//    }
    
    //积分累加
    pid_cap_i.integral += pid_cap_i.err;
    
	//* 积分上线为定值
    if(pid_cap_i.integral > pid_cap_i.integralmax)
    {
        pid_cap_i.integral = pid_cap_i.integralmax;
    }
    else if(pid_cap_i.integral < pid_cap_i.integralmin)
    {
        pid_cap_i.integral = pid_cap_i.integralmin;
    }
    
    //计算输出
    pid_cap_i.pout = pid_cap_i.kp * pid_cap_i.err;
    pid_cap_i.iout = pid_cap_i.ki * pid_cap_i.integral;
    pid_cap_i.dout = pid_cap_i.kd * (pid_cap_i.err - pid_cap_i.err_last);
    
    float temp = pid_cap_i.pout + pid_cap_i.iout + pid_cap_i.dout;
    
    //限制输出上下限
	//* 输出上限为定值
    if(temp > pid_cap_i.OutDutyCycleMax)pid_cap_i.OutDutyCycle = pid_cap_i.OutDutyCycleMax;
    else if(temp < pid_cap_i.OutDutyCycleMin)pid_cap_i.OutDutyCycle = pid_cap_i.OutDutyCycleMin;
    else pid_cap_i.OutDutyCycle = temp;
    
    //一阶滞后滤波，防抖动
//    if(pid_cap_i.Filter_fac > 0.5f)
//    {
//        pid_cap_i.OutDutyCycle = pid_cap_i.LastOutDutyCycle * pid_cap_i.Filter_fac \
//                                   + pid_cap_i.OutDutyCycle * (1.0f - pid_cap_i.Filter_fac);
//        pid_cap_i.LastOutDutyCycle = pid_cap_i.OutDutyCycle;
//    }
    
    //记录本次偏差
    pid_cap_i.err_last = pid_cap_i.err;
    
	//* 打开某个神奇的通道
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道打开
    count ++;   //用来看pid运行次数，正常情况应该1秒增加10000
}

//* 5.电容充放电pid
//* 目标:电池功放目标值(能量缓冲环pid输出)  
//* 当前:当前电池功放
//* 输出:电容目标电流(取超电充电电流为正)
void cap_pid_bat_power()
{
	//* 设置状态机
    static uint8_t flag = 0;
	//* 计数器
    static uint16_t timetick = 0;
    //判断是否运行该pid
	//* 判断条件在错误处理回调函数
    if(pid_bat_power.PidSwitch == 0)
    {
		//* 积分和输出清零
        pid_bat_power.integral = 0;
        pid_bat_power.OutCurrent = 0;
		//* 状态机清零
        flag = 0;
        timetick = 0;
        return;
    }
    else if(pid_bat_power.PidSwitch == 1 && flag <= 1)
    {
        //软启动
		//* 计数器<500
        if(timetick < 500)
        {
			//* 电容电流小于2.5
            if(abs(adc_data.cap_i) < 2.5f)
            {
				//* 计数器计数
                timetick ++;
            }
			//* 积分和输出清零
            pid_bat_power.OutCurrent = 0;
            pid_bat_power.integral = 0;
            return;
        }
		//* 计数器500~1000
        else if(timetick < 1000)
        {
			//* 状态机置1(软起动状态)
            flag = 1;
            timetick ++;
        }
        else
        {
			//* 状态机置2
            flag = 2;
			//* 计数器清0
            timetick = 0;
        }
    }
    
	//* 赋值目标值为底盘功率限制pid的输出
    pid_bat_power.SetPower = pid_powerbuffer.OutPower;
	//* 计算电池当前功放作为当前值
    pid_bat_power.ActualPower = adc_data.bat_v * adc_data.bat_i;
    
    //计算偏差
	//* 注意控制量为超电的电流(取超电充电电流为正)
    pid_bat_power.err = pid_bat_power.SetPower - pid_bat_power.ActualPower;

    //死区处理
//    if(abs(pid_bat_power.err) < pid_bat_power.deadarea)
//    {
//        return;
//    }
    
    //积分累加
    pid_bat_power.integral += pid_bat_power.err;
    
    //限制积分上下限
	//* 积分上线为定值
    if(pid_bat_power.integral > pid_bat_power.integralmax)
    {
        pid_bat_power.integral = pid_bat_power.integralmax;
    }
    else if(pid_bat_power.integral < pid_bat_power.integralmin)
    {
        pid_bat_power.integral = pid_bat_power.integralmin;
    }
    
    //计算输出
    pid_bat_power.pout = pid_bat_power.kp * pid_bat_power.err;
    pid_bat_power.iout = pid_bat_power.ki * pid_bat_power.integral;
    pid_bat_power.dout = pid_bat_power.kd * (pid_bat_power.err - pid_bat_power.err_last);
    
    float temp = pid_bat_power.pout + pid_bat_power.iout + pid_bat_power.dout;
	//* 输出上下限为设置的定值和接收的充放电上限取最小最大
    float temp_max = min(pid_bat_power.OutCurrentMax, receive_data.input_power_limit / adc_data.cap_v);
    float temp_min = max(pid_bat_power.OutCurrentMin, -receive_data.output_power_limit / adc_data.cap_v);
    //限制输出上下限
    if(temp > temp_max)pid_bat_power.OutCurrent = temp_max;
    else if(temp < temp_min)pid_bat_power.OutCurrent = temp_min;
    else pid_bat_power.OutCurrent = temp;
    
    //软启动
	//* timetick(500~1000)
	//* ((float)(timetick - 500) / 500) -> (0,1)
	//* 软起动结果 = (pid计算结果-电容当前电流)*(0~1)时间系数+电容当前电流
	//* 随timetick(500~1000)的时间平滑充电
    if(flag == 1)
    {
        pid_bat_power.OutCurrent = (pid_bat_power.OutCurrent - adc_data.cap_i) * ((float)(timetick - 500) / 500) + adc_data.cap_i;
    }
    
    //限制过充过放
	//* 保护系数cap_vol_min_limit为定值
	//* 过放
    if(adc_data.cap_v < cap_vol_min_limit + 2.0f && pid_bat_power.OutCurrent < 0)
    {
        pid_bat_power.OutCurrent *= max(0.5f*(adc_data.cap_v - cap_vol_min_limit), 0);
    }
	//* 过充
    else if(adc_data.cap_v > cap_vol_max_limit - 2.0f && pid_bat_power.OutCurrent > 0)
    {
        pid_bat_power.OutCurrent *= max(0.5f*(cap_vol_max_limit - adc_data.cap_v), 0);
    }
    
    
    //一阶滞后滤波，防抖动
//    if(pid_bat_power.Filter_fac > 0.5f)
//    {
//        pid_bat_power.OutCurrent = pid_bat_power.LastOutCurrent * pid_bat_power.Filter_fac \
//                                   + pid_bat_power.OutCurrent * (1.0f - pid_bat_power.Filter_fac);
//        pid_bat_power.LastOutCurrent = pid_bat_power.OutCurrent;
//    }
    
    //记录本次偏差
    pid_bat_power.err_last = pid_bat_power.err;
}

//* 2.底盘功率限制pid
//* 目标:50J能量缓冲  
//* 当前:当前能量缓冲
//* 输出:电池功放
void cap_pid_powerbuffer()
{
    //注意这是正反馈pid，底盘功率缓冲越大裁判系统输出功率就要越大，计算偏差的方式与负反馈相反。
    //判断是否运行该pid
	//* 判断条件在error回调函数中
    if(pid_powerbuffer.PidSwitch == 0){
        pid_powerbuffer.integral = 0;
        return;
    }
    
    //根据裁判系统信息调节功率限制，取值范围在功率限制±5W，防止功率波动太大。
    //因为裁判系统能量缓冲0.1s才刷新一次，改变功率对能量缓冲的影响又有滞后性，所以这个pid在底盘功率快速变化时效果不好。
    //功率取值范围越小，pid波动越小，但对功率测量的准确度要求变高
	//* 赋值机器人底盘功率限制上限
    pid_powerbuffer.OutPowerMax = receive_data.chassis_power_limit + 5;
    pid_powerbuffer.OutPowerMin = receive_data.chassis_power_limit - 5;
	//* 赋值pid积分上限
    pid_powerbuffer.integralmax = pid_powerbuffer.OutPowerMax / pid_powerbuffer.ki;
    pid_powerbuffer.integralmin = pid_powerbuffer.OutPowerMin / pid_powerbuffer.ki;
	//* 当前底盘功率缓冲
    pid_powerbuffer.ActualBuffer = receive_data.chassis_power_buffer;
    
    //计算偏差
	//* 当前能量缓冲和目标能量缓冲(50定值)
    pid_powerbuffer.err = pid_powerbuffer.ActualBuffer - pid_powerbuffer.SetBuffer;
    
    if(pid_powerbuffer.err > 1)
    {
        pid_powerbuffer.err = pid_powerbuffer.err * pid_powerbuffer.err;
    }
    else if(pid_powerbuffer.err < -1)
    {
        pid_powerbuffer.err = -pid_powerbuffer.err * pid_powerbuffer.err;
    }

    //死区处理
    if(abs(pid_powerbuffer.err) < pid_powerbuffer.deadarea)
    {
        return;
    }
    
    //积分累加
    pid_powerbuffer.integral += pid_powerbuffer.err;
    
    //限制积分上下限
    if(pid_powerbuffer.integral > pid_powerbuffer.integralmax)
    {
        pid_powerbuffer.integral = pid_powerbuffer.integralmax;
    }
    else if(pid_powerbuffer.integral < pid_powerbuffer.integralmin)
    {
        pid_powerbuffer.integral = pid_powerbuffer.integralmin;
    }
    
    //计算输出
    pid_powerbuffer.pout = pid_powerbuffer.kp * pid_powerbuffer.err;
    pid_powerbuffer.iout = pid_powerbuffer.ki * pid_powerbuffer.integral;
    pid_powerbuffer.dout = pid_powerbuffer.kd * (pid_powerbuffer.err - pid_powerbuffer.err_last);
    
    float temp = pid_powerbuffer.pout + pid_powerbuffer.iout + pid_powerbuffer.dout;
    
    //限制输出上下限
    if(temp > pid_powerbuffer.OutPowerMax)pid_powerbuffer.OutPower = pid_powerbuffer.OutPowerMax;
    else if(temp < pid_powerbuffer.OutPowerMin)pid_powerbuffer.OutPower = pid_powerbuffer.OutPowerMin;
    else pid_powerbuffer.OutPower = temp;
    
    //低功率缓冲时快速恢复
    if(pid_powerbuffer.ActualBuffer < 20)
    {
		//* 防止底盘功率超限
        pid_powerbuffer.OutPower *= max(0.05f * (pid_powerbuffer.ActualBuffer - 22), 0);
    }
    
    //一阶滞后滤波，防抖动
//    if(pid_powerbuffer.Filter_fac > 0.5f)
//    {
//        pid_powerbuffer.OutPower = pid_powerbuffer.LastOutPower * pid_powerbuffer.Filter_fac \
//                                   + pid_powerbuffer.OutPower * (1.0f - pid_powerbuffer.Filter_fac);
//        pid_powerbuffer.LastOutPower = pid_powerbuffer.OutPower;
//    }
    
    //记录本次偏差
    pid_powerbuffer.err_last = pid_powerbuffer.err;
}

//* 8.把pid_cap_i.OutDutyCycle输出出去
void pwm_output()
{
    //这里参考安合科技的程序
    //控制思路在资料《基于STM32F334同步整流BUCK-BOOST数字电源设计》P8有详细说明
    uint32_t	buck_duty,boost_duty;
	uint32_t	PWM_PER_0_5	= 0.5f * hhrtim1.Instance->sTimerxRegs[0].PERxR;
    
	if(pid_cap_i.OutDutyCycle > 0.9f)  //当总占空比大于90%时，进入BOOST模式。
		boost_duty = (pid_cap_i.OutDutyCycle - 0.8f) * PWM_PER_0_5;//设置boost的占空比，并且模拟中心对称的PWM。注意要减去buck_duty的占空比: 0.8*PWM_PER_0_5
	else
		boost_duty = 0.1f * PWM_PER_0_5;	//当总占空比不大于90%时，进入BUCK模式,boost_duty给固定占空比

	if(pid_cap_i.OutDutyCycle > 0.9f)
		buck_duty = 0.9f * PWM_PER_0_5; //当总占空比大于90%时，进入BOOST模式,buck_duty给固定占空比
	else
		buck_duty = pid_cap_i.OutDutyCycle * PWM_PER_0_5;	//设置buck的占空比，并且模拟中心对称的PWM。

	hhrtim1.Instance->sTimerxRegs[0].CMP1xR = PWM_PER_0_5 - buck_duty;//TimerA PWM置1比较器，模拟中心对称的PWM，以PWM_PER_0_5为中心
    hhrtim1.Instance->sTimerxRegs[0].CMP2xR = PWM_PER_0_5 + buck_duty;//TimerA PWM置0比较器，比如  （400-50）与（400+50）的中心点为400	

	hhrtim1.Instance->sTimerxRegs[3].CMP1xR = PWM_PER_0_5 + boost_duty;//TimerD PWM置1比较器，模拟中心对称的PWM，以PWM_PER_0_5为中心
    hhrtim1.Instance->sTimerxRegs[3].CMP2xR = PWM_PER_0_5 - boost_duty;//TimerD PWM置0比较器，比如  （400-50）与（400+50）的中心点为400	
    
}

void system_init()
{
    //opamp
    HAL_OPAMP_Start(&hopamp2);
    //hrtim
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
//    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道打开
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //开启子定时器
    //adc
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data.adc1_list_record, 3 * aver_sample_len);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_data.adc2_list_record, 3 * aver_sample_len);
    //tim
    HAL_TIM_Base_Start_IT(&htim2);
    //can
    CAN_Init();
    //oled
    OLED_Init();
}

//* 9.错误检测
void error_check()
{
    static uint32_t timetick[5] = {0};
    outtime[0] = (adc_data.cap_v > 25) ? ++outtime[0] : 0;  //电容过压计时
    outtime[1] = (abs(adc_data.cap_i) > 14) ? ++outtime[1] : 0;  //电容过流计时
    outtime[2] = (adc_data.cap_v < 3) ? ++outtime[2] : 0;  //电容欠压计时
    outtime[3] = (adc_data.bat_v < 16) ? ++outtime[3] : 0;  //裁判系统欠压计时
    outtime[4]++;
    
    if(outtime[0] > 10)
    {
        cap_state.bit.cap_v_over = 1;
    }
    else
    {
        cap_state.bit.cap_v_over = 0;
    }
    
    if(outtime[1] > 3)
    {
        cap_state.bit.cap_i_over = 1;
        timetick[1] = 0;
    }
    else if(cap_state.bit.cap_i_over == 1)
    {
        timetick[1]++;
        if(timetick[1] > 3000)
        {
            timetick[1] = 0;
            cap_state.bit.cap_i_over = 0;
        }
    }
    
    if(outtime[2] > 10)
    {
        cap_state.bit.cap_v_low = 1;
    }
    else
    {
        cap_state.bit.cap_v_low = 0;
    }
    
    if(outtime[3] > 10)
    {
        cap_state.bit.bat_v_low = 1;
        timetick[3] = 0;
    }
    else if(cap_state.bit.bat_v_low == 1)
    {
        timetick[3]++;
        if(timetick[3] > 3000)
        {
            timetick[3] = 0;
            cap_state.bit.bat_v_low = 0;
        }
    }
    
    if(outtime[4] > 1000)
    {
		cap_state.bit.can_receive_miss = 1;//****1
        receive_data.cap_control.bit.cap_switch = 0;
    }
    else
    {
        cap_state.bit.can_receive_miss = 0;
    }
    test11=cap_state.bit.can_receive_miss;
	
    if(cap_state.bit.cap_i_over == 1 || cap_state.bit.bat_v_low == 1 || (receive_data.cap_control.bit.cap_switch == 0 && adjust_switch == 0) || pwm_switch == 0)
    {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道关闭
        pid_cap_i.PidSwitch = 0;
        pid_bat_power.PidSwitch = 0;
        pid_powerbuffer.PidSwitch = 0;
    }
    else
    {
        pid_cap_i.PidSwitch = 1;
        pid_bat_power.PidSwitch = 1;
        pid_powerbuffer.PidSwitch = 1;
    }
}

//* 6.控制板吞吐能量效率计算
void data_cal()
{
    //这里的效率只是控制板吞吐能量的效率，不包括电容内阻等损耗的效率。
    static uint16_t timetick = 0;
    static float powerin = 0;   //吸收功率
    static float powerout = 0;  //发出功率
    
    //大于0发出功率，小于0吸收功率
	//* 电池输出
    bat_power = adc_data.bat_v * adc_data.bat_i;
	//* 电容输出/输入
    cap_power = -adc_data.cap_v * adc_data.cap_i;
	//* 底盘输入
    chas_power = -adc_data.chas_v * adc_data.chas_i;
    
    bat_power > 0 ? (powerout += bat_power) : (powerin += bat_power);
    cap_power > 0 ? (powerout += cap_power) : (powerin += cap_power);
    chas_power > 0 ? (powerout += chas_power) : (powerin += chas_power);
    
	//* 100个节拍计算一次(10ms)
    timetick ++;
    if(timetick > 100)
    {
        timetick = 0;
		//* 控制板吞吐能量效率计算
        efficiency = powerin / powerout;
        powerin = 0;
        powerout = 0;
    }
}

void cap_loop()
{
    static uint8_t timetick = 0;
    if(timetick++ >= 20)
    {
        timetick = 0;
        cap_pid_powerbuffer();
        can_send();
    }
    adc_solve();
    cap_pid_bat_power();
    data_cal();
	//错误检测
    error_check();
	//调试
//    led_blue_set();
//    led_red_set();
//    oled_display();
}

//* 1.接收主控接收的裁判系统信息-> 底盘功率缓冲，机器人底盘功率限制上限，电容放电功率限制，电容充电功率限制
void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x02E)
    {
		//* 底盘功率缓冲
        receive_data.chassis_power_buffer = ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
        if(receive_data.chassis_power_buffer > 60)receive_data.chassis_power_buffer = 0;
    }
    else if(canId == 0x02F)
    {
		//* 机器人底盘功率限制上限
        receive_data.chassis_power_limit = ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
		//* 电容放电功率限制
        receive_data.output_power_limit = ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
		//* 电容充电功率限制
        receive_data.input_power_limit = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);
		//* 电容开关
        receive_data.cap_control.all = ((uint16_t)rxBuf[6] << 8| rxBuf[7]);
		//* 限幅
        if(receive_data.chassis_power_limit > 120)receive_data.chassis_power_limit = 120;
        if(receive_data.output_power_limit > 300 || receive_data.output_power_limit < -120)
        {
            receive_data.output_power_limit = 300;
        }
        if(receive_data.input_power_limit > 150)receive_data.input_power_limit = 150;
    }
    outtime[4] = 0;
}

//* 4.CAN发送
void can_send()
{
	//* 回传电容电压
    can_send_data[0] = float_to_int16(adc_data.cap_v, 30, 0, 32000, -32000);
	//* 回传电容电流
    can_send_data[1] = float_to_int16(adc_data.cap_i, 20, -20, 32000, -32000);
    can_send_data[2] = cap_state.state;
	
    CAN_SendData(0x030, can_send_data);
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}

void led_blue_set()
{
    if(adc_data.cap_v > 22.5f)
    {
        LED_BLUE_ON();
    }
    else
    {
        LED_BLUE_OFF();
    }
}

void led_red_set()
{
    if(adc_data.cap_v < 14.0f)
    {
        LED_RED_ON();
    }
    else
    {
        LED_RED_OFF();
    }
}

typedef struct
{
    float now;
    float max;
    float min;
    float aver;
    float start;
    float end;
    uint32_t count;
}f_value_cal_t;

f_value_cal_t cap_vol;
f_value_cal_t chas_pow;

typedef struct
{
    uint16_t now;
    uint16_t max;
    uint16_t min;
    float aver;
    uint16_t start;
    uint16_t end;
    uint32_t count;
}u8_value_cal_t;

u8_value_cal_t powerbuff;

void oled_display()
{
    static uint8_t record_state = 0;
    static uint32_t timetick = 0;
    static uint32_t display_timetick = 0;
    if(receive_data.cap_control.bit.cap_record == 1)
    {
        if(record_state == 0)
        {
            record_state = 1;
            timetick = 0;
            
            cap_vol.now = adc_data.cap_v;
            cap_vol.max = cap_vol.now;
            cap_vol.min = cap_vol.now;
            cap_vol.aver = cap_vol.now;
            cap_vol.start = cap_vol.now;
            cap_vol.end = 0;
            cap_vol.count = 1;
            
            chas_pow.now = chas_power;
            chas_pow.max = chas_pow.now;
            chas_pow.min = chas_pow.now;
            chas_pow.aver = chas_pow.now;
            chas_pow.count = 1;
            
            powerbuff.now = receive_data.chassis_power_buffer;
            powerbuff.max = powerbuff.now;
            powerbuff.min = powerbuff.now;
            powerbuff.aver = powerbuff.now;
            powerbuff.start = powerbuff.now;
            powerbuff.end = 0;
            powerbuff.count = 1;
        }
        else
        {
            timetick++;
            
            cap_vol.now = adc_data.cap_v;
            cap_vol.max = (cap_vol.now > cap_vol.max) ? cap_vol.now : cap_vol.max;
            cap_vol.min = (cap_vol.now < cap_vol.min) ? cap_vol.now : cap_vol.min;
            cap_vol.aver = (cap_vol.aver * cap_vol.count + cap_vol.now) / (cap_vol.count + 1);
            cap_vol.count++;
            
            chas_pow.now = adc_data.chas_v * adc_data.chas_i;
            chas_pow.max = (chas_pow.now > chas_pow.max) ? chas_pow.now : chas_pow.max;
            chas_pow.min = (chas_pow.now < chas_pow.min) ? chas_pow.now : chas_pow.min;
            chas_pow.aver = (chas_pow.aver * chas_pow.count + chas_pow.now) / (chas_pow.count + 1);
            chas_pow.count++;
            
            powerbuff.now = receive_data.chassis_power_buffer;
            powerbuff.max = (powerbuff.now > powerbuff.max) ? powerbuff.now : powerbuff.max;
            powerbuff.min = (powerbuff.now < powerbuff.min) ? powerbuff.now : powerbuff.min;
            powerbuff.aver = (powerbuff.aver * powerbuff.count + powerbuff.now) / (powerbuff.count + 1);
            powerbuff.count++;
        }
    }
    else
    {
        cap_vol.now = adc_data.cap_v;
        chas_pow.now = adc_data.chas_v * adc_data.chas_i;
        powerbuff.now = receive_data.chassis_power_buffer;
        if(record_state == 1)
        {
            record_state = 0;
            cap_vol.end = cap_vol.now;
            chas_pow.end = chas_pow.now;
            powerbuff.end = powerbuff.now;
        }
    }
    
    display_timetick++;
    if(display_timetick > 99)
    {
        display_timetick = 0;
        char ch[4][22] = {0};
        sprintf(ch[0], "U:%4.1f %4.1f %4.1f %4.1f", cap_vol.now, cap_vol.start, cap_vol.end, cap_vol.min);
        sprintf(ch[1], "P:%4.0f %4.0f %4.0f %4.0f", chas_pow.now, chas_pow.max, chas_pow.min, chas_pow.aver);
        sprintf(ch[2], "BF:%2d %2d %2d %2.0f %2d %2d", powerbuff.now, powerbuff.max, powerbuff.min, powerbuff.aver, powerbuff.start, powerbuff.end);
        sprintf(ch[3], "E:%-5.0f %.3f T:%.1f", chas_pow.aver * timetick / 1000, efficiency, (float)timetick / 1000);
//        sprintf(ch[0], "U:%4.1f %4.1f %4.1f %4.1f", 12.3f, 23.4f, 10.2f, 18.8f);
//        sprintf(ch[1], "P:%4.0f %4.0f %4.0f %4.0f", 123.5f, 288.1f, 11.2f, 198.8f);
//        sprintf(ch[2], "BF:%2d %2d %2d %2.0f %2d %2d", 40, 59, 32, 45.2f, 60, 45);
//        sprintf(ch[3], "E:%-6.0f T:%.3f", 88453.45f, 31.5888f);
        OLED_ShowString(0, 0, (uint8_t *)ch[0], 12);
        OLED_ShowString(0, 1, (uint8_t *)ch[1], 12);
        OLED_ShowString(0, 2, (uint8_t *)ch[2], 12);
        OLED_ShowString(0, 3, (uint8_t *)ch[3], 12);
    }
}




