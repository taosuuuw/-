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
uint8_t pwm_switch = 1;     //������debug�п���pwm����
uint8_t adjust_switch = 1;     //У׼ģʽ����
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

//���ռ�ձȺ���Сռ�ձȿ���һ���̶��Ϸ�ֹ�������
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

//�����������������������ŵ������ע��������޺͵������Ƶı�����ϵ��Ҫ��ͬʱ��
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

//* 3.adc���ݴ���
float cap_v;
float cap_i;
void adc_solve()
{
	//* ��ص�ѹ
    adc_data.bat_v = (float)adc_data.adc_list_aver[0] / 4096 * 3.3f * 15 * adc_data.bat_v_m + adc_data.bat_v_a;
    //* ��ص���
	adc_data.bat_i = ((float)adc_data.adc_list_aver[3] / 4096 * 3.3f - 1.65f) / 0.075f * adc_data.bat_i_m + adc_data.bat_i_a;
    //* ���ݵ�ѹ
	adc_data.cap_v = (float)adc_data.adc_list_aver[1] / 4096 * 3.3f * 15 * adc_data.cap_v_m + adc_data.cap_v_a;
	//* ���ݵ���(TIM2_IRQHandler)
	//   adc_data.cap_i//�ڶ�ʱ���ж������
	//* ���̵�ѹ
    adc_data.chas_v = (float)adc_data.adc_list_aver[5] / 4096 * 3.3f * 15 * adc_data.chas_v_m + adc_data.chas_v_a;
    //* ���̵���
	adc_data.chas_i = -((float)adc_data.adc_list_aver[2] / 4096 * 3.3f - 1.65f) / 0.062f * adc_data.chas_i_m + adc_data.chas_i_a;
    
	cap_v=(adc_data.cap_v)/0.333*5;
	cap_i=((adc_data.cap_i)-1.65)/0.075;
	
    //����
    for(uint8_t i = 0; i < 4; i++)
    {
        adc_data.adc_list_sum[i] = 0;
    }
    adc_data.adc_list_sum[5] = 0;
    
    //���
    for(uint8_t i = 0; i < aver_sample_len; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            adc_data.adc_list_sum[j] += adc_data.adc1_list_record[i][j];
        }
        adc_data.adc_list_sum[3] += adc_data.adc2_list_record[i][0];
        adc_data.adc_list_sum[5] += adc_data.adc2_list_record[i][2];
    }
    
    //ȡƽ��
	//* ��������ĵ�ѹ��������
    for(uint8_t i = 0; i < 4; i++)
    {
        adc_data.adc_list_aver[i] = adc_data.adc_list_sum[i] / aver_sample_len;
    }
    adc_data.adc_list_aver[5] = adc_data.adc_list_sum[5] / aver_sample_len;
}

//* 7.���������pid
//* Ŀ��:���ݵ���Ŀ��ֵ(���ʻ���pid���)  
//* ��ǰ:��ǰ���ݵ���
//* ���:pwmռ�ձ�
void cap_pid_i()
{
    //�ж��Ƿ����и�pid
    if(pid_cap_i.PidSwitch == 0){
		//* ��Ż���ֵ = ���ݵ�ѹ/(��ص�ѹ*ki)
        pid_cap_i.integral = adc_data.cap_v / (adc_data.bat_v * pid_cap_i.ki);  //�����ŵĻ����ۼ�ֵ���������ĵ�������
        return;
    }
    
	//* У׼ģʽ
    if(adjust_switch == 1)
    {
		//* Ŀ��ֵ��ֵΪ���ݳ�ŵ���Ŀ��ֵ
        pid_cap_i.SetI = pid_bat_power.OutCurrent;  
    }
	//* ��ǰֵΪ���ݵ�ǰ����ֵ
    pid_cap_i.ActualI = adc_data.cap_i;
    
    //����ƫ��
    pid_cap_i.err = pid_cap_i.SetI - pid_cap_i.ActualI;

    //��������
//    if(abs(pid_cap_i.err) < pid_cap_i.deadarea)
//    {
//        return;
//    }
    
    //�����ۼ�
    pid_cap_i.integral += pid_cap_i.err;
    
	//* ��������Ϊ��ֵ
    if(pid_cap_i.integral > pid_cap_i.integralmax)
    {
        pid_cap_i.integral = pid_cap_i.integralmax;
    }
    else if(pid_cap_i.integral < pid_cap_i.integralmin)
    {
        pid_cap_i.integral = pid_cap_i.integralmin;
    }
    
    //�������
    pid_cap_i.pout = pid_cap_i.kp * pid_cap_i.err;
    pid_cap_i.iout = pid_cap_i.ki * pid_cap_i.integral;
    pid_cap_i.dout = pid_cap_i.kd * (pid_cap_i.err - pid_cap_i.err_last);
    
    float temp = pid_cap_i.pout + pid_cap_i.iout + pid_cap_i.dout;
    
    //�������������
	//* �������Ϊ��ֵ
    if(temp > pid_cap_i.OutDutyCycleMax)pid_cap_i.OutDutyCycle = pid_cap_i.OutDutyCycleMax;
    else if(temp < pid_cap_i.OutDutyCycleMin)pid_cap_i.OutDutyCycle = pid_cap_i.OutDutyCycleMin;
    else pid_cap_i.OutDutyCycle = temp;
    
    //һ���ͺ��˲���������
//    if(pid_cap_i.Filter_fac > 0.5f)
//    {
//        pid_cap_i.OutDutyCycle = pid_cap_i.LastOutDutyCycle * pid_cap_i.Filter_fac \
//                                   + pid_cap_i.OutDutyCycle * (1.0f - pid_cap_i.Filter_fac);
//        pid_cap_i.LastOutDutyCycle = pid_cap_i.OutDutyCycle;
//    }
    
    //��¼����ƫ��
    pid_cap_i.err_last = pid_cap_i.err;
    
	//* ��ĳ�������ͨ��
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����
    count ++;   //������pid���д������������Ӧ��1������10000
}

//* 5.���ݳ�ŵ�pid
//* Ŀ��:��ع���Ŀ��ֵ(�������廷pid���)  
//* ��ǰ:��ǰ��ع���
//* ���:����Ŀ�����(ȡ���������Ϊ��)
void cap_pid_bat_power()
{
	//* ����״̬��
    static uint8_t flag = 0;
	//* ������
    static uint16_t timetick = 0;
    //�ж��Ƿ����и�pid
	//* �ж������ڴ�����ص�����
    if(pid_bat_power.PidSwitch == 0)
    {
		//* ���ֺ��������
        pid_bat_power.integral = 0;
        pid_bat_power.OutCurrent = 0;
		//* ״̬������
        flag = 0;
        timetick = 0;
        return;
    }
    else if(pid_bat_power.PidSwitch == 1 && flag <= 1)
    {
        //������
		//* ������<500
        if(timetick < 500)
        {
			//* ���ݵ���С��2.5
            if(abs(adc_data.cap_i) < 2.5f)
            {
				//* ����������
                timetick ++;
            }
			//* ���ֺ��������
            pid_bat_power.OutCurrent = 0;
            pid_bat_power.integral = 0;
            return;
        }
		//* ������500~1000
        else if(timetick < 1000)
        {
			//* ״̬����1(����״̬)
            flag = 1;
            timetick ++;
        }
        else
        {
			//* ״̬����2
            flag = 2;
			//* ��������0
            timetick = 0;
        }
    }
    
	//* ��ֵĿ��ֵΪ���̹�������pid�����
    pid_bat_power.SetPower = pid_powerbuffer.OutPower;
	//* �����ص�ǰ������Ϊ��ǰֵ
    pid_bat_power.ActualPower = adc_data.bat_v * adc_data.bat_i;
    
    //����ƫ��
	//* ע�������Ϊ����ĵ���(ȡ���������Ϊ��)
    pid_bat_power.err = pid_bat_power.SetPower - pid_bat_power.ActualPower;

    //��������
//    if(abs(pid_bat_power.err) < pid_bat_power.deadarea)
//    {
//        return;
//    }
    
    //�����ۼ�
    pid_bat_power.integral += pid_bat_power.err;
    
    //���ƻ���������
	//* ��������Ϊ��ֵ
    if(pid_bat_power.integral > pid_bat_power.integralmax)
    {
        pid_bat_power.integral = pid_bat_power.integralmax;
    }
    else if(pid_bat_power.integral < pid_bat_power.integralmin)
    {
        pid_bat_power.integral = pid_bat_power.integralmin;
    }
    
    //�������
    pid_bat_power.pout = pid_bat_power.kp * pid_bat_power.err;
    pid_bat_power.iout = pid_bat_power.ki * pid_bat_power.integral;
    pid_bat_power.dout = pid_bat_power.kd * (pid_bat_power.err - pid_bat_power.err_last);
    
    float temp = pid_bat_power.pout + pid_bat_power.iout + pid_bat_power.dout;
	//* ���������Ϊ���õĶ�ֵ�ͽ��յĳ�ŵ�����ȡ��С���
    float temp_max = min(pid_bat_power.OutCurrentMax, receive_data.input_power_limit / adc_data.cap_v);
    float temp_min = max(pid_bat_power.OutCurrentMin, -receive_data.output_power_limit / adc_data.cap_v);
    //�������������
    if(temp > temp_max)pid_bat_power.OutCurrent = temp_max;
    else if(temp < temp_min)pid_bat_power.OutCurrent = temp_min;
    else pid_bat_power.OutCurrent = temp;
    
    //������
	//* timetick(500~1000)
	//* ((float)(timetick - 500) / 500) -> (0,1)
	//* ���𶯽�� = (pid������-���ݵ�ǰ����)*(0~1)ʱ��ϵ��+���ݵ�ǰ����
	//* ��timetick(500~1000)��ʱ��ƽ�����
    if(flag == 1)
    {
        pid_bat_power.OutCurrent = (pid_bat_power.OutCurrent - adc_data.cap_i) * ((float)(timetick - 500) / 500) + adc_data.cap_i;
    }
    
    //���ƹ������
	//* ����ϵ��cap_vol_min_limitΪ��ֵ
	//* ����
    if(adc_data.cap_v < cap_vol_min_limit + 2.0f && pid_bat_power.OutCurrent < 0)
    {
        pid_bat_power.OutCurrent *= max(0.5f*(adc_data.cap_v - cap_vol_min_limit), 0);
    }
	//* ����
    else if(adc_data.cap_v > cap_vol_max_limit - 2.0f && pid_bat_power.OutCurrent > 0)
    {
        pid_bat_power.OutCurrent *= max(0.5f*(cap_vol_max_limit - adc_data.cap_v), 0);
    }
    
    
    //һ���ͺ��˲���������
//    if(pid_bat_power.Filter_fac > 0.5f)
//    {
//        pid_bat_power.OutCurrent = pid_bat_power.LastOutCurrent * pid_bat_power.Filter_fac \
//                                   + pid_bat_power.OutCurrent * (1.0f - pid_bat_power.Filter_fac);
//        pid_bat_power.LastOutCurrent = pid_bat_power.OutCurrent;
//    }
    
    //��¼����ƫ��
    pid_bat_power.err_last = pid_bat_power.err;
}

//* 2.���̹�������pid
//* Ŀ��:50J��������  
//* ��ǰ:��ǰ��������
//* ���:��ع���
void cap_pid_powerbuffer()
{
    //ע������������pid�����̹��ʻ���Խ�����ϵͳ������ʾ�ҪԽ�󣬼���ƫ��ķ�ʽ�븺�����෴��
    //�ж��Ƿ����и�pid
	//* �ж�������error�ص�������
    if(pid_powerbuffer.PidSwitch == 0){
        pid_powerbuffer.integral = 0;
        return;
    }
    
    //���ݲ���ϵͳ��Ϣ���ڹ������ƣ�ȡֵ��Χ�ڹ������ơ�5W����ֹ���ʲ���̫��
    //��Ϊ����ϵͳ��������0.1s��ˢ��һ�Σ��ı书�ʶ����������Ӱ�������ͺ��ԣ��������pid�ڵ��̹��ʿ��ٱ仯ʱЧ�����á�
    //����ȡֵ��ΧԽС��pid����ԽС�����Թ��ʲ�����׼ȷ��Ҫ����
	//* ��ֵ�����˵��̹�����������
    pid_powerbuffer.OutPowerMax = receive_data.chassis_power_limit + 5;
    pid_powerbuffer.OutPowerMin = receive_data.chassis_power_limit - 5;
	//* ��ֵpid��������
    pid_powerbuffer.integralmax = pid_powerbuffer.OutPowerMax / pid_powerbuffer.ki;
    pid_powerbuffer.integralmin = pid_powerbuffer.OutPowerMin / pid_powerbuffer.ki;
	//* ��ǰ���̹��ʻ���
    pid_powerbuffer.ActualBuffer = receive_data.chassis_power_buffer;
    
    //����ƫ��
	//* ��ǰ���������Ŀ����������(50��ֵ)
    pid_powerbuffer.err = pid_powerbuffer.ActualBuffer - pid_powerbuffer.SetBuffer;
    
    if(pid_powerbuffer.err > 1)
    {
        pid_powerbuffer.err = pid_powerbuffer.err * pid_powerbuffer.err;
    }
    else if(pid_powerbuffer.err < -1)
    {
        pid_powerbuffer.err = -pid_powerbuffer.err * pid_powerbuffer.err;
    }

    //��������
    if(abs(pid_powerbuffer.err) < pid_powerbuffer.deadarea)
    {
        return;
    }
    
    //�����ۼ�
    pid_powerbuffer.integral += pid_powerbuffer.err;
    
    //���ƻ���������
    if(pid_powerbuffer.integral > pid_powerbuffer.integralmax)
    {
        pid_powerbuffer.integral = pid_powerbuffer.integralmax;
    }
    else if(pid_powerbuffer.integral < pid_powerbuffer.integralmin)
    {
        pid_powerbuffer.integral = pid_powerbuffer.integralmin;
    }
    
    //�������
    pid_powerbuffer.pout = pid_powerbuffer.kp * pid_powerbuffer.err;
    pid_powerbuffer.iout = pid_powerbuffer.ki * pid_powerbuffer.integral;
    pid_powerbuffer.dout = pid_powerbuffer.kd * (pid_powerbuffer.err - pid_powerbuffer.err_last);
    
    float temp = pid_powerbuffer.pout + pid_powerbuffer.iout + pid_powerbuffer.dout;
    
    //�������������
    if(temp > pid_powerbuffer.OutPowerMax)pid_powerbuffer.OutPower = pid_powerbuffer.OutPowerMax;
    else if(temp < pid_powerbuffer.OutPowerMin)pid_powerbuffer.OutPower = pid_powerbuffer.OutPowerMin;
    else pid_powerbuffer.OutPower = temp;
    
    //�͹��ʻ���ʱ���ٻָ�
    if(pid_powerbuffer.ActualBuffer < 20)
    {
		//* ��ֹ���̹��ʳ���
        pid_powerbuffer.OutPower *= max(0.05f * (pid_powerbuffer.ActualBuffer - 22), 0);
    }
    
    //һ���ͺ��˲���������
//    if(pid_powerbuffer.Filter_fac > 0.5f)
//    {
//        pid_powerbuffer.OutPower = pid_powerbuffer.LastOutPower * pid_powerbuffer.Filter_fac \
//                                   + pid_powerbuffer.OutPower * (1.0f - pid_powerbuffer.Filter_fac);
//        pid_powerbuffer.LastOutPower = pid_powerbuffer.OutPower;
//    }
    
    //��¼����ƫ��
    pid_powerbuffer.err_last = pid_powerbuffer.err;
}

//* 8.��pid_cap_i.OutDutyCycle�����ȥ
void pwm_output()
{
    //����ο����ϿƼ��ĳ���
    //����˼·�����ϡ�����STM32F334ͬ������BUCK-BOOST���ֵ�Դ��ơ�P8����ϸ˵��
    uint32_t	buck_duty,boost_duty;
	uint32_t	PWM_PER_0_5	= 0.5f * hhrtim1.Instance->sTimerxRegs[0].PERxR;
    
	if(pid_cap_i.OutDutyCycle > 0.9f)  //����ռ�ձȴ���90%ʱ������BOOSTģʽ��
		boost_duty = (pid_cap_i.OutDutyCycle - 0.8f) * PWM_PER_0_5;//����boost��ռ�ձȣ�����ģ�����ĶԳƵ�PWM��ע��Ҫ��ȥbuck_duty��ռ�ձ�: 0.8*PWM_PER_0_5
	else
		boost_duty = 0.1f * PWM_PER_0_5;	//����ռ�ձȲ�����90%ʱ������BUCKģʽ,boost_duty���̶�ռ�ձ�

	if(pid_cap_i.OutDutyCycle > 0.9f)
		buck_duty = 0.9f * PWM_PER_0_5; //����ռ�ձȴ���90%ʱ������BOOSTģʽ,buck_duty���̶�ռ�ձ�
	else
		buck_duty = pid_cap_i.OutDutyCycle * PWM_PER_0_5;	//����buck��ռ�ձȣ�����ģ�����ĶԳƵ�PWM��

	hhrtim1.Instance->sTimerxRegs[0].CMP1xR = PWM_PER_0_5 - buck_duty;//TimerA PWM��1�Ƚ�����ģ�����ĶԳƵ�PWM����PWM_PER_0_5Ϊ����
    hhrtim1.Instance->sTimerxRegs[0].CMP2xR = PWM_PER_0_5 + buck_duty;//TimerA PWM��0�Ƚ���������  ��400-50���루400+50�������ĵ�Ϊ400	

	hhrtim1.Instance->sTimerxRegs[3].CMP1xR = PWM_PER_0_5 + boost_duty;//TimerD PWM��1�Ƚ�����ģ�����ĶԳƵ�PWM����PWM_PER_0_5Ϊ����
    hhrtim1.Instance->sTimerxRegs[3].CMP2xR = PWM_PER_0_5 - boost_duty;//TimerD PWM��0�Ƚ���������  ��400-50���루400+50�������ĵ�Ϊ400	
    
}

void system_init()
{
    //opamp
    HAL_OPAMP_Start(&hopamp2);
    //hrtim
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
//    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //�����Ӷ�ʱ��
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

//* 9.������
void error_check()
{
    static uint32_t timetick[5] = {0};
    outtime[0] = (adc_data.cap_v > 25) ? ++outtime[0] : 0;  //���ݹ�ѹ��ʱ
    outtime[1] = (abs(adc_data.cap_i) > 14) ? ++outtime[1] : 0;  //���ݹ�����ʱ
    outtime[2] = (adc_data.cap_v < 3) ? ++outtime[2] : 0;  //����Ƿѹ��ʱ
    outtime[3] = (adc_data.bat_v < 16) ? ++outtime[3] : 0;  //����ϵͳǷѹ��ʱ
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
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ���ر�
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

//* 6.���ư���������Ч�ʼ���
void data_cal()
{
    //�����Ч��ֻ�ǿ��ư�����������Ч�ʣ������������������ĵ�Ч�ʡ�
    static uint16_t timetick = 0;
    static float powerin = 0;   //���չ���
    static float powerout = 0;  //��������
    
    //����0�������ʣ�С��0���չ���
	//* ������
    bat_power = adc_data.bat_v * adc_data.bat_i;
	//* �������/����
    cap_power = -adc_data.cap_v * adc_data.cap_i;
	//* ��������
    chas_power = -adc_data.chas_v * adc_data.chas_i;
    
    bat_power > 0 ? (powerout += bat_power) : (powerin += bat_power);
    cap_power > 0 ? (powerout += cap_power) : (powerin += cap_power);
    chas_power > 0 ? (powerout += chas_power) : (powerin += chas_power);
    
	//* 100�����ļ���һ��(10ms)
    timetick ++;
    if(timetick > 100)
    {
        timetick = 0;
		//* ���ư���������Ч�ʼ���
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
	//������
    error_check();
	//����
//    led_blue_set();
//    led_red_set();
//    oled_display();
}

//* 1.�������ؽ��յĲ���ϵͳ��Ϣ-> ���̹��ʻ��壬�����˵��̹����������ޣ����ݷŵ繦�����ƣ����ݳ�繦������
void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x02E)
    {
		//* ���̹��ʻ���
        receive_data.chassis_power_buffer = ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
        if(receive_data.chassis_power_buffer > 60)receive_data.chassis_power_buffer = 0;
    }
    else if(canId == 0x02F)
    {
		//* �����˵��̹�����������
        receive_data.chassis_power_limit = ((uint16_t)rxBuf[0] << 8| rxBuf[1]);
		//* ���ݷŵ繦������
        receive_data.output_power_limit = ((uint16_t)rxBuf[2] << 8| rxBuf[3]);
		//* ���ݳ�繦������
        receive_data.input_power_limit = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);
		//* ���ݿ���
        receive_data.cap_control.all = ((uint16_t)rxBuf[6] << 8| rxBuf[7]);
		//* �޷�
        if(receive_data.chassis_power_limit > 120)receive_data.chassis_power_limit = 120;
        if(receive_data.output_power_limit > 300 || receive_data.output_power_limit < -120)
        {
            receive_data.output_power_limit = 300;
        }
        if(receive_data.input_power_limit > 150)receive_data.input_power_limit = 150;
    }
    outtime[4] = 0;
}

//* 4.CAN����
void can_send()
{
	//* �ش����ݵ�ѹ
    can_send_data[0] = float_to_int16(adc_data.cap_v, 30, 0, 32000, -32000);
	//* �ش����ݵ���
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




