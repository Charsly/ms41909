/***************************************************************************************************
* FILE: ms41909.c
*
* DESCRIPTION:  MS41909�����������
*
***************************************************************************************************/
#include "ms41909.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "arm_math.h"


/* �汾��Ϣ */
#define VER_MAJOR   1
#define VER_MINOR1  0
#define VER_MINOR2  11
#define DATE_YEAR   2020
#define DATE_MONTH  09
#define DATE_DAY    08

#define STR(s)     #s
#define VERSION(a, b, c)    STR(a) "." STR(b) "." STR(c)
#define DATE(y, m, d)       STR(y) "." STR(m) "." STR(d)

//static const char version[] = VERSION(VER_MAJOR, VER_MINOR1, VER_MINOR2);
static const char version_full[] = VERSION(VER_MAJOR, VER_MINOR1, VER_MINOR2) " " DATE(DATE_YEAR, DATE_MONTH, DATE_DAY);

#define DEF_OSC_FREQ			(27 * 1000000u)

#define DEFAULT_VD_FREQUENCY    50
#define DEFAULT_STOP_POWER      60
#define DEFAULT_RUN_POWER       100

/* VD��Ч���� */
#define MS41909_VD_RAISING      0
#define MS41909_VD_FALLING      1

/* �����ת���� */
#define MS41909_DIR_POS         0
#define MS41909_DIR_NEG         1

/* ���ʹ�� */
#define MS41909_DISABLE         0
#define MS41909_ENABLE          1

/* ���ɲ����һ�㲻ʹ�� */
#define MS41909_BRAKE_DISABLE   0
#define MS41909_BRAKE_ENABLE    1

/* ϸ��ģʽ */
#define MS41909_DIV_64          3
#define MS41909_DIV_128         2
#define MS41909_DIV_256         0


/*********************** �Ĵ������� ***********************/

/* ��������ʱɲ���ٶ� */
#define MS41909_INV_BRAKE           10

/* VD FZ����*/
#define MS41909_VD_FZ_POLARITY      MS41909_VD_RAISING

/* DELAY1����λ��303.4us */
#define MS41909_DELAY1              15

/* PWMƵ��=(OSCIN / ((mode * 2^3) * 2^res)) 
Ĭ��mode=8��res=1����ʱpwmƵ��Ϊ210.9k */
/* PWMģʽ */
#define MS41909_PWM_MODE            8

/* PWM�ֱ��� */
#define MS41909_PWM_RES             2


/* MOTOR1 DELAY����λ��303.4us */
#define MS41909_MOTOR1_DELAY        2

/* MOTOR1 ��λ���� */
#define MS41909_MOTOR1_PHASE        0

/* MOTOR1 ��ֵ���� */
#define MS41909_MOTOR1_MAX_PULSE    100

/* MOTOR1 ϸ���� */
#define MS41909_MOTOR1_DIV          MS41909_DIV_256

/* MOTOR2 DELAY����λ��303.4us */
#define MS41909_MOTOR2_DELAY        2

/* MOTOR2 ��λ���� */
#define MS41909_MOTOR2_PHASE        0

/* MOTOR2 ��ֵ���� */
#define MS41909_MOTOR2_MAX_PULSE    100

/* MOTOR2 ϸ���� */
#define MS41909_MOTOR2_DIV          MS41909_DIV_256


/* ����������� */
#define MOTOR_IS_RUNNING(m)     (!((m)->cur_pos == (m)->tar_pos))
/* ������з��� */
#define GET_RUN_DIR(cur, tar)   (((tar)-(cur)) > 0 ? 1:0)

#define GET_POS_DIFF(a, b)      (m_abs(b-a))


/* ������� */
struct ms41909_motor
{
    /* ģʽ���� */
    uint8_t dir_reverse;
    uint8_t stop_power;
    uint8_t run_power;
    uint8_t disable_when_stop;
    
    /* �Ĵ������� */
    uint8_t enable;         //���ʹ��
    uint8_t brake;          //ɲ��
    uint8_t max_power;      //��ֵ���壬0~100
    uint8_t phase;          //��λ����
    
    uint8_t dir;            //��ת����
    uint8_t steps;          //һ��VD�������еĲ���
    uint16_t step_period;   //΢������
    
    int32_t cur_pos;        //��ǰλ��
    int32_t tar_pos;        //Ŀ��λ��
    //    int32_t sta_pos;        //��ʼλ��
    int32_t acc_pos;        //���ٽ���λ��
    int32_t dec_pos;        //��ʼ����λ��
    
    int32_t cur_speed;       //��ǰ�ٶ�
    //    int32_t sta_speed;       //��ʼ�ٶ�
    int32_t tar_speed;       //Ŀ���ٶ�
    int32_t dec_off;        //����ƫ��
    
    int32_t acc;             //���ٶ�1
    int32_t dec;             //���ٶ�2
    
};

/* MS41909�������� */
struct ms41909
{
    uint8_t vd_frequency;
    uint16_t period;
    struct ms41909_motor motor1;    //���1����
    struct ms41909_motor motor2;    //���2����
};

static struct ms41909 ms41909_status;
static struct ms41909_ops ms41909_ops_s;

int32_t MS41909_SetPwmPulse(struct ms41909_motor *pmotor, uint8_t per);

/***************************************************************************************************
* Description:  �������ֵ
***************************************************************************************************/
static inline int32_t m_abs(int32_t n)
{
    //    int32_t val;
    //    arm_abs_q31(&n, &val, 1);
    //    return val;
    return abs(n);
}

/***************************************************************************************************
* Description:  ����ƽ�����������ܳ���46340
***************************************************************************************************/
//static inline int32_t m_power(int16_t n)
//{
//    int64_t val;
//    arm_power_q15(&n, 1, &val);
//    return (int32_t)val;
//}

/***************************************************************************************************
* Description:  ����ƽ����
***************************************************************************************************/
//static inline int32_t m_sqrt(int32_t n)
//{
//    float val=0;
//    //        return (int32_t)sqrt((float)n);
//    arm_sqrt_f32((float)n, &val);
//    return (int32_t)val;
//}

/***************************************************************************************************
* Description:  �����v0��v1�����λ��
***************************************************************************************************/
static inline int32_t m_acc_sum(int32_t v0, int32_t v1, int32_t acc)
{
    int32_t n, sum;
    
    if(v0 > v1) 
    {
        //        n = (v0 - v1) / acc;
        //        sum = ((v1 * n) + (n * (n-1)) * acc / 2);
        //        if((v1 + n * acc) < v0)
        //        {
        //            sum += v0;
        //        }
        
        if((v0-v1) < acc) return (v0-v1);
        
        n = (v0-v1) / acc;
        sum = (((v0-acc) * n) - (n * (n-1)) * acc / 2);
    }
    else if(v0 < v1)
    {
        n = (v1 - v0) / acc;
        sum = (((v0+acc) * n) + (n * (n-1)) * acc / 2);
        if((v0 + n * acc) < v1)
        {
            sum += v1;
        }
    }
    else return 0;
    return sum;
}

/***************************************************************************************************
* Description: ������ٵ�0����λ��
***************************************************************************************************/
static inline int32_t m_dec_sum(int32_t v, int32_t dec)
{
    int32_t n, sum;
    
    if(v <= dec)
        return v;
    
    n = v / dec;
    sum = (((v-dec) * n) - (n * (n-1)) * dec / 2);
    return sum;
}

/***************************************************************************************************
* Description:  ���ݾ��������ٶ�
***************************************************************************************************/
//static inline int32_t m_sum2dec(int32_t v0, int32_t v1, int32_t sum)
//{
//    return ((m_power(v1) - m_power(v0)) / (2 * sum + v1 - v0));    
//}

/***************************************************************************************************
* Description:  ���õ����ֵ������
***************************************************************************************************/
int32_t MS41909_SetPwmPulse(struct ms41909_motor *pmotor, uint8_t per)
{
    uint16_t temp;
    uint8_t pulse;
    
    if(per > 100) per = 100;
    
    /* ��ֵ���� = x / (PWM_MODE * 8) */
    pulse = (MS41909_PWM_MODE << 3) * per / 100;
    
    temp = (pulse << 8) | pulse;
    
    /* ���2 */
    if(pmotor == &ms41909_status.motor2)
    {
        if(ms41909_ops_s.write_reg(0x28, temp) != 0) return 1;
    }
    /* ���1 */
    else
    {
        if(ms41909_ops_s.write_reg(0x23, temp) != 0) return 1;
    }
    return 0;
}

/***************************************************************************************************
* Description:  д�����в���
***************************************************************************************************/
static inline void MS41909_WriteRunData(uint8_t motor)
{
    uint16_t temp;
    
    /* ���2 */
    if(motor)
    {
        /* ϸ�֣�LED��ʹ�ܣ�ɲ�������򣬲��� */
        temp = (MS41909_MOTOR2_DIV << 12) | (ms41909_status.motor2.enable << 10) | \
            ((ms41909_status.motor2.dir ^ ms41909_status.motor2.dir_reverse) << 8) | ms41909_status.motor2.steps;
        ms41909_ops_s.write_reg(0x29, temp);
        
        /* ΢������ */
        ms41909_ops_s.write_reg(0x2A, ms41909_status.motor2.step_period);
    }
    /* ���1 */
    else
    {
        /* ϸ�֣�LED��ʹ�ܣ�ɲ�������򣬲��� */
        temp = (MS41909_MOTOR1_DIV << 12) | (ms41909_status.motor1.enable << 10) | \
            ((ms41909_status.motor1.dir ^ ms41909_status.motor1.dir_reverse) << 8) | ms41909_status.motor1.steps;
        ms41909_ops_s.write_reg(0x24, temp);
        
        /* ΢������ */
        ms41909_ops_s.write_reg(0x25, ms41909_status.motor1.step_period);
    }
}

/***************************************************************************************************
* Description:  �������в���
***************************************************************************************************/
static inline void MS41909_CalcuMotorPara(struct ms41909_motor *pmotor, int32_t speed)
{
    pmotor->steps = speed;
    pmotor->step_period = ms41909_status.period / speed;
}

static inline int32_t MS41909_CalcuRunPara(struct ms41909_motor *pmotor)
{
    if(MOTOR_IS_RUNNING(pmotor))
    {
        /********** ��ת **********/
        if(pmotor->dir)
        {
            /********** ���� **********/
            if(pmotor->cur_pos < pmotor->acc_pos)
            {
                pmotor->cur_speed += pmotor->acc;
                if(pmotor->acc > 0)
                {
                    if(pmotor->cur_speed > pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                else 
                {
                    if(pmotor->cur_speed < pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                
                pmotor->cur_pos += pmotor->cur_speed;
                
                MS41909_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
            
            /********** ���� **********/
            else if(pmotor->cur_pos < pmotor->dec_pos)
            {
                pmotor->cur_pos += pmotor->cur_speed;
                return 0;
            }
            
            /********** ���� **********/
            else if(pmotor->cur_pos < pmotor->tar_pos)
            {
                /* ��ֵ */
                if(pmotor->cur_speed <= pmotor->dec_off)
                {
                    if((pmotor->cur_pos + pmotor->dec_off) > pmotor->tar_pos)
                    {
                        pmotor->dec_off = pmotor->tar_pos - pmotor->cur_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos += pmotor->dec_off;
                    }
                    MS41909_CalcuMotorPara(pmotor, pmotor->dec_off);
                    pmotor->dec_off = -1;
                }
                else
                {
                    pmotor->cur_speed -= pmotor->dec;
                    
                    if(pmotor->cur_speed == 0)
                    {
                        if(pmotor->tar_pos == (pmotor->cur_pos + pmotor->dec))
                        {
                            pmotor->cur_pos = pmotor->tar_pos;
                            MS41909_CalcuMotorPara(pmotor, pmotor->dec);
                            return 0;
                        }
                    }
                    
                    if(pmotor->cur_speed < 0) pmotor->cur_speed = 1;
                    
                    if((pmotor->cur_pos + pmotor->cur_speed) > pmotor->tar_pos)
                    {
                        pmotor->cur_speed = pmotor->tar_pos - pmotor->cur_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos += pmotor->cur_speed;
                    }
                    MS41909_CalcuMotorPara(pmotor, pmotor->cur_speed);
                }
            }
            else
            {
                pmotor->cur_speed = 0;
                MS41909_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
        }
        
        
        /********** ��ת **********/
        else
        {
            /********** ���� **********/
            if(pmotor->cur_pos > pmotor->acc_pos)
            {
                pmotor->cur_speed += pmotor->acc;
                if(pmotor->acc > 0)
                {
                    if(pmotor->cur_speed > pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                else 
                {
                    if(pmotor->cur_speed < pmotor->tar_speed) pmotor->cur_speed = pmotor->tar_speed;
                }
                
                pmotor->cur_pos -= pmotor->cur_speed;
                
                MS41909_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
            
            /********** ���� **********/
            else if(pmotor->cur_pos > pmotor->dec_pos)
            {
                pmotor->cur_pos -= pmotor->cur_speed;
                return 0;
            }
            
            /********** ���� **********/
            else if(pmotor->cur_pos > pmotor->tar_pos)
            {
                /* ��ֵ */
                if(pmotor->cur_speed <= pmotor->dec_off)
                {
                    if((pmotor->cur_pos - pmotor->dec_off) < pmotor->tar_pos)
                    {
                        pmotor->dec_off = pmotor->cur_pos - pmotor->tar_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos -= pmotor->dec_off;
                    }
                    MS41909_CalcuMotorPara(pmotor, pmotor->dec_off);
                    pmotor->dec_off = -1;
                }
                else
                {
                    pmotor->cur_speed -= pmotor->dec;
                    
                    if(pmotor->cur_speed == 0)
                    {
                        if(pmotor->tar_pos == (pmotor->cur_pos - pmotor->dec))
                        {
                            pmotor->cur_pos = pmotor->tar_pos;
                            MS41909_CalcuMotorPara(pmotor, pmotor->dec);
                            return 0;
                        }
                    }
                    
                    if(pmotor->cur_speed < 0) pmotor->cur_speed = 1;
                    
                    if((pmotor->cur_pos - pmotor->cur_speed) < pmotor->tar_pos)
                    {
                        pmotor->cur_speed = pmotor->cur_pos - pmotor->tar_pos;
                        pmotor->cur_pos = pmotor->tar_pos;
                    }
                    else
                    {
                        pmotor->cur_pos -= pmotor->cur_speed;
                    }
                    
                    MS41909_CalcuMotorPara(pmotor, pmotor->cur_speed);
                }
            }
            else
            {
                pmotor->cur_speed = 0;
                MS41909_CalcuMotorPara(pmotor, pmotor->cur_speed);
            }
        }
        return 1;
    }
    
    
    /********** ͣ�� **********/
    else
    {
        if(pmotor->brake == 0)
        {
            pmotor->brake = 1;
            pmotor->cur_speed = 0;
            MS41909_CalcuMotorPara(pmotor, 0);
            
            if(pmotor->disable_when_stop)
                pmotor->enable = 0;
            pmotor->max_power = pmotor->stop_power;
            MS41909_SetPwmPulse(pmotor, pmotor->max_power);
            
            return 1;
        }
        else if(pmotor->brake == 1)
        {
            pmotor->brake = 2;
            
            if((!MOTOR_IS_RUNNING(&ms41909_status.motor1)) && (!MOTOR_IS_RUNNING(&ms41909_status.motor2)))
            {
                ms41909_ops_s.stop_vd();
            }
        }
    }
    
    return 0;
}

/***************************************************************************************************
* Description:  VD FZ���崦��
***************************************************************************************************/
void MS41909_VdFzPulseHandler(void)
{
    if(MS41909_CalcuRunPara(&ms41909_status.motor1))
    {
        MS41909_WriteRunData(MS41909_MOTOR1);
    }
    
    if(MS41909_CalcuRunPara(&ms41909_status.motor2))
    {
        MS41909_WriteRunData(MS41909_MOTOR2);
    }
}

/***************************************************************************************************
* Description:  ��ȡ�汾��
***************************************************************************************************/
const char* MS41909_GetVersion(void)
{
    return version_full;
}

/***************************************************************************************************
* Description:  ���õ����ת����
***************************************************************************************************/
int32_t MS41909_SetDirInv(int32_t motor, uint8_t inv)
{
    if(motor)
    {
        if(inv) ms41909_status.motor2.dir_reverse = 1;
        else ms41909_status.motor2.dir_reverse = 0;
    }
    else
    {
        if(inv) ms41909_status.motor1.dir_reverse = 1;
        else ms41909_status.motor1.dir_reverse = 0;
    }
    return 0;
}

/***************************************************************************************************
* Description:  �������к�ֹͣ��ѹ��
***************************************************************************************************/
int32_t MS41909_SetStopPower(int32_t motor, uint8_t per)
{
    if(per > 100) per = 100;
    
    if(motor) ms41909_status.motor2.stop_power = per;
    else ms41909_status.motor1.stop_power = per;
    
    return 0;
}
int32_t MS41909_SetRunPower(int32_t motor, uint8_t per)
{
    if(per > 100) per = 100;
    
    if(motor) ms41909_status.motor2.run_power = per;
    else ms41909_status.motor1.run_power = per;
    
    return 0;
}

/***************************************************************************************************
* Description:  ֹͣʱ�ضϵ��
***************************************************************************************************/
int32_t MS41909_SetStopCutOff(int32_t motor, uint8_t onoff)
{
    if(onoff) onoff = 1;
    
    if(motor) ms41909_status.motor2.disable_when_stop = onoff;
    else ms41909_status.motor1.disable_when_stop = onoff;
    
    return 0;
}

/***************************************************************************************************
* Description:  MS41909ȫ�ֳ�ʼ��
***************************************************************************************************/
int32_t  MS41909_Init(struct ms41909_ops *ops)
{
    uint16_t temp;
    
    if(ops == NULL)
    {
        return 1;
    }
    /* ע������ӿ� */
    memcpy(&ms41909_ops_s, ops, sizeof(struct ms41909_ops));
    
    /* ��ʼ��״̬ */
    memset(&ms41909_status, 0, sizeof(struct ms41909));
    
    
    MS41909_SetFrequency(DEFAULT_VD_FREQUENCY, DEF_OSC_FREQ);
    
    ms41909_status.motor1.stop_power = DEFAULT_STOP_POWER;
    ms41909_status.motor1.run_power = DEFAULT_RUN_POWER;
    ms41909_status.motor2.stop_power = DEFAULT_STOP_POWER;
    ms41909_status.motor2.run_power = DEFAULT_RUN_POWER;
    
    //    ms41909_status.motor1.enable = 1;
    //    ms41909_status.motor2.enable = 1;
    
    /************************ ȫ�ֲ��� ************************/
    /* VD FZ���� */
    temp = (MS41909_VD_FZ_POLARITY << 8);
    if(ms41909_ops_s.write_reg(0x0B, temp) != 0) goto error;
    
    /* pwm�ֱ��ʣ�pwmģʽ��delay1 */
    temp = (MS41909_PWM_RES << 13) | (MS41909_PWM_MODE << 8) | MS41909_DELAY1;
    if(ms41909_ops_s.write_reg(0x20, temp) != 0) goto error;
    
    
    /************************ MOTOR1 ************************/
    /* ��λ�motor1_delay2 */
    temp = (MS41909_MOTOR1_PHASE << 8) | MS41909_MOTOR1_DELAY;
    if(ms41909_ops_s.write_reg(0x22, temp) != 0) goto error;
    
    /* ����������ɲ����ʹ�ܣ�LED��ϸ�� */
    if(ms41909_ops_s.write_reg(0x24, 0) != 0) goto error;
    
    /* ΢������ */
    if(ms41909_ops_s.write_reg(0x25, 0) != 0) goto error;
    
    
    /************************ MOTOR2 ************************/
    /* ��λ�motor1_delay2 */
    temp = (MS41909_MOTOR2_PHASE << 8) | MS41909_MOTOR2_DELAY;
    if(ms41909_ops_s.write_reg(0x27, temp) != 0) goto error;
    
    /* ����������ɲ����ʹ�ܣ�LED��ϸ�� */
    if(ms41909_ops_s.write_reg(0x29, 0) != 0) goto error;
    
    /* ΢������ */
    if(ms41909_ops_s.write_reg(0x2A, 0) != 0) goto error;
    
    return 0;
    
    error:
    return 1;
}

/***************************************************************************************************
* Description:  ���ɲ��
***************************************************************************************************/
int32_t MS41909_MotorBrake(int32_t motor, int8_t dec)
{
    struct ms41909_motor *pmotor;
    int32_t dec_dist;
    
    if(motor) pmotor = &ms41909_status.motor2;
    else pmotor = &ms41909_status.motor1;
    
    if(!MOTOR_IS_RUNNING(pmotor))
        return 0;
    
    ms41909_ops_s.stop_vd();
    
    dec_dist = m_dec_sum(pmotor->cur_speed, dec);
    
    pmotor->dec = dec;
    pmotor->tar_speed = 0;
    pmotor->acc_pos = pmotor->cur_pos;
    pmotor->dec_pos = pmotor->cur_pos;
    pmotor->dec_off = -1;
    
    if(pmotor->dir)
    {
        pmotor->tar_pos = pmotor->cur_pos + dec_dist;
    }
    else
    {
        pmotor->tar_pos = pmotor->cur_pos - dec_dist;
    }
    pmotor->brake = 0;
    
    ms41909_ops_s.start_vd();
    
    return dec_dist;
}

/***************************************************************************************************
* Description:  �������
***************************************************************************************************/
int32_t MS41909_MotorGoto(int32_t motor, int32_t tar_pos, uint8_t speed, int8_t acc, int8_t dec)
{
    uint8_t flag = 0;
    struct ms41909_motor *pmotor;
    static int32_t acc_dist, dec_dist, pos_off;
    
    if(speed == 0 || acc == 0 || dec == 0)
        return 1;
    
    if(motor) pmotor = &ms41909_status.motor2;
    else pmotor = &ms41909_status.motor1;
    
    if(tar_pos == pmotor->cur_pos)
        return 1;
    
    if((tar_pos == pmotor->tar_pos) && (speed == pmotor->tar_speed))
        return 1;
    
    ms41909_ops_s.stop_vd();
    
    /* �������� */
    if(MOTOR_IS_RUNNING(pmotor))
    {
        flag = 1;
        /* ���� */
        if(pmotor->dir != GET_RUN_DIR(pmotor->cur_pos, tar_pos))
        {
            MS41909_MotorBrake(motor, MS41909_INV_BRAKE);
            while(MOTOR_IS_RUNNING(pmotor))
            {
                
            }
            MS41909_MotorGoto(motor, tar_pos, speed, acc, dec);
        }
        /* ͬ�� */
        else
        {
            pos_off = GET_POS_DIFF(tar_pos, pmotor->cur_pos);
            acc_dist = m_acc_sum(pmotor->cur_speed, speed, acc);
            dec_dist = m_dec_sum(speed, dec);
            
            /* ���벻�� */
            if(pos_off < (acc_dist + dec_dist))
            {
                /* ���벻����ֱ�Ӽ��� */
                if(pos_off < m_dec_sum(pmotor->cur_speed, dec))
                {
                    /* ��������ֱ��ɲ����������ʵļ��ٶ������������е�Ŀ��λ�� */
                    if(pos_off > m_dec_sum(pmotor->cur_speed, MS41909_INV_BRAKE))
                    {
                        uint8_t new_dec = dec;
                        do {
                            dec_dist = m_dec_sum(pmotor->cur_speed, new_dec);
                        } while((pos_off < dec_dist) && (++new_dec));
                        
                        if((pos_off-dec_dist) % speed)
                        {
                            pmotor->dec_off = (pos_off-dec_dist) % speed;
                            dec_dist += pmotor->dec_off;
                        }
                        acc_dist = 0;
                        pmotor->acc = 0;
                        pmotor->dec = new_dec;
                        pmotor->tar_speed = 0;
                        pmotor->acc_pos = pmotor->cur_pos;
                        pmotor->dec_pos = pmotor->cur_pos;
                        goto END;
                    }
                    /* ���벻����ֱ��ɲ������ɲͣ�����е�Ŀ��λ�� */
                    else
                    {
                        MS41909_MotorBrake(motor, MS41909_INV_BRAKE);
                        while(MOTOR_IS_RUNNING(pmotor))
                        {
                            
                        }
                        MS41909_MotorGoto(motor, tar_pos, speed, acc, dec);
                    }
                }
                /* ��������ֱ�Ӽ��� */
                else
                {
                    /* ��ǰ�ٶȴ��������ٶȣ��Ӵ���ٶ����������ٶȺͼ��ٶ����� */
                    if(pmotor->cur_speed > speed)
                    {
                        uint8_t new_acc = acc;
						if(pos_off <= dec_dist)
						{
							new_acc = new_acc;
						}
						else
						{
							while(pos_off < (m_acc_sum(pmotor->cur_speed, speed, new_acc) + dec_dist))
							{
								new_acc++;
							}
						}
                        acc = new_acc;
                        goto R;
                    }
                    /* ��ǰ�ٶ�С�ڵ��������ٶȣ����������ٶ��������ļ��ٶȺͼ��ٶ����� */
                    else
                    {
                        goto R;
                    }
                }
            }
            /* �������ԼӼ��� */
            else
            {
                goto R;
            }
        }
    }
    /* ��ֹ״̬ */
    else
    {
        R:
        if(flag == 0)
        {
            pmotor->cur_speed = 0;
        }
        pos_off = GET_POS_DIFF(tar_pos, pmotor->cur_pos);
        acc_dist = m_acc_sum(pmotor->cur_speed, speed, acc);
        dec_dist = m_dec_sum(speed, dec);
        
        /* ���벻�� */
        if(pos_off < (acc_dist + dec_dist))
        {
            uint8_t new_speed = speed;
            do
            {
                if(new_speed > 1)
                    new_speed--;
                else
                    new_speed = 1;
                
                acc_dist = m_acc_sum(pmotor->cur_speed, new_speed, acc);
                dec_dist = m_dec_sum(new_speed, dec);
            } while(((acc_dist + dec_dist) > pos_off) && (new_speed > 1));
            speed = new_speed;
            if(speed < 1)
            {
                speed = 1;
                acc_dist = m_acc_sum(0, new_speed, acc);
                dec_dist = m_dec_sum(new_speed, dec);
            }
        }
        
        /* ����ƫ�� */
        if((pos_off-acc_dist-dec_dist) % speed)
        {
            pmotor->dec_off = (pos_off-acc_dist-dec_dist) % speed;
            dec_dist += pmotor->dec_off;
            if(dec == 0)
                dec = 1;
        }
        
        if(pmotor->cur_speed < speed) pmotor->acc = acc;
        else if(pmotor->cur_speed > speed)pmotor->acc = acc * -1;
        else pmotor->acc = 0;
        pmotor->dec = dec;
        pmotor->tar_speed = speed;
        pmotor->tar_pos = tar_pos;
    }
    
    END:
    if(GET_RUN_DIR(pmotor->cur_pos, pmotor->tar_pos))
    {
        pmotor->acc_pos = pmotor->cur_pos + acc_dist;
        pmotor->dec_pos = tar_pos - dec_dist;
        pmotor->dir = 1;
    }
    else
    {
        pmotor->acc_pos = pmotor->cur_pos - acc_dist;
        pmotor->dec_pos = tar_pos + dec_dist;
        pmotor->dir = 0;
    }
    
    pmotor->max_power = pmotor->run_power;
    MS41909_SetPwmPulse(pmotor, pmotor->max_power);
    pmotor->brake = 0;
    pmotor->enable = 1;
    /* �ָ����� */
    ms41909_ops_s.start_vd();
    
    return 0;
}

/***************************************************************************************************
* Description:  ʹ�ܵ��
***************************************************************************************************/
int32_t MS41909_MotorEnable(int32_t motor, int32_t new_state)
{
    if(motor)
    {
        if(new_state)
        {
            ms41909_status.motor2.enable = 1;
            MS41909_WriteRunData(motor);
        }
        else
        {
            if(MOTOR_IS_RUNNING(&ms41909_status.motor2))
                return 1;
            ms41909_status.motor2.enable = 0;
            MS41909_WriteRunData(motor);
        }
    }
    else
    {
        if(new_state)
        {
            ms41909_status.motor1.enable = 1;
            MS41909_WriteRunData(motor);
        }
        else
        {
            if(MOTOR_IS_RUNNING(&ms41909_status.motor1))
                return 1;
            ms41909_status.motor1.enable = 0;
            MS41909_WriteRunData(motor);
        }
    }
    return 0;
}

/***************************************************************************************************
* Description:  �����״̬
***************************************************************************************************/
int32_t MS41909_MotorIsRunning(int32_t motor)
{
    if(motor)
    {
        return MOTOR_IS_RUNNING(&ms41909_status.motor2);
    }
    else
    {
        return MOTOR_IS_RUNNING(&ms41909_status.motor1);
    }
}

/***************************************************************************************************
* Description:  ��ȡ�����ǰ���ٶ�
***************************************************************************************************/
int32_t MS41909_GetMotorCurSpeed(int32_t motor)
{
    if(motor)
    {
        return ms41909_status.motor2.cur_speed;
    }
    else
    {
        return ms41909_status.motor1.cur_speed;
    }
}

/***************************************************************************************************
* Description:  ��ȡ�����ǰλ��
***************************************************************************************************/
int32_t MS41909_GetMotorPosition(int32_t motor)
{
    if(motor)
    {
        return ms41909_status.motor2.cur_pos;
    }
    else
    {
        return ms41909_status.motor1.cur_pos;
    }
}

/***************************************************************************************************
* Description:  �޸ĵ����ǰλ��
***************************************************************************************************/
int32_t MS41909_SetMotorPosition(int32_t motor, int32_t new_pos)
{
    if(motor)
    {
        ms41909_status.motor2.cur_pos = new_pos;
    }
    else
    {
        ms41909_status.motor1.cur_pos = new_pos;
    }
    return 0;
}

/***************************************************************************************************
* Description:  ������������
***************************************************************************************************/
int32_t MS41909_SetFrequency(uint8_t freq, uint32_t osc)
{
    if(freq == 0)
        return 1;
    ms41909_status.vd_frequency = freq;
    ms41909_status.period = osc / 24 / ms41909_status.vd_frequency;
    return 0;
}








/****************************************** END OF FILE *******************************************/
