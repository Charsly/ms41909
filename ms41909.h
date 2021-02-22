/***************************************************************************************************
* FILE: ms41909.h

* DESCRIPTION: --

***************************************************************************************************/
#ifndef __MS41909_H__
#define __MS41909_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define MS41909_MOTOR1          0
#define MS41909_MOTOR2          1

/* 电机驱动操作接口 */
struct ms41909_ops
{
    int32_t (*write_reg)(uint8_t addr, uint16_t val);
    void (*start_vd)(void);
    void (*stop_vd)(void);
};


/* 初始化 */
const char* MS41909_GetVersion(void);
int32_t MS41909_Init(struct ms41909_ops *ops);

/* 基本控制 */
int32_t MS41909_MotorEnable(int32_t motor, int32_t new_state);
int32_t MS41909_SetStopPower(int32_t motor, uint8_t per);
int32_t MS41909_SetRunPower(int32_t motor, uint8_t per);
int32_t MS41909_SetStopCutOff(int32_t motor, uint8_t onoff);
int32_t MS41909_SetDirInv(int32_t motor, uint8_t inv);
int32_t MS41909_SetFrequency(uint8_t freq, uint32_t osc);

/* 状态 */
int32_t MS41909_MotorIsRunning(int32_t motor);
int32_t MS41909_GetMotorCurSpeed(int32_t motor);
int32_t MS41909_GetMotorPosition(int32_t motor);
int32_t MS41909_SetMotorPosition(int32_t motor, int32_t new_pos);

/* 运行控制 */
int32_t MS41909_MotorBrake(int32_t motor, int8_t dec);
int32_t MS41909_MotorGoto(int32_t motor, int32_t tar_pos, uint8_t speed, int8_t acc, int8_t dec);

/* 脉冲驱动，必须调用 */
void MS41909_VdFzPulseHandler(void);

#ifdef __cplusplus
}
#endif

#endif
/****************************************** END OF FILE *******************************************/
