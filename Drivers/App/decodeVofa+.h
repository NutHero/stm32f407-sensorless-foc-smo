#ifndef __DECODE_VOFA_H
#define __DECODE_VOFA_H

#include "./SYSTEM/sys/sys.h"
#include "bldc_control.h"
#include "pid.h"

/**
 * @brief 高速发送任意数量float数据到VOFA+
 * @note  直接在调用处展开，无函数调用开销
 *        使用示例：
 *        VOFA_SEND(&g_uart6_handle, 3, 1.23f, 2.34f, 3.45f);
 *        VOFA_SEND(&g_uart6_handle, 6, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
 */
#define VOFA_SEND(huart, count, ...) do { \
    float _vofa_buf[] = {__VA_ARGS__}; \
    uint8_t *_vofa_ptr = (uint8_t*)_vofa_buf; \
    uint32_t _vofa_len = (count) * 4; \
    for (uint32_t _i = 0; _i < _vofa_len; _i++) { \
        while(((huart)->Instance->SR & USART_SR_TXE) == 0); \
        (huart)->Instance->DR = _vofa_ptr[_i]; \
    } \
    while(((huart)->Instance->SR & USART_SR_TXE) == 0); \
    (huart)->Instance->DR = 0x00; \
    while(((huart)->Instance->SR & USART_SR_TXE) == 0); \
    (huart)->Instance->DR = 0x00; \
    while(((huart)->Instance->SR & USART_SR_TXE) == 0); \
    (huart)->Instance->DR = 0x80; \
    while(((huart)->Instance->SR & USART_SR_TXE) == 0); \
    (huart)->Instance->DR = 0x7f; \
} while(0)

/* 命令类型 */
typedef enum {
    SPEED_P = 0,        //速度环PID比例系数
    SPEED_I,            //速度环PID积分系数
    SPEED_D,            //速度环PID微分系数
    POS_P,              //位置环PID比例系数
    POS_I,              //位置环PID积分系数
    POS_D,              //位置环PID微分系数
    CURRENT_P,          //电流环PID比例系数
    CURRENT_I,          //电流环PID积分系数
    CURRENT_D,          //电流环PID微分系数
    CMD_UNKNOWN         //未知命令
} vofa_command_t;

/* 命令结构体 */
typedef struct {
    vofa_command_t type;    //命令类型
    uint8_t motor_index;    //电机编号
    int32_t value;          //PID参数值
} command_data_t;

void decode_vofa_command(void);                                 //解码串口下发的VOFA+指令
void upload_pid_parameter(PID_Q15_TypeDef *p);                  //上传PID参数到上位机
uint8_t parse_vofa_command(char *buffer, command_data_t *cmd);  //解析命令字符串
void execute_command(command_data_t *cmd);                      //执行解析出的命令
void send_data_vofa(int16_t dat1, int16_t dat2, int16_t dat3, UART_HandleTypeDef *huart);   //JustFloat格式发送3个数据
void send_data_vofa_float(float dat1, float dat2, float dat3, UART_HandleTypeDef *huart);   //JustFloat格式发送3个浮点数据
void send_three_phase_sine_wave(float frequency, float amplitude, uint32_t sample_rate, uint32_t point_num, UART_HandleTypeDef *huart);

#endif
