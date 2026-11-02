#include "decodeVofa+.h"
#include "./SYSTEM/usart/uart.h"
#include <string.h>
#include <math.h>

/**
  **************************************************************************
  * @file     decodeVofa+.c
  * @brief    支持上位机PID调参（同步PID参数到上位机），并支持JustFloat格式发送数据到VOFA+
  * @note     使用串口6
  * @note     指令格式：S M:x-参数名:x E（帧头S，帧尾E）
  **************************************************************************
  */

/**
 * @brief  通过串口发送一个字节（阻塞）
 * @param  huart 串口句柄
 * @param  dat 待发送字节
 */
static void vofa_send_byte(UART_HandleTypeDef *huart, uint8_t dat)
{
    while((huart->Instance->SR & USART_SR_TXE) == 0);
    huart->Instance->DR = dat;
}

/**
 * @brief  通过串口发送一段数据（阻塞）
 * @param  huart 串口句柄
 * @param  buf 数据首地址
 * @param  len 数据长度
 */
static void vofa_send_bytes(UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for(i = 0; i < len; i++)
    {
        vofa_send_byte(huart, buf[i]);
    }
}

/**
 * @brief  发送JustFloat帧尾（0x00 0x00 0x80 0x7F）
 * @param  huart 串口句柄
 */
static void vofa_send_tail(UART_HandleTypeDef *huart)
{
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    vofa_send_bytes(huart, tail, 4);
}

/**
 * @brief  上传PID参数到上位机
 * @param  p PID结构体指针
 */
void upload_pid_parameter(PID_Q15_TypeDef *p)
{
    (void)p;
}

/**
 * @brief  解码串口下发的VOFA+指令
 */
void decode_vofa_command(void)
{
    char temp_buffer[UART6_BUFFER_SIZE] = {0};
    command_data_t cmd;

    if(g_uart6_rx_flag == 1) //串口接收完成
    {
        __disable_irq();
        memcpy(temp_buffer, g_uart6_rxbuffer, UART6_BUFFER_SIZE); //拷贝到临时缓冲区
        temp_buffer[UART6_BUFFER_SIZE - 1] = '\0';
        g_uart6_rx_flag = 0;
        __enable_irq();

        if(parse_vofa_command(temp_buffer, &cmd) == 1)
        {
            __disable_irq();
            execute_command(&cmd); //执行解析出的命令
            __enable_irq();
        }
    }
}

/**
 * @brief  解析命令字符串
 * @param  buffer 接收缓冲区
 * @param  cmd 解析结果
 * @retval 1:解析成功 0:解析失败
 */
uint8_t parse_vofa_command(char *buffer, command_data_t *cmd)
{
    char *p;
    char cmd_type[20];
    int motor, value;

    p = strstr(buffer, "M:"); //兼容帧头，定位命令起始
    if(p == NULL)
    {
        cmd->type = CMD_UNKNOWN;
        return 0;
    }

    if(sscanf(p, "M:%d-%19[^:]:%d", &motor, cmd_type, &value) != 3)
    {
        cmd->type = CMD_UNKNOWN;
        return 0;
    }

    cmd->motor_index = (uint8_t)motor;
    cmd->value = value;

    if(strcmp(cmd_type, "SPEED_P") == 0)        cmd->type = SPEED_P;
    else if(strcmp(cmd_type, "SPEED_I") == 0)   cmd->type = SPEED_I;
    else if(strcmp(cmd_type, "SPEED_D") == 0)   cmd->type = SPEED_D;
    else if(strcmp(cmd_type, "POS_P") == 0)     cmd->type = POS_P;
    else if(strcmp(cmd_type, "POS_I") == 0)     cmd->type = POS_I;
    else if(strcmp(cmd_type, "POS_D") == 0)     cmd->type = POS_D;
    else if(strcmp(cmd_type, "CURRENT_P") == 0) cmd->type = CURRENT_P;
    else if(strcmp(cmd_type, "CURRENT_I") == 0) cmd->type = CURRENT_I;
    else if(strcmp(cmd_type, "CURRENT_D") == 0) cmd->type = CURRENT_D;
    else
    {
        cmd->type = CMD_UNKNOWN;
        return 0;
    }

    return 1;
}

/**
 * @brief  执行解析出的命令（修改对应PID参数）
 * @param  cmd 命令数据
 */
void execute_command(command_data_t *cmd)
{
    dev_motor *pmotor = NULL;

    if(cmd->motor_index == 1) //选择电机
    {
        pmotor = &motor1;
    }
    else
    {
        return;
    }

    switch(cmd->type)
    {
        case SPEED_P:   pmotor->_speed_pid->Proportion = cmd->value; break;
        case SPEED_I:   pmotor->_speed_pid->Integral   = cmd->value; break;
        case SPEED_D:   pmotor->_speed_pid->Derivative = cmd->value; break;
        case POS_P:     pmotor->_pos_pid->Proportion   = cmd->value; break;
        case POS_I:     pmotor->_pos_pid->Integral     = cmd->value; break;
        case POS_D:     pmotor->_pos_pid->Derivative   = cmd->value; break;
        case CURRENT_P: pmotor->_iq_pid->Proportion    = cmd->value; break;
        case CURRENT_I: pmotor->_iq_pid->Integral      = cmd->value; break;
        case CURRENT_D: pmotor->_iq_pid->Derivative    = cmd->value; break;
        default: break;
    }
}

/**
 * @brief  JustFloat格式发送3个整型数据
 */
void send_data_vofa(int16_t dat1, int16_t dat2, int16_t dat3, UART_HandleTypeDef *huart)
{
    float buf[3];
    buf[0] = (float)dat1;
    buf[1] = (float)dat2;
    buf[2] = (float)dat3;
    vofa_send_bytes(huart, (uint8_t *)buf, 12);
    vofa_send_tail(huart);
}

/**
 * @brief  JustFloat格式发送3个浮点数据
 */
void send_data_vofa_float(float dat1, float dat2, float dat3, UART_HandleTypeDef *huart)
{
    float buf[3];
    buf[0] = dat1;
    buf[1] = dat2;
    buf[2] = dat3;
    vofa_send_bytes(huart, (uint8_t *)buf, 12);
    vofa_send_tail(huart);
}

#define PI 3.1415926f
/**
 * @brief  发送三路相位差120度的正弦波到VOFA+
 * @param  frequency 正弦波频率 (Hz)
 * @param  amplitude 幅值 (0.0 ~ 1.0)
 * @param  sample_rate 采样率 (Hz)
 * @param  point_num 发送点数
 * @param  huart 串口句柄
 */
void send_three_phase_sine_wave(float frequency, float amplitude, uint32_t sample_rate, uint32_t point_num, UART_HandleTypeDef *huart)
{
    float time_step = 1.0f / (float)sample_rate;
    float angular_freq = 2.0f * PI * frequency;
    uint32_t i;

    for(i = 0; i < point_num; i++)
    {
        float phase = angular_freq * (float)i * time_step;
        float buf[3];
        buf[0] = amplitude * sinf(phase);                      //A相
        buf[1] = amplitude * sinf(phase - 2.0f * PI / 3.0f);   //B相滞后120度
        buf[2] = amplitude * sinf(phase + 2.0f * PI / 3.0f);   //C相超前120度
        vofa_send_bytes(huart, (uint8_t *)buf, 12);
        vofa_send_tail(huart);
    }
}
