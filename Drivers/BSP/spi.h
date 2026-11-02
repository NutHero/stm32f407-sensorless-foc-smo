#ifndef __SPI_H
#define __SPI_H

#include "./SYSTEM/sys/sys.h"

/* SPI总线速度设置 */
#define SPI_SPEED_2         0
#define SPI_SPEED_4         1
#define SPI_SPEED_8         2
#define SPI_SPEED_16        3
#define SPI_SPEED_32        4
#define SPI_SPEED_64        5
#define SPI_SPEED_128       6
#define SPI_SPEED_256       7
extern SPI_HandleTypeDef g_spi3_handler; /* SPI3句柄 */

void spi3_init(void);
void spi3_set_speed(uint8_t speed);
uint8_t spi3_read_write_byte(uint8_t txdata);
void spi3_write_byte(uint8_t data);
uint16_t spi3_read_write_data16(uint16_t txdata);
void spi3_write_data16(uint16_t txdata); //写数据16位

#endif
























