/*
 * modbus_slave.h
 *
 *  Created on: 03/05/2017
 *      Author: andru
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

/* -----------------------Slave Defines -------------------------------------*/
#define MB_MODE						MB_RTU			// rtu mode only
#define MB_PORT						1				// 1 port only

#define S_DISCRETE_INPUT_START      0
#define S_DISCRETE_INPUT_NDISCRETES 6
#define S_COIL_START                0
#define S_COIL_NCOILS               6
#define S_REG_INPUT_START           0
#define S_REG_INPUT_NREGS           6
#define S_REG_HOLDING_START         0
#define S_REG_HOLDING_NREGS         6

#include "FreeModbus/port/port.h"

//Slave mode:DiscreteInputs variables
extern UCHAR    ucSDiscInBuf[];
//Slave mode:Coils variables
extern UCHAR    ucSCoilBuf[];
//Slave mode:InputRegister variables
extern SHORT   usSRegInBuf[];
//Slave mode:HoldingRegister variables
extern SHORT   usSRegHoldBuf[];

extern UCHAR ucCoilBytes;
extern UCHAR ucChangeCoilBuf[];

/* ----------------------- Modbus registers -------------------------------------*/
/* ---  Type                        Adr Num    - Description
#define S_DISCRETE                      ( 2)   - флаги ошибок и состояния
#define S_DISCRETE_INPUT_START      0   (+1)   - признак ошибки блока
                                    1   (+1)   - NU
                                    2   (+1)   - state - признак вкл/выкл ex_sw1
                                    3   (+1)   - NU
                                    4   (+1)   - state - признак вкл/выкл ex_sw2
                                    5   (+1)   - NU
#define S_COIL                          ( 2)   - признаки получения данных и записи в регистры
#define S_COIL_START                0   (+1)   - признак получения данных блока
                                    1   (+1)   - признак записи настроек modbus
                                    2   (+1)   - признак вкл/выкл + изменения состояния диммера 0
                                    3   (+1)   - признак записи настройки мощности диммера 0 по умолчания в %
                                    4   (+1)   - признак вкл/выкл + изменения состояния диммера 1
                                    5   (+1)   - признак записи настройки мощности диммера 1 по умолчания в %
#define S_REG_INPUT                     ( 2)   - значения и коды ошибок
#define S_REG_INPUT_START           0   (+2)   - код ошибки
                                    1   (+2)   - регистр счетчик блока
                                    2   (+2)   - значение мощности диммера 0 в %
                                    3   (+2)   - NU
                                    4   (+2)   - значение мощности диммера 1 в %
                                    5   (+2)   - NU
#define S_REG_HOLDING                   ( 2)   - установки блока
#define S_REG_HOLDING_START         0   (+2)   - при старте Firmware / при записи д.б. MAGIC
                                    1   (+2)   - значения настроек Modbus config.(addr, bitrate, parity)
                                    2   (+2)   - установка текущей мощности диммера  0в %
                                    3   (+2)   - config.percent установка мощности диммера 0 в %
                                    4   (+2)   - установка текущей мощности диммера 1 в %
                                    5   (+2)   - config.percent установка мощности диммера 1 в %
 -- */

typedef enum {
	ADDR_STATUS,
	ADDR_CONFIG,
	ADDR_DIMMER0_STATE,
	ADDR_DIMMER0_CONFIG,
	ADDR_DIMMER1_STATE,
	ADDR_DIMMER1_CONFIG,
} mb_addr_t;

typedef enum {
	MB_BR_2400,
	MB_BR_4800,
	MB_BR_9600,
	MB_BR_19200,
	MB_BR_38400,
} mb_bitrate_t;

typedef enum {
    MB_PARITY_NONE, /*!< No parity. */
    MB_PARITY_ODD,  /*!< Odd parity. */
    MB_PARITY_EVEN  /*!< Even parity. */
} mb_parity_t;

BOOL initModbus (UCHAR addr, mb_bitrate_t bitrate, mb_parity_t parity);

void setDiscreteBit(USHORT regAddr, UCHAR ucValue);
uint8_t getCoilBit(USHORT regAddr);
void setCoilBit(USHORT regAddr, UCHAR ucValue);
void writeInput(USHORT regAddr, SHORT sValue);
SHORT readHolding(USHORT regAddr);
void writeHolding(USHORT regAddr, SHORT sValue);

#endif /* MODBUS_SLAVE_H_ */
