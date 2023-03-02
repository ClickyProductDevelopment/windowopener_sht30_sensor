#ifndef __I2C_EE_H
#define __I2C_EE_H


#include "hk32f030m.h"


/**************************I2C参数定义********************************/
#define             sEE_I2C                                 I2C1
#define             sEE_I2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             sEE_I2C_CLK                             RCC_APB1Periph_I2C1
#define             sEE_I2C_GPIO_AHBxClock_FUN              RCC_AHBPeriphClockCmd

#define             sEE_I2C_GPIO_CLK                        RCC_AHBPeriph_GPIOC     
#define             sEE_I2C_SCL_PORT                        GPIOC   
#define             sEE_I2C_SCL_PIN                         GPIO_Pin_6
#define             sEE_I2C_SCL_SOURCE                      GPIO_PinSource6
#define             sEE_I2C_SCL_AF                          GPIO_AF_0 
#define             sEE_I2C_SDA_PORT                        GPIOC 
#define             sEE_I2C_SDA_PIN                         GPIO_Pin_5
#define             sEE_I2C_SDA_SOURCE                      GPIO_PinSource5
#define             sEE_I2C_SDA_AF                          GPIO_AF_0

/* 这个地址只要与HK32外挂的I2C器件地址不一样即可 */
#define I2C_OWN_ADDRESS7      0X0A   

/* AT24C01/02每页有8个字节 */
#define sEE_PAGESIZE           8

/* AT24C04/08A/16A每页有16个字节 */
//#define sEE_PAGESIZE           16	


/*等待超时时间*/
#define sEE_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define sEE_LONG_TIMEOUT         ((uint32_t)(10 * sEE_FLAG_TIMEOUT))

/* sEE_WaitEepromStandbyState() function 最大测试数值 */
#define sEE_MAX_TRIALS_NUMBER     300
      
#define sEE_OK                    0
#define sEE_FAIL                  1  


/*信息输出*/
#define EEPROM_DEBUG_ON         0

#define EEPROM_INFO(fmt,arg...)           printf("<<-EEPROM-INFO->> "fmt"\n",##arg)
#define EEPROM_ERROR(fmt,arg...)          printf("<<-EEPROM-ERROR->> "fmt"\n",##arg)
#define EEPROM_DEBUG(fmt,arg...)          do{\
                                          if(EEPROM_DEBUG_ON)\
                                          printf("<<-EEPROM-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)


/* 
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */

/* EEPROM Addresses defines */
#define EEPROM_Block0_ADDRESS 0xA0   /* E2 = 0 */
//#define EEPROM_Block1_ADDRESS 0xA2 /* E2 = 0 */
//#define EEPROM_Block2_ADDRESS 0xA4 /* E2 = 0 */
//#define EEPROM_Block3_ADDRESS 0xA6 /* E2 = 0 */


void     sEE_Init(void);
uint32_t sEE_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t* NumByteToRead);
uint32_t sEE_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint8_t* NumByteToWrite);
void     sEE_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
uint32_t sEE_WaitEepromStandbyState(void);

#endif



























