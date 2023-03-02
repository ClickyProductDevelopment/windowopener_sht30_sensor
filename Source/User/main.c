/**
  ******************************************************************************
  * @file    main.c
  * @author  Alexander
  * @version V1.0
  * @date    2022-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:HK32F030M开发板 
  * 论坛    :https://bbs.21ic.com/iclist-1010-1.html
  *
  ******************************************************************************
  */ 
#include "hk32f030m.h" 
#include "bsp_usart.h"
#include "bsp_led.h"
#include "sht3x.h"

 
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */ 
int main(void)
{
	etError   error;       // error code
  uint32_t      serialNumber;// serial number
  regStatus status;      // sensor status
  ft        temperature; // temperature [?C]
  ft        humidity;    // relative humidity [%RH]
	
  /* 初始化LED */
//  LED_GPIO_Config();
  
  /* 初始化USART 配置模式为 115200 8-N-1 */
  USART_Config();
  
  printf("\r\n SHT30 Test\r\n");
  
  //SHT30 Init
	SHT3X_Init(0x44);

  // wait 50ms after power on
  DelayMicroSeconds(50000);    
  
  error = SHT3x_ReadSerialNumber(&serialNumber);
  if(error != NO_ERROR){} // do error handling here
	
	printf("SHT SN=%x\r\n", serialNumber);
  
  while(1)
	{	
		error = NO_ERROR;
    
    // loop while no error
    while(error == NO_ERROR)
    {
      // read status register
      error |= SHT3X_ReadStatus(&status.u16);
      if(error != NO_ERROR) break;
      
      // check if the reset bit is set after a reset or power-up
      if(status.bit.ResetDetected)
      {
        //override default temperature and humidity alert limits (red LED)
        error = SHT3X_SetAlertLimits( 70.0f,  50.0f,  // high set:   RH [%], T [?C]
                                      68.0f,  48.0f,  // high clear: RH [%], T [?C]
                                      32.0f,  -2.0f,  // low clear:  RH [%], T [?C]
                                      30.0f,  -4.0f); // low set:    RH [%], T [?C]
		    if(error != NO_ERROR) break;
		
        
        // clear reset and alert flags
        error = SHT3X_ClearAllAlertFlags();
        if(error != NO_ERROR) break;
        
        //start periodic measurement, with high repeatability and 1 measurements per second
        error = SHT3X_StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_1HZ);
        if(error != NO_ERROR) break;
      }
			
			// demonstrate a single shot measurement with polling and 50ms timeout
			error = SHT3X_GetTempAndHumi(&temperature, &humidity, REPEATAB_HIGH, MODE_POLLING, 50);
      
      // read measurment buffer
      error = SHT3X_ReadMeasurementBuffer(&temperature, &humidity);
      if(error == NO_ERROR)
      {
				// new temperature and humidity values
      }
			else if (error == ACK_ERROR)
      {
        // there were no new values in the buffer -> ignore this error
        error = NO_ERROR;
      }
			else
			{
				break;
			}
			
			printf("Temp: %f\r\n", temperature);
			printf("Humi: %f\r\n", humidity);
      
      DelayMicroSeconds(1000000);
    }
    
    // ... try first a soft reset ...
    error = SHT3X_SoftReset();
    
    // ... if the soft reset fails, do a hard reset
    if(error != NO_ERROR)
    {
      SHT3X_HardReset();
    }
    
    DelayMicroSeconds(10000);
	}	
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char* file , uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */	
       /* Infinite loop */
	
	while (1)
  {		
  }
}
#endif /* USE_FULL_ASSERT */


