
//-- Includes -----------------------------------------------------------------
#include "sht3x.h"
#include "bsp_usart.h"	

//-- Defines ------------------------------------------------------------------
// Generator polynomial for CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

//=============================================================================
// IO-Pins                            /* -- adapt the defines for your uC -- */
//-----------------------------------------------------------------------------
// Reset on port A, bit 2
#define RESET_LOW()  GPIO_ResetBits(SHT_RST_PORT,SHT_RST_PIN) // set Reset to low
#define RESET_HIGH() GPIO_SetBits(SHT_RST_PORT,SHT_RST_PIN) // set Reset to high

// Alert on port A, bit 1
#define ALERT_READ   GPIO_ReadInputDataBit(SHT_ALERT_PORT,SHT_ALERT_PIN)     // read Alert
//=============================================================================

//-- Global variables ---------------------------------------------------------
static uint8_t _i2cAddress; // I2C Address

__IO uint16_t  SHTAddress = 0;   
__IO uint32_t  SHTTimeout = SHT_LONG_TIMEOUT;
__IO uint16_t  SHTDataNum;  

//-- Static function prototypes -----------------------------------------------
static etError SHT3X_WriteAlertLimitData(ft humidity, ft temperature);
static etError SHT3X_ReadAlertLimitData(ft* humidity, ft* temperature);
static etError SHT3X_StartWriteAccess(void);
static etError SHT3X_StartReadAccess(void);
static void SHT3X_StopAccess(void);
static etError SHT3X_WriteCommand(etCommands command);
static etError SHT3X_Read2BytesAndCrc(uint16_t* data, etI2cAck finaleAckNack,
                                      uint8_t timeout);
static etError SHT3X_Write2BytesAndCrc(uint16_t data);
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes);
static etError SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
static ft SHT3X_CalcTemperature(uint16_t rawValue);
static ft SHT3X_CalcHumidity(uint16_t rawValue);
static uint16_t SHT3X_CalcRawTemperature(ft temperature);
static uint16_t SHT3X_CalcRawHumidity(ft humidity);

void Sf_SDA_OUT(void);
void Sf_SDA_IN(void);
uint8_t Sf_SDA_READ(void);

//-----------------------------------------------------------------------------
void SHT3X_Init(uint8_t i2cAddress)          /* -- adapt the init for your uC -- */
{
  GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHBPeriphClockCmd( SHT_I2C_SCL_GPIO_CLK | SHT_I2C_SDA_GPIO_CLK | SHT_RST_GPIO_CLK | SHT_ALERT_GPIO_CLK, ENABLE);
	
	// SHT reset
	GPIO_InitStructure.GPIO_Pin = SHT_RST_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(SHT_RST_PORT, &GPIO_InitStructure);	
	
  RESET_LOW();
	
	// SHT ALERT
	GPIO_InitStructure.GPIO_Pin = SHT_ALERT_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Schmit = GPIO_Schmit_Disable;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SHT_ALERT_PORT, &GPIO_InitStructure);
	
  // Software I2C SDA
  GPIO_InitStructure.GPIO_Pin = SHT_I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SHT_I2C_SDA_PORT, &GPIO_InitStructure);
  
  // Software I2C SCL
  GPIO_InitStructure.GPIO_Pin = SHT_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SHT_I2C_SCL_PORT, &GPIO_InitStructure);
  
  Sf_SDA_OUT();
  Sf_SCL_H;
  Sf_SDA_H; 
	
	SHT3X_SetI2cAdr(i2cAddress);
	
  // release reset
  RESET_HIGH();
}


/**
  * @brief  Set SDA as output
  * @param  None
  * @retval None
  */
void Sf_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	#if 0
		SHT_I2C_SDA_PORT->MODER &= ~(0x3<<(SHT_I2C_SDA_SOURCE*2));
		SHT_I2C_SDA_PORT->MODER |=  (0x1<<(SHT_I2C_SDA_SOURCE*2));
	#else
		GPIO_InitStructure.GPIO_Pin = SHT_I2C_SDA_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(SHT_I2C_SDA_PORT, &GPIO_InitStructure);
	#endif
}

/**
  * @brief  Set SDA as input
  * @param  None
  * @retval None
  */
void Sf_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	#if 0
		SHT_I2C_SDA_PORT->MODER &= ~(0x3<<(SHT_I2C_SDA_SOURCE*2));
	#else
		GPIO_InitStructure.GPIO_Pin = SHT_I2C_SDA_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(SHT_I2C_SDA_PORT, &GPIO_InitStructure);
	#endif
}

/**
  * @brief  Read SDA level
  * @param  None
  * @retval None
  */
uint8_t Sf_SDA_READ(void)
{
  return GPIO_ReadInputDataBit(SHT_I2C_SDA_PORT, SHT_I2C_SDA_PIN);
}

/**
  * @brief  Read SCL level
  * @param  None
  * @retval None
  */
uint8_t Sf_SCL_READ(void)
{
  return GPIO_ReadInputDataBit(SHT_I2C_SCL_PORT, SHT_I2C_SCL_PIN);
}


/**
  * @brief  Delay some time
  * @param  None
  * @retval None
  */
void DelayMicroSeconds(uint32_t t)
{
  t = t*5;
  while(--t);
}

//-----------------------------------------------------------------------------
void I2c_StartCondition(void)
{
  Sf_SDA_H;
	Sf_SDA_OUT();
  DelayMicroSeconds(1);
  Sf_SCL_H;
  DelayMicroSeconds(5);
  Sf_SDA_L;
  DelayMicroSeconds(10);  // hold time start condition (t_HD;STA)
  Sf_SCL_L;
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
void I2c_StopCondition(void)
{
	Sf_SDA_OUT();
  Sf_SCL_L;
  DelayMicroSeconds(5);
  Sf_SDA_L;
  DelayMicroSeconds(5);
  Sf_SCL_H;
  DelayMicroSeconds(10);  // set-up time stop condition (t_SU;STO)
  Sf_SDA_H;
  DelayMicroSeconds(10);
}
//-----------------------------------------------------------------------------
//static etError I2c_WaitWhileClockStreching(uint8_t timeout)
//{
//  etError error = NO_ERROR;
//  
//  while(Sf_SCL_READ() == 0)
//  {
//    if(timeout-- == 0) return TIMEOUT_ERROR;
//    DelayMicroSeconds(1000);
//  }
//  
//  return error;
//}
//-----------------------------------------------------------------------------
etError I2c_WriteByte(uint8_t txByte)
{
  etError error = NO_ERROR;
  uint8_t     mask;
	
	Sf_SDA_OUT();
	Sf_SCL_L;
	
  for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
  {
    if((mask & txByte) == 0) Sf_SDA_L; // masking txByte, write bit to SDA-Line
    else                     Sf_SDA_H;
    DelayMicroSeconds(5);               // data set-up time (t_SU;DAT)
    Sf_SCL_H;                         // generate clock pulse on SCL
    DelayMicroSeconds(5);               // SCL high time (t_HIGH)
    Sf_SCL_L;
    DelayMicroSeconds(3);               // data hold time(t_HD;DAT)
  }
  Sf_SDA_IN();                           // release SDA-line
	
	DelayMicroSeconds(3);
  Sf_SCL_H;                           // clk #9 for ack
  DelayMicroSeconds(3);                 // data set-up time (t_SU;DAT)
  if(Sf_SDA_READ()) error = ACK_ERROR;       // check ack from i2c slave
  Sf_SCL_L;
  DelayMicroSeconds(2);                // wait to see byte package on scope
  return error;                         // return error code
}

//-----------------------------------------------------------------------------
etError I2c_ReadByte(uint8_t *rxByte, etI2cAck ack, uint8_t timeout)
{
  etError error = NO_ERROR;
  uint8_t mask;
  *rxByte = 0x00;
  Sf_SDA_IN();                            // release SDA-line
	Sf_SCL_L;
	DelayMicroSeconds(5);
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  { 
    Sf_SCL_H;                          // start clock on SCL-line
//    DelayMicroSeconds(1);                // clock set-up time (t_SU;CLK)
//    error = I2c_WaitWhileClockStreching(timeout);// wait while clock streching
    DelayMicroSeconds(5);                // SCL high time (t_HIGH)
    if(Sf_SDA_READ()) *rxByte |= mask;        // read bit
    Sf_SCL_L;
    DelayMicroSeconds(5);                // data hold time(t_HD;DAT)
  }
	Sf_SDA_OUT();
	DelayMicroSeconds(1); 
  if(ack == ACK) Sf_SDA_L;              // send acknowledge if necessary
  else           Sf_SDA_H;
  DelayMicroSeconds(5);                  // data set-up time (t_SU;DAT)
  Sf_SCL_H;                            // clk #9 for ack
  DelayMicroSeconds(5);                  // SCL high time (t_HIGH)
  Sf_SCL_L;
//  Sf_SDA_IN();                            // release SDA-line
  DelayMicroSeconds(5);                 // wait to see byte package on scope
  
  return error;                          // return with no error
}

//-----------------------------------------------------------------------------
etError I2c_GeneralCallReset(void)
{
  etError error;
  
  I2c_StartCondition();
                        error = I2c_WriteByte(0x00);
  if(error == NO_ERROR) error = I2c_WriteByte(0x06);
  
  return error;
}



//-----------------------------------------------------------------------------
void SHT3X_SetI2cAdr(uint8_t i2cAddress)
{
  _i2cAddress = i2cAddress;
}

//-----------------------------------------------------------------------------
etError SHT3x_ReadSerialNumber(uint32_t* serialNumber)
{
  etError error; // error code
  uint16_t serialNumWords[2];
  
  error = SHT3X_StartWriteAccess();
  
  // write "read serial number" command
  error |= SHT3X_WriteCommand(CMD_READ_SERIALNBR);
	
	#if 1
		// if no error, start read access
		if(error == NO_ERROR) error = SHT3X_StartReadAccess();
		// if no error, read first serial number word
		if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[0], ACK, 100);
		// if no error, read second serial number word
		if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&serialNumWords[1], NACK, 0);
  #else
		error = SHT3X_StartReadAccess();
		error = SHT3X_Read2BytesAndCrc(&serialNumWords[0], ACK, 100);
		error = SHT3X_Read2BytesAndCrc(&serialNumWords[1], NACK, 0);
	#endif
	
  SHT3X_StopAccess();
  
  // if no error, calc serial number as 32-bit integer
  if(error == NO_ERROR)
  {
    *serialNumber = (serialNumWords[0] << 16) | serialNumWords[1];
  }
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_ReadStatus(uint16_t* status)
{
  etError error; // error code
  
  error = SHT3X_StartWriteAccess();
  #if 1
		// if no error, write "read status" command
		if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_READ_STATUS);
		// if no error, start read access
		if(error == NO_ERROR) error = SHT3X_StartReadAccess(); 
		// if no error, read status
		if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(status, NACK, 0);
	#else
		error = SHT3X_WriteCommand(CMD_READ_STATUS);
		error = SHT3X_StartReadAccess(); 
		error = SHT3X_Read2BytesAndCrc(status, NACK, 0);
	#endif
  SHT3X_StopAccess();
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_ClearAllAlertFlags(void)
{
  etError error; // error code
  
  error = SHT3X_StartWriteAccess();
  
  // if no error, write clear status register command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_CLEAR_STATUS);
  
  SHT3X_StopAccess();
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumi(ft* temperature, ft* humidity,
                             etRepeatability repeatability, etMode mode,
                             uint8_t timeout)
{
  etError error;
                               
  switch(mode)
  {    
    case MODE_CLKSTRETCH: // get temperature with clock stretching mode
      error = SHT3X_GetTempAndHumiClkStretch(temperature, humidity,
                                             repeatability, timeout);
      break;
    case MODE_POLLING:    // get temperature with polling mode
      error = SHT3X_GetTempAndHumiPolling(temperature, humidity,
                                          repeatability, timeout);
      break;
    default:              
      error = PARM_ERROR;
      break;
  }
  
  return error;
}


//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiClkStretch(ft* temperature, ft* humidity,
                                       etRepeatability repeatability,
                                       uint8_t timeout)
{
  etError error;        // error code
  uint16_t    rawValueTemp; // temperature raw value from sensor
  uint16_t    rawValueHumi; // humidity raw value from sensor
  
  error = SHT3X_StartWriteAccess();
  
  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in clock stretching mode
    // use depending on the required repeatability, the corresponding command
    switch(repeatability)
    {
      case REPEATAB_LOW:
        error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_L);
        break;
      case REPEATAB_MEDIUM:
        error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_M);
        break;
      case REPEATAB_HIGH:
        error = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_H);
        break;
      default:
        error = PARM_ERROR;
        break;
    }
  }

  // if no error, start read access
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  // if no error, read temperature raw values
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, timeout);
  // if no error, read humidity raw values
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
  
  SHT3X_StopAccess();
  
  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR)
  {
    *temperature = SHT3X_CalcTemperature(rawValueTemp);
    *humidity = SHT3X_CalcHumidity(rawValueHumi);
  }
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_GetTempAndHumiPolling(ft* temperature, ft* humidity,
                                    etRepeatability repeatability,
                                    uint8_t timeout)
{
  etError error;           // error code
  uint16_t    rawValueTemp;    // temperature raw value from sensor
  uint16_t    rawValueHumi;    // humidity raw value from sensor
  
  error  = SHT3X_StartWriteAccess();
  
  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in polling mode
    // use depending on the required repeatability, the corresponding command
    switch(repeatability)
    {
      case REPEATAB_LOW:
        error = SHT3X_WriteCommand(CMD_MEAS_POLLING_L);
        break;
      case REPEATAB_MEDIUM:
        error = SHT3X_WriteCommand(CMD_MEAS_POLLING_M);
        break;
      case REPEATAB_HIGH:
        error = SHT3X_WriteCommand(CMD_MEAS_POLLING_H);
        break;
      default:
        error = PARM_ERROR;
        break;
    }
  }
  
  // if no error, wait until measurement ready
  if(error == NO_ERROR)
  {
    // poll every 1ms for measurement ready until timeout
    while(timeout--)
    {
      // check if the measurement has finished
      error = SHT3X_StartReadAccess();
  
      // if measurement has finished -> exit loop
      if(error == NO_ERROR) break;
  
      // delay 1ms
      DelayMicroSeconds(1000);
    }
    
    // check for timeout error
    if(timeout == 0) error = TIMEOUT_ERROR;
  }
  
  // if no error, read temperature and humidity raw values
  if(error == NO_ERROR)
  {
    error |= SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
    error |= SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
  }
  
  SHT3X_StopAccess();
  
  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR)
  {
    *temperature = SHT3X_CalcTemperature(rawValueTemp);
    *humidity = SHT3X_CalcHumidity(rawValueHumi);
  }
  
  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_StartPeriodicMeasurment(etRepeatability repeatability,
                                      etFrequency frequency)
{
  etError error;        // error code
  
  error = SHT3X_StartWriteAccess();
  
  // if no error, start periodic measurement 
  if(error == NO_ERROR)
  {
    // use depending on the required repeatability and frequency,
    // the corresponding command
    switch(repeatability)
    {
      case REPEATAB_LOW: // low repeatability
        switch(frequency)
        {
          case FREQUENCY_HZ5:  // low repeatability,  0.5 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_L);
            break;          
          case FREQUENCY_1HZ:  // low repeatability,  1.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_L);
            break;          
          case FREQUENCY_2HZ:  // low repeatability,  2.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_L);
            break;          
          case FREQUENCY_4HZ:  // low repeatability,  4.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_L);
            break;          
          case FREQUENCY_10HZ: // low repeatability, 10.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_L);
            break;          
          default:
            error |= PARM_ERROR;
            break;
        }
        break;
        
      case REPEATAB_MEDIUM: // medium repeatability
        switch(frequency)
        {
          case FREQUENCY_HZ5:  // medium repeatability,  0.5 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_M);
			break;
          case FREQUENCY_1HZ:  // medium repeatability,  1.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_M);
			break;        
          case FREQUENCY_2HZ:  // medium repeatability,  2.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_M);
			break;        
          case FREQUENCY_4HZ:  // medium repeatability,  4.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_M);
			break;      
          case FREQUENCY_10HZ: // medium repeatability, 10.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_M);
			break;
          default:
            error |= PARM_ERROR;
			break;
        }
        break;
        
      case REPEATAB_HIGH: // high repeatability
        switch(frequency)
        {
          case FREQUENCY_HZ5:  // high repeatability,  0.5 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_H);
            break;
          case FREQUENCY_1HZ:  // high repeatability,  1.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_H);
            break;
          case FREQUENCY_2HZ:  // high repeatability,  2.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_H);
            break;
          case FREQUENCY_4HZ:  // high repeatability,  4.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_H);
            break;
          case FREQUENCY_10HZ: // high repeatability, 10.0 Hz
            error |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_H);
            break;
          default:
            error |= PARM_ERROR;
            break;
        }
        break;
      default:
        error |= PARM_ERROR;
        break;
    }
  }

  SHT3X_StopAccess();

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_ReadMeasurementBuffer(ft* temperature, ft* humidity)
{
  etError  error;        // error code
  uint16_t     rawValueTemp; // temperature raw value from sensor
  uint16_t     rawValueHumi; // humidity raw value from sensor

  error = SHT3X_StartWriteAccess();

  // if no error, read measurements
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_FETCH_DATA);
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();  
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
  if(error == NO_ERROR) error = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);

  // if no error, calculate temperature in °C and humidity in %RH
  if(error == NO_ERROR)
  {
    *temperature = SHT3X_CalcTemperature(rawValueTemp);
    *humidity = SHT3X_CalcHumidity(rawValueHumi);
  }

  SHT3X_StopAccess();

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_EnableHeater(void)
{
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write heater enable command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_ENABLE);

  SHT3X_StopAccess();

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_DisableHeater(void)
{
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // if no error, write heater disable command
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_HEATER_DISABLE);

  SHT3X_StopAccess();

  return error;
}


//-----------------------------------------------------------------------------
etError SHT3X_SetAlertLimits(ft humidityHighSet,   ft temperatureHighSet,
                             ft humidityHighClear, ft temperatureHighClear,
                             ft humidityLowClear,  ft temperatureLowClear,
                             ft humidityLowSet,    ft temperatureLowSet)
{
  etError  error;  // error code
  
  // write humidity & temperature alter limits, high set
  error = SHT3X_StartWriteAccess();
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_HS);
  if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityHighSet,
                                                          temperatureHighSet);
  SHT3X_StopAccess();

  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, high clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_HC);
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityHighClear,
                                                            temperatureHighClear);
    SHT3X_StopAccess();
  }

  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, low clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_LC);
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityLowClear,
                                                            temperatureLowClear);
    SHT3X_StopAccess();
  }
  
  if(error == NO_ERROR)
  {
    // write humidity & temperature alter limits, low set
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_W_AL_LIM_LS);
    if(error == NO_ERROR) error = SHT3X_WriteAlertLimitData(humidityLowSet,
                                                            temperatureLowSet);
    SHT3X_StopAccess();
  }

  return error;
}

//-----------------------------------------------------------------------------
etError SHT3X_GetAlertLimits(ft* humidityHighSet,   ft* temperatureHighSet,
                             ft* humidityHighClear, ft* temperatureHighClear,
                             ft* humidityLowClear,  ft* temperatureLowClear,
                             ft* humidityLowSet,    ft* temperatureLowSet)
{
  etError  error;  // error code
  
  // read humidity & temperature alter limits, high set
  error = SHT3X_StartWriteAccess();
  if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_HS);
  if(error == NO_ERROR) error = SHT3X_StartReadAccess();
  if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityHighSet,
                                                         temperatureHighSet);
  SHT3X_StopAccess();

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, high clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_HC);
    if(error == NO_ERROR) error = SHT3X_StartReadAccess();
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityHighClear,
                                                           temperatureHighClear);
    SHT3X_StopAccess();
  }

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, low clear
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_LC);
    if(error == NO_ERROR) error = SHT3X_StartReadAccess();
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityLowClear,
                                                           temperatureLowClear);
    SHT3X_StopAccess();
  }

  if(error == NO_ERROR)
  {
    // read humidity & temperature alter limits, low set
    error = SHT3X_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3X_WriteCommand(CMD_R_AL_LIM_LS);
    if(error == NO_ERROR) error = SHT3X_StartReadAccess();
    if(error == NO_ERROR) error = SHT3X_ReadAlertLimitData(humidityLowSet,
                                                           temperatureLowSet);
    SHT3X_StopAccess();
  }
 
  return error;
}
                             
//-----------------------------------------------------------------------------
bt SHT3X_ReadAlert(void)
{
  // read alert pin
  return (ALERT_READ != 0) ? TRUE : FALSE;
}

//-----------------------------------------------------------------------------
etError SHT3X_SoftReset(void)
{
  etError error; // error code

  error = SHT3X_StartWriteAccess();

  // write reset command
  error |= SHT3X_WriteCommand(CMD_SOFT_RESET);

  SHT3X_StopAccess();
  
  // if no error, wait 50 ms after reset
  if(error == NO_ERROR) DelayMicroSeconds(50000); 

  return error;
}

//-----------------------------------------------------------------------------
void SHT3X_HardReset(void)
{
  // set reset low
  RESET_LOW();

  // wait 100 ms
  DelayMicroSeconds(100000);
  
  // release reset
  RESET_HIGH();
  
  // wait 50 ms after reset
  DelayMicroSeconds(50000);
}

                             
//-----------------------------------------------------------------------------
static etError SHT3X_WriteAlertLimitData(ft humidity, ft temperature)
{
  etError  error;           // error code
  
  int16_t rawHumidity;
  int16_t rawTemperature;
  
  if((humidity < 0.0f) || (humidity > 100.0f) 
  || (temperature < -45.0f) || (temperature > 130.0f))
  {
    error = PARM_ERROR;
  }
  else
  {
    rawHumidity    = SHT3X_CalcRawHumidity(humidity);
    rawTemperature = SHT3X_CalcRawTemperature(temperature);

    error = SHT3X_Write2BytesAndCrc((rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF));
  }
  
  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_ReadAlertLimitData(ft* humidity, ft* temperature)
{
  etError  error;           // error code
  uint16_t     data;
  
  error = SHT3X_Read2BytesAndCrc(&data, NACK, 0);
  
  if(error == NO_ERROR)
  {
    *humidity = SHT3X_CalcHumidity(data & 0xFE00);
    *temperature = SHT3X_CalcTemperature(data << 7);
  }
  
  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_StartWriteAccess(void)
{
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the write flag
  error = I2c_WriteByte(_i2cAddress << 1);

  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_StartReadAccess(void)
{
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the read flag
  error = I2c_WriteByte(_i2cAddress << 1 | 0x01);

  return error;
}

//-----------------------------------------------------------------------------
static void SHT3X_StopAccess(void)
{
  // write a stop condition
  I2c_StopCondition();
}

//-----------------------------------------------------------------------------
static etError SHT3X_WriteCommand(etCommands command)
{
  etError error; // error code

  // write the upper 8 bits of the command to the sensor
  error  = I2c_WriteByte(command >> 8);

  // write the lower 8 bits of the command to the sensor
  error |= I2c_WriteByte(command & 0xFF);

  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_Read2BytesAndCrc(uint16_t* data, etI2cAck finaleAckNack,
                                      uint8_t timeout)
{
  etError error;    // error code
  uint8_t     bytes[2]; // read data array
  uint8_t     checksum; // checksum byte
 
  // read two data bytes and one checksum byte
                        error = I2c_ReadByte(&bytes[0], ACK, timeout);
  if(error == NO_ERROR) error = I2c_ReadByte(&bytes[1], ACK, 0);
  if(error == NO_ERROR) error = I2c_ReadByte(&checksum, finaleAckNack, 0);
  
  // verify checksum
  if(error == NO_ERROR) error = SHT3X_CheckCrc(bytes, 2, checksum);
  
  // combine the two bytes to a 16-bit value
  *data = (bytes[0] << 8) | bytes[1];
  
  return error;
}

//-----------------------------------------------------------------------------
static etError SHT3X_Write2BytesAndCrc(uint16_t data)
{
  etError error;    // error code
  uint8_t     bytes[2]; // read data array
  uint8_t     checksum; // checksum byte
  
  bytes[0] = data >> 8;
  bytes[1] = data & 0xFF;
  checksum = SHT3X_CalcCrc(bytes, 2);
 
  // write two data bytes and one checksum byte
                        error = I2c_WriteByte(bytes[0]); // write data MSB
  if(error == NO_ERROR) error = I2c_WriteByte(bytes[1]); // write data LSB
  if(error == NO_ERROR) error = I2c_WriteByte(checksum); // write checksum
  
  return error;
}

//-----------------------------------------------------------------------------
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes)
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  }
  
  return crc;
}

//-----------------------------------------------------------------------------
static etError SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  uint8_t crc;     // calculated checksum
  
  // calculates 8-Bit checksum
  crc = SHT3X_CalcCrc(data, nbrOfBytes);
  
  // verify checksum
  if(crc != checksum) return CHECKSUM_ERROR;
  else                return NO_ERROR;
}

//-----------------------------------------------------------------------------
static ft SHT3X_CalcTemperature(uint16_t rawValue)
{
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (ft)rawValue / 65535.0f - 45.0f;
}

//-----------------------------------------------------------------------------
static ft SHT3X_CalcHumidity(uint16_t rawValue)
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (ft)rawValue / 65535.0f;
}

//-----------------------------------------------------------------------------
static uint16_t SHT3X_CalcRawTemperature(ft temperature)
{
  // calculate raw temperature [ticks]
  // rawT = (temperature + 45) / 175 * (2^16-1)
  return (temperature + 45.0f) / 175.0f * 65535.0f;
}

//-----------------------------------------------------------------------------
static uint16_t SHT3X_CalcRawHumidity(ft humidity)
{
  // calculate raw relative humidity [ticks]
  // rawRH = humidity / 100 * (2^16-1)
  return humidity / 100.0f * 65535.0f;
}
