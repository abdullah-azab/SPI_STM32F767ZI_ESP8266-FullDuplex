#include "main.h"

#define MASTER_BOARD


__IO uint8_t ubButtonPress = 0;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "hi";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
__IO uint8_t ubTransmitIndex = 0;

/* Buffer used for reception */
uint8_t aRxBuffer[sizeof(aTxBuffer)];
uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
__IO uint8_t ubReceiveIndex = 0;


/* function prototypes */
void     SystemClock_Config(void);
void     Configure_SPI(void);
void     Activate_SPI(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Blinking(uint32_t Period);
void     LED_Off(void);
void     UserButton_Init(void);
void     WaitForUserButtonPress(void);
void     WaitAndCheckEndOfTransfer(void);
uint8_t  Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
static void CPU_CACHE_Enable(void);

int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Initialize LED1 */
  LED_Init();

  /* Configure the SPI1 parameters */
  Configure_SPI();

  /* Initialize User push-button in EXTI mode */
  UserButton_Init();

  /* Wait for User push-button press to start transfer */
  WaitForUserButtonPress();

  /* Enable the SPI1 peripheral */
  Activate_SPI();

  /* Wait for the end of the transfer and check received data */
  /* LED blinking FAST during waiting time */
  WaitAndCheckEndOfTransfer();

  /* Infinite loop */
  while (1);
}



void Configure_SPI(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure SCK Pin connected to pin 10 of CN7 connector */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

  /* Configure MISO Pin connected to pin 12 of CN7 connector */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to pin 14 of CN7 connector */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);

  /* (2) Configure NVIC for SPI1 transfer complete/error interrupts **********/
  /* Set priority for SPI1_IRQn */
  NVIC_SetPriority(SPI1_IRQn, 0);
  /* Enable SPI1_IRQn           */
  NVIC_EnableIRQ(SPI1_IRQn);

  /* (3) Configure SPI1 functional parameters ********************************/

  /* Enable the peripheral clock of GPIOA */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  /* Configure SPI1 communication */
  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV64);
  LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);
  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
  /* Reset value is LL_SPI_MSB_FIRST */
  LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);

  /* Configure SPI1 transfer interrupts */
  /* Enable RXNE  Interrupt             */
  LL_SPI_EnableIT_RXNE(SPI1);
  /* Enable Error Interrupt             */
  LL_SPI_EnableIT_ERR(SPI1);
}


void Activate_SPI(void)
{
  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
	LL_SPI_DisableIT_TXE(SPI1);
}


void LED_Init(void)
{
  /* Enable the LED1 Clock */
  LED1_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED1 */
  LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
	
}


void LED_On(void)
{
  /* Turn LED1 on */
  LL_GPIO_SetOutputPin(LED1_GPIO_PORT, LED1_PIN);
}



void LED_Off(void)
{
  /* Turn LED1 off */
  LL_GPIO_ResetOutputPin(LED1_GPIO_PORT, LED1_PIN);
}

void LED_Blinking(uint32_t Period)
{
  /* Toggle LED1 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
    LL_mDelay(Period);
  }
}

void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();

  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
	
	
	LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  //LL_GPIO_SetPinOutputType(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
	LL_GPIO_ResetOutputPin(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6);
  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();

  /* Enable a rising trigger External line 13 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 0x03);
}

void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
    LL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
    LL_mDelay(LED_BLINK_FAST);
  }
  /* Ensure that LED1 is turned Off */
	LL_GPIO_SetOutputPin(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6);
	LL_mDelay(LED_BLINK_FAST);
  LED_Off();
}


void WaitAndCheckEndOfTransfer(void)
{
	LL_GPIO_ResetOutputPin(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6);
	LL_SPI_TransmitData8(SPI1, 0x02);
	while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
	LL_SPI_TransmitData8(SPI1, 0x00);
	while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
		LL_SPI_EnableIT_TXE(SPI1);
	
  /* 1 - Wait end of transmission */
  while (ubTransmitIndex != ubNbDataToTransmit)
  {
  }
	LL_SPI_DisableIT_TXE(SPI1);
	
	for(uint16_t i=0;i< 32 -( ubNbDataToTransmit );i++){
		LL_SPI_TransmitData8(SPI1, 0x00);
		while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
	}
	while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
	LL_GPIO_SetOutputPin(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6);
	 LL_mDelay(1000);
	LL_GPIO_ResetOutputPin(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6);
		LL_SPI_TransmitData8(SPI1, 0x03);
		while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
		LL_SPI_TransmitData8(SPI1, 0x00);
		while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
	for(uint16_t i=0;i< 32;i++){
		LL_SPI_TransmitData8(SPI1, 0x00);
		while((__IO uint32_t) (READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY)) ? 1UL : 0UL);
	}
	
  /* 2 - Wait end of reception */
  while (ubNbDataToReceive > ubReceiveIndex)
  {
  }
  /* Disable RXNE Interrupt */
	/* Disable TXE Interrupt */
  LL_SPI_DisableIT_RXNE(SPI1);
	LL_GPIO_SetOutputPin(USER_BUTTON_GPIO_PORT, LL_GPIO_PIN_6);
  /* 3 - Compare Transmit data to receive data */
  if(Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, ubNbDataToTransmit))
  {
    /* Processing Error */
    LED_Blinking(LED_BLINK_ERROR);
  }
  else
  {
    /* Turn On Led if data are well received */
    LED_On();
  }
}
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }
    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}
void SystemClock_Config(void)
{
  /* Enable HSE clock */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);

  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Activation OverDrive Mode */
  LL_PWR_EnableOverDriveMode();
  while(LL_PWR_IsActiveFlag_OD() != 1)
  {
  };

  /* Activation OverDrive Switching */
  LL_PWR_EnableOverDriveSwitching();
  while(LL_PWR_IsActiveFlag_ODSW() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 96, LL_RCC_PLLP_DIV_6);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  /* Set systick to 1ms */
  SysTick_Config(32000000 / 1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 32000000;
}


void UserButton_Callback(void)
{
  /* Update User push-button variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

void  SPI1_Rx_Callback(void)
{
  /* Read character in Data register.
  RXNE flag is cleared by reading data in DR register */
	uint8_t tmp =LL_SPI_ReceiveData8(SPI1);
	aRxBuffer[ubReceiveIndex++] = tmp;
  if(ubReceiveIndex == ubNbDataToTransmit+1){
		
	if(Buffercmp8((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, ubNbDataToTransmit))
  {
    ubReceiveIndex=0;
  }
  else
  {
    /* Turn On Led if data are well received */
    LED_On();
		LL_SPI_DisableIT_RXNE(SPI1);
  }
	}
}

void  SPI1_Tx_Callback(void)
{
  /* Write character in Data register.
  TXE flag is cleared by reading data in DR register */
  LL_SPI_TransmitData8(SPI1, aTxBuffer[ubTransmitIndex++]);
}

void SPI1_TransferError_Callback(void)
{
  /* Disable RXNE  Interrupt             */
  LL_SPI_DisableIT_RXNE(SPI1);

  /* Disable TXE   Interrupt             */
  LL_SPI_DisableIT_TXE(SPI1);

  /* Set LED1 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}


static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
