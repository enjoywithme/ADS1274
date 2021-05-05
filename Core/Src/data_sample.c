#include "data_sample.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdio.h>

#define 	SPI_BUFFSIZE 	12
#define   SPI_BUFFER_N	    10
#define 	SPI_DR_BASE 	(SPI3_BASE + 0x0C) //SPI3 数据寄存器地址
#define		SPI_DMA_STREAM DMA1_Stream0
#define		AD_SYNC_GPIO GPIOD
#define		AD_SYNC_GPIO_PIN	GPIO_PIN_8

uint8_t	ad_start_flag = 0;//AD启动标志

unsigned char arr1[]={'1','2', '3', '4', '5'};
uint8_t spi_send_data[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t spi_recv_data[SPI_BUFFSIZE * SPI_BUFFER_N] = {0};//接收缓冲

uint8_t buffer_recv_index = 0;//当前接收缓冲指针
uint8_t buffer_send_index = 0;//当前发送缓冲指针
uint8_t buffer_flip = 0;//缓冲指针翻篇
uint8_t	buffer_overflow = 0;

static void NVIC_ADS1274(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

static void MX_SPI3_Init(void);

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD8 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}


/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}







void ADS1274_Init(void)
{
	MX_GPIO_Init();
	MX_SPI3_Init();
	MX_DMA_Init();
	
}

/*	ADS1274数据准备好中断	*/
void EXTI9_5_IRQHandler(void)
{   

}


/* SPI3 DMA RX 完成中断   */
void SPI_DMA_STREAM_IRQHandler(void)
{
//	DMA_Cmd(SPI_DMA_STREAM,DISABLE);
//	DMA_ClearITPendingBit(SPI_DMA_STREAM,DMA_IT_TCIF0);
//	DMA_ClearFlag(SPI_DMA_STREAM,DMA_FLAG_TCIF0);
//	
//	if(buffer_flip==1 && (buffer_recv_index + 1 == buffer_send_index))
//	{
//		buffer_overflow = 1;
//		ad_start_flag = 0;
//		return;
//	}
//	
//	buffer_recv_index++;
//	if(buffer_recv_index == SPI_BUFFER_N)
//	{
//		buffer_recv_index = 0;
//		buffer_flip = 1;
//	}
//	
//	//开始下一次采集
//	DMA_SetCurrDataCounter(SPI_DMA_STREAM,SPI_BUFFSIZE);
//	DMA_MemoryTargetConfig(SPI_DMA_STREAM, (uint32_t)spi_recv_data + buffer_recv_index * SPI_BUFFSIZE, DMA_Memory_0);
//	EXTI9_Enable();
		
	
}

void ADS1274_Start(void)
{
	GPIO_PinState t = HAL_GPIO_ReadPin(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);
	if(t==1)
		return;
	
	buffer_recv_index = 0;
	buffer_send_index = 0;
	buffer_overflow = 0;

	printf("START\r\n");	
	HAL_GPIO_WritePin(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN,GPIO_PIN_SET);	//拉低禁止转换，拉高开始转换
}

static void ADS1274_Stop(void)
{
	uint8_t t = HAL_GPIO_ReadPin(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);
	if(t==0)
		return;
	
	HAL_GPIO_WritePin (AD_SYNC_GPIO,AD_SYNC_GPIO_PIN,GPIO_PIN_RESET);	//拉低禁止转换，拉高开始转换

	printf("STOP\r\n");
}

void ADS1274_run(void)
{
	if(ad_start_flag == 0)
		ADS1274_Stop();
	else
		ADS1274_Start();
}

void ADS1274_tcp_send_data(void)
{

}

uint8_t SPI_Read_Data(void)
{
	uint8_t Data[1];
	
		HAL_SPI_Receive(&hspi3,Data,1,1000);
	return Data[0];
}


