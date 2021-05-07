#include "data_sample.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "tcp_echoserver.h"
#include "lwip/err.h"
#include <stdio.h>

#define 	SPI_BUFFER_SIZE 	12
#define   SPI_BUFFER_N	    10
#define		SPI_DMA_STREAM DMA1_Stream0
#define		AD_SYNC_GPIO GPIOD
#define		AD_SYNC_GPIO_PIN	GPIO_PIN_8

extern err_t tcp_echoserver_send_data(struct tcp_echoserver_struct *es,void *payload,unsigned short int len);
extern struct tcp_echoserver_struct *client_es;//TCP客户端连接

uint8_t	ad_start_flag = 0;//AD启动标志
//	uint8_t Data[12];

uint8_t spi_ready=0;
uint8_t spi_recv_data_1[SPI_BUFFER_SIZE * SPI_BUFFER_N] = {0};//接收缓冲
uint8_t spi_recv_data_2[SPI_BUFFER_SIZE * SPI_BUFFER_N] = {0};//接收缓冲

uint8_t buffer_index=0;
uint8_t buffer_recv_index = 0;//当前接收缓冲指针
uint8_t buffer_send_index = 0;//当前发送缓冲指针
uint8_t buffer_flip = 0;//缓冲指针翻篇
uint8_t	buffer_overflow = 0;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
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
	MX_DMA_Init();
	MX_SPI3_Init();

}

/*	ADS1274数据准备好中断	*/
void EXTI9_5_IRQHandler(void)
{   
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);//自动清除中断标志
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	//HAL_SPI_Receive(&hspi3,Data,12,1000);
	//printf("%x %x",Data[0],Data[1]);
	
	spi_ready = 1;

	
}

//SPI DMA接收完成回调
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
//	printf("%d -",buffer_recv_index);
//	uint8_t n;
//	for(n=0;n<SPI_BUFFER_SIZE;n++)
//	{
//		printf(" %x",spi_recv_data[buffer_recv_index*SPI_BUFFER_SIZE + n]);
//	}
//	printf("\r\n");
	
	buffer_recv_index++;
	if(buffer_recv_index == SPI_BUFFER_N)
	{
		if(buffer_flip==1)
		{
			printf("Buffer overflow\r\n");
			buffer_overflow = 1;
			ad_start_flag = 0;
			return;
		}
		buffer_recv_index = 0;
		buffer_flip = 1;
		
		if(buffer_index==0) 
			buffer_index =1;
		else 
			buffer_index=0;
	}	
	
	exit:

		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

void DMA1_Stream5_IRQHandler(void)		//这里发送和接收要同时配置，否则中断标志位无法清除干净
{
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}



void ADS1274_Start(void)
{
	GPIO_PinState t = HAL_GPIO_ReadPin(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);
	if(t==1)
		return;
	
	buffer_recv_index = 0;
	buffer_send_index = 0;
	buffer_flip = 0;
	buffer_overflow = 0;
	buffer_index = 0;
	spi_ready = 0;
	
	HAL_SPI_Abort(&hspi3);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

//读取一次1274数据
void ADS1274_read_once(void)
{
	if(spi_ready == 0)
		return;
	
	uint8_t* buffer;
	if(buffer_index==0)
		buffer = spi_recv_data_1 + buffer_recv_index * SPI_BUFFER_SIZE;
	else
		buffer = spi_recv_data_2 + buffer_recv_index * SPI_BUFFER_SIZE;
	//HAL_StatusTypeDef ret = HAL_SPI_Receive_DMA(&hspi3,spi_recv_data + buffer_recv_index * SPI_BUFFER_SIZE,SPI_BUFFER_SIZE);
	HAL_StatusTypeDef ret = HAL_SPI_Receive_DMA(&hspi3,buffer,SPI_BUFFER_SIZE);
	if(HAL_OK!=ret)
	{
		printf("error dma spi3:%x\r\n",ret);
	}
	
	spi_ready = 0;
}

//从TCP端口发送数据
void ADS1274_tcp_send_data(void)
{
	if(buffer_flip==0)
		return;
	
		
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);//关数据准备好中断
	
	err_t ret;
	if(buffer_index==0)
		ret = tcp_echoserver_send_data(client_es, spi_recv_data_2, SPI_BUFFER_SIZE * SPI_BUFFER_N);
	else
		ret = tcp_echoserver_send_data(client_es, spi_recv_data_1, SPI_BUFFER_SIZE * SPI_BUFFER_N);
	//ret=0;
	//printf("x");
	if(ret!=ERR_OK)
	{
		//ad_start_flag=0;
		printf("Error tcp:%d\r\n",ret);
	}
	else
		buffer_flip=0;
	
	exit:
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);;
}




