#include "data_sample.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include <stdio.h>
#include "udp_echoclient.h"

#define SPI_BUFFER_SIZE 	12
#define SPI_BUFFER_N	    60
#define	AD_SYNC_GPIO GPIOD
#define	AD_SYNC_GPIO_PIN	GPIO_PIN_8
#define	SEND_LINES_LIMIT
uint8_t ads1274_irq_disable_counter = 0;

uint8_t ad_start_flag = 0; //AD启动标志

uint8_t spi_recv_data[SPI_BUFFER_SIZE] = { 0 }; //接收缓冲
uint8_t spi_send_data[SPI_BUFFER_SIZE] = { 0 };

struct pbuf *pbuf1;
struct pbuf *pbuf2;
struct pbuf *p;
uint8_t spi_ready = 0;

uint8_t buffer_recv_index = 0; //当前接收缓冲指针
uint8_t buffer_send_index = 0; //当前发送缓冲指针
uint8_t buffer_flip = 0; //缓冲指针翻篇
uint8_t buffer_overflow = 0;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void ADS1274_irq_enable(void);
static void ADS1274_irq_disable(void);

uint32_t tickstart = 0;
uint32_t ext9_int = 0;
uint32_t dma_rx = 0;
uint32_t dma_tx = 0;
uint32_t seconds = 0;
uint8_t reading = 0;

void Debug_Int_Count(void) {
	uint32_t tick;
	if (tickstart == 0)
		tickstart = HAL_GetTick();
	else {
		tick = HAL_GetTick();
		if (tick - tickstart > 1000) {
			seconds++;
			printf("seconds=%d,tick=%d,ext9_int=%d,dma_rx=%d,dma_tx=%d\r\n",
					seconds, tick - tickstart, ext9_int, dma_rx, dma_tx);
			tickstart = 0;
			dma_rx = 0;
			dma_tx = 0;
			ext9_int = 0;
		}
	}
}

//#define RECV_DEBUG
void DMA1_Stream0_IRQHandler(void) {
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_ClearFlag_TC0(DMA1);
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);

	dma_rx++;

#ifdef RECV_DEBUG
printf("%d -",buffer_recv_index);
uint8_t n;
for(n=0;n<SPI_BUFFER_SIZE;n++)
{
	printf(" %x",spi_recv_data[buffer_recv_index*SPI_BUFFER_SIZE + n]);
}
printf("\r\n");
#endif

}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void) {

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
	LL_DMA_ClearFlag_TC5(DMA1);
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);

	LL_SPI_Disable(SPI3);

	dma_tx++;

	if(buffer_flip==1)
		p = pbuf2;
	else
		p = pbuf1;

	if (ERR_OK
			!= pbuf_take_at(p, spi_recv_data, SPI_BUFFER_SIZE,
					buffer_recv_index * SPI_BUFFER_SIZE)) {
		printf("Failed to copy data\r\n");
		goto error_exit;
	}

	buffer_recv_index++;
	if (buffer_recv_index == SPI_BUFFER_N) {
		p->len = SPI_BUFFER_SIZE * SPI_BUFFER_N;
		p->tot_len = p->len;

		if(buffer_flip==1)
			buffer_flip = 0;
		else
			buffer_flip = 1;

		buffer_recv_index = 0;
		spi_ready=1;
		//current_pbuf = NULL;
	}

	return;

	error_exit:
			ad_start_flag = 0;

}

void ADS1274_Send_Data(void)
{
	if(spi_ready==0)
		return;

	if(buffer_flip==1)
		udp_echoclient_Send(pbuf1);
	else
		udp_echoclient_Send(pbuf2);

	spi_ready=0;
}

/*	ADS1274数据准备好中断	*/
void EXTI9_5_IRQHandler(void) {

	//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET) {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
	}
	ext9_int++;
	ADS1274_read_once();

}

//读取一次1274数据
void ADS1274_read_once(void) {
	//if(reading==1) return;
	//reading=1;
	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t) spi_recv_data);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, SPI_BUFFER_SIZE);
	LL_SPI_EnableDMAReq_RX(SPI3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_0);
	LL_SPI_EnableDMAReq_TX(SPI3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

	LL_SPI_Enable(SPI3);
//	uint8_t n;
//	for(n=0;n<12;n++)
//		LL_SPI_TransmitData8(SPI3,1);
}

void ADS1274_Start(void) {
	GPIO_PinState t = HAL_GPIO_ReadPin(AD_SYNC_GPIO, AD_SYNC_GPIO_PIN);
	if (t == 1)
		return;

	buffer_recv_index = 0;
	buffer_send_index = 0;
	buffer_flip = 0;
	buffer_overflow = 0;

	spi_ready = 0;
	seconds = 0;
	ads1274_irq_disable_counter = 0;

	if(pbuf1==NULL)
		pbuf1 = pbuf_alloc(PBUF_TRANSPORT,
					SPI_BUFFER_SIZE * SPI_BUFFER_N, PBUF_POOL);
	if(pbuf1==NULL)
	{
		printf("Failed to allocate pbuf1\r\n");
		goto error_exit;
	}

	if(pbuf2==NULL)
		pbuf2 = pbuf_alloc(PBUF_TRANSPORT,
					SPI_BUFFER_SIZE * SPI_BUFFER_N, PBUF_POOL);
	if(pbuf2==NULL)
	{
		printf("Failed to allocate pbuf2\r\n");
		goto error_exit;
	}

	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	printf("START\r\n");
	HAL_GPIO_WritePin(AD_SYNC_GPIO, AD_SYNC_GPIO_PIN, GPIO_PIN_SET);//拉低禁止转换，拉高开始转换

	return;

	error_exit:
		printf("Unable to start.\r\n");
}

static void ADS1274_Stop(void) {
	uint8_t t = HAL_GPIO_ReadPin(AD_SYNC_GPIO, AD_SYNC_GPIO_PIN);
	if (t == 0)
		return;
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_GPIO_WritePin(AD_SYNC_GPIO, AD_SYNC_GPIO_PIN, GPIO_PIN_RESET);//拉低禁止转换，拉高开始转换

	printf("STOP\r\n");
}

void ADS1274_run(void) {
	if (ad_start_flag == 0)
		ADS1274_Stop();
	else
		ADS1274_Start();
}
static void ADS1274_irq_enable(void) {
	if (ads1274_irq_disable_counter > 0)
		ads1274_irq_disable_counter--;

	if (ads1274_irq_disable_counter == 0)
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	//printf("irq=%d",ads1274_irq_disable_counter);
}

static void ADS1274_irq_disable(void) {
	ads1274_irq_disable_counter++;

	if (ads1274_irq_disable_counter > 0)
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	//printf("irq=%d",ads1274_irq_disable_counter);
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	LL_SPI_InitTypeDef SPI_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	/**SPI3 GPIO Configuration
	 PC10   ------> SPI3_SCK
	 PC11   ------> SPI3_MISO
	 PC12   ------> SPI3_MOSI
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* SPI3 DMA Init */

	/* SPI3_RX Init */
	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_SPI_DMA_GetRegAddr(SPI3));
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, SPI_BUFFER_SIZE);
	//LL_DMA_EnableIT_TE(DMA1,LL_DMA_STREAM_0);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

	/* SPI3_TX Init */
	LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_0);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

	//LL_DMA_EnableIT_TE(DMA1,LL_DMA_STREAM_5);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t) spi_send_data);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, LL_SPI_DMA_GetRegAddr(SPI3));
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, SPI_BUFFER_SIZE);

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(SPI3, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* Init with LL driver */
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Stream0_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Stream5_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	/**/
	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_8 | LL_GPIO_PIN_12);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE9);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);

	/**/
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);

	/* EXTI interrupt init*/
	NVIC_SetPriority(EXTI9_5_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_DisableIRQ(EXTI9_5_IRQn);

}

void ADS1274_Init(void) {
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI3_Init();

}

