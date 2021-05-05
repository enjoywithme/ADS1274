#ifndef __DATA_SAMPLE_H
#define __DATA_SAMPLE_H

#include <stdint.h>


#ifdef __cplusplus
 extern "C" {
#endif

 
void ADS1274_Init(void);
void ADS1274_Start(void);
void ADS1274_run(void);
void ADS1274_tcp_send_data(void);
uint8_t SPI_Read_Data(void);

#ifdef __cplusplus
 }
#endif
	 
#endif /* __SPI_FLASH_H */

