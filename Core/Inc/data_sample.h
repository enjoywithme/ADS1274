#ifndef __DATA_SAMPLE_H
#define __DATA_SAMPLE_H

#include <stdint.h>


#ifdef __cplusplus
 extern "C" {
#endif

typedef enum 
{
  DE_NO_CONNECTION       = 0x00U,
  DE_SEND_ERROR    = 0x01U

} DATA_SAMPLE_ERROR_CODE;
 
void ADS1274_Init(void);
void ADS1274_Start(void);
void ADS1274_run(void);
void ADS1274_tcp_send_data(void);
void ADS1274_read_once(void);

#ifdef __cplusplus
 }
#endif
	 
#endif /* __SPI_FLASH_H */

