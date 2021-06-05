/**
 ******************************************************************************
 * @file    udp_echoclient.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    31-October-2011
 * @brief   UDP echo client
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "tcp_echoserver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define UDP_SERVER_PORT 8

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


extern struct tcp_echoserver_struct *client_es;

__IO uint8_t Flag = 0;
struct udp_pcb *upcb;
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Connect to UDP echo server
 * @param  None
 * @retval None
 */
void udp_echoclient_Send(struct pbuf *p) {
//   struct udp_pcb *upcb;

	ip_addr_t DestIPaddr;
	err_t err;

	if (Flag == 0) {
		Flag = 1;
		/* Create a new UDP control block  */
		upcb = udp_new();
	} else {

		if (p != NULL) {
			/* send udp data */
			udp_send(upcb, p);

			/* free pbuf */
			//pbuf_free(p);
		}
		return;
	}

	if (upcb != NULL) {
		/*assign destination IP address */
		//IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
		/* configure destination IP address and port */
		DestIPaddr = client_es->pcb->remote_ip;
		err = udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);

		if (err == ERR_OK) {
			/* Set a receive callback for the upcb */
			//udp_recv(upcb, udp_receive_callback, NULL);

			if (p != NULL) {
				/* send udp data */
				udp_send(upcb, p);

				/* free pbuf */
				//pbuf_free(p);
			}
		} else {
			printf("\n\r can not connect udp pcb");
		}
	} else {

		printf("\n\r can not create udp pcb");

	}

	//if(p !=NULL)
	//	pbuf_free(p);
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
