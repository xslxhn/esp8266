/*
 * si4463_port.c
 *
 *  Created on: 2018年7月13日
 *      Author: Administrator
 */
//------------------------------------------------------------------------------
#include "../xsl_bsp/si4463.h"
//
#define	ESP8266
//
#if		(defined(ESP8266))
#include "../xsl_config/includes.h"
#include "esp_common.h"
#include "gpio.h"
#define SI4463_NOP
#if		((HARDWARE_VER==33) && (HARDWARE_SUB_VER==1))
#define SI4463_SDN_H 		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(5), 1);
#define SI4463_SDN_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(5), 0);
#define SI4463_NSEL_H 	 		GPIO_OUTPUT_SET(GPIO_ID_PIN(15), 1);
#define SI4463_NSEL_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(15), 0);
#define SI4463_SCK_H 		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 1);
#define SI4463_SCK_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 0);
#define SI4463_MOSI_H 	 		GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 1);
#define SI4463_MOSI_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 0);
#define SI4463_MISO_PIN  		GPIO_INPUT_GET(GPIO_ID_PIN(12))
#define SI4463_NIRQ_PIN  		GPIO_INPUT_GET(GPIO_ID_PIN(2))
#define SI4463_CTS_PIN			GPIO_INPUT_GET(GPIO_ID_PIN(4))
#elif	((HARDWARE_VER==33) && (HARDWARE_SUB_VER==2))
#define SI4463_SDN_H 		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
#define SI4463_SDN_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
#define SI4463_NSEL_H 	 		GPIO_OUTPUT_SET(GPIO_ID_PIN(15), 1);
#define SI4463_NSEL_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(15), 0);
#define SI4463_SCK_H 		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 1);
#define SI4463_SCK_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 0);
#define SI4463_MOSI_H 	 		GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 1);
#define SI4463_MOSI_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 0);
#define SI4463_MISO_PIN  		GPIO_INPUT_GET(GPIO_ID_PIN(12))
#define SI4463_NIRQ_PIN  		GPIO_INPUT_GET(GPIO_ID_PIN(5))
#define SI4463_CTS_PIN			GPIO_INPUT_GET(GPIO_ID_PIN(4))
#else
#define SI4463_SDN_H 		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);
#define SI4463_SDN_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);
#define SI4463_NSEL_H 	 		GPIO_OUTPUT_SET(GPIO_ID_PIN(15), 1);
#define SI4463_NSEL_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(15), 0);
#define SI4463_SCK_H 		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 1);
#define SI4463_SCK_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 0);
#define SI4463_MOSI_H 	 		GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 1);
#define SI4463_MOSI_L		 	GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 0);
#define SI4463_MISO_PIN  		GPIO_INPUT_GET(GPIO_ID_PIN(12))
#define SI4463_NIRQ_PIN  		GPIO_INPUT_GET(GPIO_ID_PIN(5))
#define SI4463_CTS_PIN			GPIO_INPUT_GET(GPIO_ID_PIN(4))
#endif
#define SI4463_PRINTF			printf
#elif	(defined(STM32F1)||defined(STM32F4))
#define SI4463_NOP
#define SI4463_SDN_H 		 	GPIO_SetBits(GPIOA , GPIO_Pin_11)
#define SI4463_SDN_L		 	GPIO_ResetBits(GPIOA , GPIO_Pin_11)
#define SI4463_NSEL_H 	 		GPIO_SetBits(GPIOB , GPIO_Pin_12)
#define SI4463_NSEL_L		 	GPIO_ResetBits(GPIOB , GPIO_Pin_12)
#define SI4463_SCK_H 		 	GPIO_SetBits(GPIOB , GPIO_Pin_13)
#define SI4463_SCK_L		 	GPIO_ResetBits(GPIOB , GPIO_Pin_13)
#define SI4463_MOSI_H 	 		GPIO_SetBits(GPIOB , GPIO_Pin_15)
#define SI4463_MOSI_L		 	GPIO_ResetBits(GPIOB , GPIO_Pin_15)
#define SI4463_MISO_PIN  		(GPIOB->IDR & GPIO_Pin_14)
#define SI4463_NIRQ_PIN  		(GPIOA->IDR & GPIO_Pin_12)
#elif	(defined(NRF51)||defined(NRF52))
#define SI4463_NOP
#define SI4463_SDN_H
#define SI4463_SDN_L
#define SI4463_NSEL_H
#define SI4463_NSEL_L
#define SI4463_SCK_H
#define SI4463_SCK_L
#define SI4463_MOSI_H
#define SI4463_MOSI_L
#define SI4463_MISO_PIN
#define SI4463_NIRQ_PIN
#endif
//
#define	APP_PACKET_LEN	((uint16_t)21)
si4463_t si4463;
uint8_t incomingBuffer[APP_PACKET_LEN];
uint8_t outgoingBuffer[APP_PACKET_LEN];

uint8_t Si4463_PartInfo[9] = { 0 };
extern uint8_t UdpClient_Cmd_TxKey;

uint8_t si4463_Out_RxBuf[APP_PACKET_LEN] = { 0 };
/*******************************************************************************
 * 								Input Port
 */
static uint8_t SI4463_IsCTS(void) {
	if (SI4463_CTS_PIN == 1) {
		return true;
	} else {
		return false;
	}
}
static void SI4463_WriteRead(const uint8_t * pTxData, uint8_t * pRxData,
		const uint16_t sizeTxData) {
	uint8_t i;
	uint8_t tx, rx;
	uint16_t i16;
	for (i16 = 0; i16 < sizeTxData; i16++) {
		SI4463_SCK_L
		;SI4463_NOP;
		tx = pTxData[i16];
		rx = 0;
		for (i = 0; i < 8; i++) {
			//tx
			if (tx & 0x80) {
				SI4463_MOSI_H
				;
			} else {
				SI4463_MOSI_L
				;
			}SI4463_NOP;
			tx = tx << 1;
			//rx
			rx = rx << 1;
			SI4463_SCK_H
			;SI4463_NOP;
			if (SI4463_MISO_PIN) {
				rx++;
			}
			SI4463_SCK_L
			;SI4463_NOP;
		}
		if (pRxData != NULL) {
			pRxData[i16] = rx;
		}
	}
}
static void SI4463_SetShutdown(void) {
	SI4463_SDN_H
	;
}
static void SI4463_ClearShutdown(void) {
	SI4463_SDN_L
	;
}
static void SI4463_Select(void) {
	SI4463_NSEL_L
	;
}
static void SI4463_Deselect(void) {
	SI4463_NSEL_H
	;
}
static void SI4463_DelayMs(uint32_t i32) {
	while (i32) {
		i32--;
		os_delay_us(1000);
	}
}

//
uint8_t SI4463_Read_Byte(void) {
	uint8_t temp;
	uint8_t byte = 0x00;
	SI4463_SCK_L
	;SI4463_NOP;
	for (temp = 0; temp < 8; temp++) {
		byte = byte << 1;
		SI4463_SCK_H
		;SI4463_NOP;
		if (SI4463_MISO_PIN) {
			byte++;
		}
		SI4463_SCK_L
		;SI4463_NOP;
	}
	return (byte);
}
void SI4463_Write_Byte(uint8_t byte) {
	uint8_t temp;
	SI4463_SCK_L
	;SI4463_NOP;
	for (temp = 0; temp < 8; temp++) {
		if (byte & 0x80) {
			SI4463_MOSI_H
			;
		} else {
			SI4463_MOSI_L
			;
		}SI4463_NOP;
		byte = byte << 1;
		SI4463_SCK_H
		;SI4463_NOP;
		SI4463_SCK_L
		;SI4463_NOP;
	}
}
static void SI4463_Interrupt(void) {
	/* Prevent unused argument(s) compilation warning */
	//-----XSL-----del
	// UNUSED(GPIO_Pin);
	//-------------
	/* Clear incoming buffer */
	memset(incomingBuffer, 0x00, APP_PACKET_LEN);
	//SI4463_PRINTF("SI4463 Interrupt");
	/* Get interrupts and work with it */
	SI4463_GetInterrupts(&si4463);

	/* Handling PH interrupts */
	if (si4463.interrupts.filterMatch) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.filterMatch = false;
	}
	if (si4463.interrupts.filterMiss) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.filterMiss = false;
	}
	if (si4463.interrupts.packetSent) {
		/* Handling this interrupt here */
		/* Clear TX FIFO */
		SI4463_ClearTxFifo(&si4463);
		//-----XSL-----del
#if 0
		HAL_UART_Transmit(&huart1, "OUT >", 5, 10);
		HAL_UART_Transmit(&huart1, outgoingBuffer, APP_PACKET_LEN, 10);
		HAL_UART_Transmit(&huart1, "\n", 1, 10);
#endif /* DEMOFEST */
		//-------------
#ifdef	SI4463_PRINTF
		//SI4463_PRINTF("OUT >");
#endif
		/* Re-arm StartRX */
		SI4463_StartRx(&si4463, APP_PACKET_LEN, false, false, false);

		/*Toggle led for indication*/
		//-----XSL-----del
		// HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
		//-------------
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.packetSent = false;
	}
	if (si4463.interrupts.packetRx) {
		/* Handling this interrupt here */
		/* Get FIFO data */
		SI4463_ReadRxFifo(&si4463, incomingBuffer, APP_PACKET_LEN);
		/* Clear RX FIFO */
		SI4463_ClearRxFifo(&si4463);
		//-----XSL-----del
#if	0
		HAL_UART_Transmit(&huart1, "IN >", 4, 10);
		HAL_UART_Transmit(&huart1, incomingBuffer, APP_PACKET_LEN, 10);
		HAL_UART_Transmit(&huart1, "\n", 1, 10);
#endif /* DEMOFEST */
		//-------------
		if (si4463_Out_RxBuf[0] == 0) {
			memcpy(si4463_Out_RxBuf,incomingBuffer,APP_PACKET_LEN);
			UdpClient_Cmd_TxKey = 1;
		}
#ifdef	SI4463_PRINTF
		{
			uint8_t buf[100];
			sprintf(buf,
					"Rx:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
					incomingBuffer[0], incomingBuffer[1], incomingBuffer[2],
					incomingBuffer[3], incomingBuffer[4], incomingBuffer[5],
					incomingBuffer[6], incomingBuffer[7], incomingBuffer[8],
					incomingBuffer[9], incomingBuffer[10], incomingBuffer[11],
					incomingBuffer[12], incomingBuffer[13], incomingBuffer[14],
					incomingBuffer[15], incomingBuffer[16], incomingBuffer[17],
					incomingBuffer[18], incomingBuffer[19], incomingBuffer[20],
					incomingBuffer[21]);
			SI4463_PRINTF(buf);
		}
#endif

		/* Start RX again.
		 * It need because after successful receive a packet the chip change
		 * state to READY.
		 * There is re-armed mode for StartRx but it not correctly working */
		SI4463_StartRx(&si4463, APP_PACKET_LEN, false, false, false);

		/*Toggle led for indication*/
//-----XSL-----del
// HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
//-------------
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.packetRx = false;
	}
	if (si4463.interrupts.crcError) {
		/* Handling this interrupt here */

		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.crcError = false;
	}
	if (si4463.interrupts.txFifoAlmostEmpty) {
		/* Handling this interrupt here */

		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.txFifoAlmostEmpty = false;
	}
	if (si4463.interrupts.rxFifoAlmostFull) {
		/* Handling this interrupt here */

		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.rxFifoAlmostFull = false;
	}

	/* Handling Modem interrupts */
	if (si4463.interrupts.postambleDetect) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.postambleDetect = false;
	}
	if (si4463.interrupts.invalidSync) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.invalidSync = false;
	}
	if (si4463.interrupts.rssiJump) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.rssiJump = false;
	}
	if (si4463.interrupts.rssi) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.rssi = false;
	}
	if (si4463.interrupts.invalidPreamble) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.invalidPreamble = false;
	}
	if (si4463.interrupts.preambleDetect) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.preambleDetect = false;
	}
	if (si4463.interrupts.syncDetect) {
		/* Handling this interrupt here */
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.syncDetect = false;
	}

	/* Handling Chip interrupts */
	if (si4463.interrupts.cal) {
		/* Handling this interrupt here */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.cal = false;
	}
	if (si4463.interrupts.fifoUnderflowOverflowError) {
		/* Handling this interrupt here */
		/* Clear RX FIFO */
		SI4463_ClearRxFifo(&si4463);
		/* Claer Chip Status errors if exists */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.fifoUnderflowOverflowError = false;
	}
	if (si4463.interrupts.stateChange) {
		/* Handling this interrupt here */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.stateChange = false;
	}
	if (si4463.interrupts.cmdError) {
		/* Handling this interrupt here */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.stateChange = false;
	}
	if (si4463.interrupts.chipReady) {
		/* Handling this interrupt here */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.chipReady = false;
	}
	if (si4463.interrupts.lowBatt) {
		/* Handling this interrupt here */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.lowBatt = false;
	}
	if (si4463.interrupts.wut) {
		/* Handling this interrupt here */
		SI4463_GetChipStatus(&si4463);
		/* Following instruction only for add breakpoints. May be deleted */
		si4463.interrupts.wut = false;
	}

	/* Clear All interrupts before exit */
	SI4463_ClearAllInterrupts(&si4463);
}
/*******************************************************************************
 * 								Output Port
 */
void si4463_port_init(void) {
	printf("SI4463 Init Begin!\r\n");
	/* Assign functions */
	si4463.IsClearToSend = SI4463_IsCTS;
	si4463.WriteRead = SI4463_WriteRead;
	si4463.Select = SI4463_Select;
	si4463.Deselect = SI4463_Deselect;
	si4463.SetShutdown = SI4463_SetShutdown;
	si4463.ClearShutdown = SI4463_ClearShutdown;
	si4463.DelayMs = SI4463_DelayMs;
	/* Disable interrupt pin for init Si4463 */
// HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	/* Init Si4463 with structure */
	SI4463_Init(&si4463);
#if 1
	/* Clear RX FIFO before starting RX packets */
	SI4463_ClearRxFifo(&si4463);
	/* Start RX mode.
	 * SI4463_StartRx() put a chip in non-armed mode in cases:
	 * - successfully receive a packet;
	 * - invoked RX_TIMEOUT;
	 * - invalid receive.
	 * For receiveing next packet you have to invoke SI4463_StartRx() again!*/
	SI4463_StartRx(&si4463, APP_PACKET_LEN, false, false, false);
#else
	SI4463_ClearTxFifo(&si4463);
#endif
	/* Enable interrupt pin and */
// HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	/* Clear interrupts after enabling interrupt pin.
	 * Without it may be situation when interrupt is asserted but pin not cleared.*/
	SI4463_ClearInterrupts(&si4463);
	printf("SI4463 Init OK!\r\n");
// 芯片打印
	{
		static uint8_t buf[10] = { 0 };
		buf[0] = SI4463_GetPartInfo(&si4463, buf);
		printf(
				"Si4463 PartInfo(Err=%02x): %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
				buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
				buf[8], buf[9]);
		Si4463_PartInfo[0] = buf[1];
		Si4463_PartInfo[1] = buf[2];
		Si4463_PartInfo[2] = buf[3];
		Si4463_PartInfo[3] = buf[4];
		Si4463_PartInfo[4] = buf[5];
		Si4463_PartInfo[5] = buf[6];
		Si4463_PartInfo[6] = buf[7];
		Si4463_PartInfo[7] = buf[8];
		Si4463_PartInfo[8] = buf[9];
	}
}
void SI4463_Send(uint8_t *pbuf, uint8_t len) {
	SI4463_Transmit(&si4463, pbuf, len);
}
void SI4463_PeriodFun(void) {
	static uint8_t si = 0;
	if (0 == SI4463_NIRQ_PIN) {
		SI4463_Interrupt();
	}
	if (si++ >= 100) {
		si = 0;
		SI4463_Transmit(&si4463, "012345678901234567890", APP_PACKET_LEN);
	}
}
/*******************************************************************************
 * 								App Port
 */
void SI4463_AppSend(void) {
	uint8_t buf[APP_PACKET_LEN];
	uint8_t i = 0;
	// 设备类型	(硬件版本号)
	buf[i++] = HARDWARE_VER;
	// 设备ID		(Si4463的芯片ID)
	//buf[i++] = Si4463_PartInfo[1];
	buf[i++] = Si4463_PartInfo[2];
	buf[i++] = Si4463_PartInfo[3];
	buf[i++] = Si4463_PartInfo[4];
	buf[i++] = Si4463_PartInfo[5];
	buf[i++] = Si4463_PartInfo[6];
	buf[i++] = Si4463_PartInfo[7];
	buf[i++] = Si4463_PartInfo[8];
	// 消息类型
	buf[i++] = 5;
	// 消息数据
	buf[i++] = 2;
	//
	SI4463_Transmit(&si4463, buf, APP_PACKET_LEN);
}
