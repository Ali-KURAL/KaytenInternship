#include "UartIF.h"


// PRIVATE FUNCTIONS
bool CRC8(uint8_t * checkingArray,uint16_t size);

uint8_t rxBuffer[PACKET_LENGTH];
bool msg_DONE = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    HAL_UART_Receive_DMA(huart, rxBuffer, PACKET_LENGTH);
		msg_DONE = true;
}

void UartIF_init(UART_HandleTypeDef *uart){
	/* USER CODE END USART2_Init 1 */
  uart->Instance = USART2;
  uart->Init.BaudRate = UIF_BAUD_RATE;
  uart->Init.WordLength = UIF_WORD_LEN;
  uart->Init.StopBits = UIF_STOP_BITS;
  uart->Init.Parity = UIF_PARITY;
  uart->Init.Mode = UART_MODE_TX_RX;
  uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(uart) != HAL_OK)
  {
    return;
  }
	HAL_UART_Receive_DMA(uart,rxBuffer,8);
}

bool UartIF_getID(uint8_t * msgID){
	bool retval = false;
	bool idCheck = UartIF_checkID(rxBuffer);
	if (idCheck){
		*msgID = rxBuffer[0];
		retval = true;
	}
	return retval;
}



bool UartIF_checkID(uint8_t * buffer){
	bool retval = false;
	uint8_t id = buffer[0];
	if (id == ID_A || id == ID_B || id == ID_C || id == ID_D || id == ID_E) retval = true;
	return retval;
}


bool UartIF_checkCRC(uint8_t * buffer){
	bool retval = false;
	volatile bool crc_OK = CRC8(rxBuffer,PACKET_LENGTH);
	if (crc_OK == true){
		retval = true;
	}
	return retval;
}



bool UartIF_getPacket(UART_HandleTypeDef *uart,uint8_t * packet){
	bool retval = false;
	volatile bool id_OK = UartIF_checkID(rxBuffer);
	if (id_OK == true){
		bool packet_OK = UartIF_checkCRC(rxBuffer);
		if( packet_OK == true){
			retval = true;
			msg_DONE = false;
			for(uint8_t i = 0; i<PACKET_LENGTH;i++) packet[i] = rxBuffer[i];
			for(uint8_t i = 0; i < PACKET_LENGTH; i++) rxBuffer[i] = 0;
		}	
	}
	return retval;
}

void UartIF_main(UART_HandleTypeDef *uart,uint8_t* response){
	UartIF_getPacket(uart,response);	
	return;
}


bool CRC8(uint8_t * checkingArray,uint16_t size){
	uint8_t crc = 0x00;
	for (uint8_t i = 0; i < size-1;i++){
		crc ^= checkingArray[i];
	}
	if (crc == checkingArray[size-1]) return true;
	else return false;
}

