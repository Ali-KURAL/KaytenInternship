#include "stm32l1xx_hal.h"
#include <stdbool.h>

#define UIF_BAUD_RATE 115200
#define UIF_WORD_LEN UART_WORDLENGTH_8B
#define UIF_STOP_BITS UART_STOPBITS_1
#define UIF_PARITY UART_PARITY_NONE

#define PACKET_LENGTH 8

typedef enum PACKET_ID{
	ID_A = 0xA,
	ID_B,
	ID_C,
	ID_D,
	ID_E
}PID;

bool UartIF_getID(uint8_t * msgID);
bool UartIF_checkID(uint8_t * buffer);
bool UartIF_checkCRC(uint8_t * buffer);
bool UartIF_getPacket(UART_HandleTypeDef *uart,uint8_t * packet);

void UartIF_init(UART_HandleTypeDef *uart);
void UartIF_main(UART_HandleTypeDef *uart,uint8_t* response);
