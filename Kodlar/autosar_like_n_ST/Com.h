#include "Router.h"

typedef enum operationCom{
	NO_OPERATION,
	OPEN_LED,
	PWM_LED,
	ADC_READ,
	UART_READ,
	FREERTOS
}operationCom;

void Com_init(void);
bool Com_getRawData(uint8_t *data,uint8_t *data_len);
void Com_convertData(operationCom *operation);

bool Com_main(uint8_t *ans);

