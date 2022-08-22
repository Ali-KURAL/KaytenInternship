#include "Com.h"

bool Com_getRawData(uint8_t *data,uint8_t *data_len){
	bool retval = false;
	uint8_t temp[100];
	bool route_OK = Router_routeData(temp);
	if (route_OK){
		*data_len = temp[0];
		for(uint8_t i = 0;i<*data_len;i++) data[i] = temp[i+1];
		retval = true;
	}
	return retval;
}




void Com_init(void){
	Router_init();
}


bool Com_main(uint8_t * ans){
	bool retval = false;
	uint8_t comArr[100];
	uint8_t comArr_len = 0;
	bool com_OK = Com_getRawData(comArr,&comArr_len);
	if(com_OK){
		if (comArr_len == 32) *ans = OPEN_LED;
		if (comArr_len == 8) *ans = PWM_LED;
		if (comArr_len == 16) *ans = ADC_READ;
		if (comArr_len == 64) *ans = UART_READ;
		if (comArr_len == 128) *ans = FREERTOS;
		retval = true;
	}
	return retval;

}
