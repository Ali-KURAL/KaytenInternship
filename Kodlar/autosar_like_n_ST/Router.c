#include "Router.h"

void Router_init(void){
	UartTP_init();
}

bool Router_routeData(uint8_t *comArr){
	bool retval = false;
	uint8_t response[100] = {0};
	bool data_OK = UartTP_getFullData(response);
	if (data_OK){
		for(uint8_t i = 0;i<response[0];i++) comArr[i] = response[i];
		retval = true;
	}
	return retval;
}

bool Router_main(uint8_t *comArr){
	Router_routeData(comArr);
	return true;
}
