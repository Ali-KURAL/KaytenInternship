#include "UartTP.h"

UART_HandleTypeDef uart;

void UartTP_init(void){
	UartIF_init(&uart);
}

void UartTP_main(uint8_t* response){
	UartTP_getFullData(response);
}


bool UartTP_getFullData(uint8_t* response){
	bool retval = false;
	volatile bool save_OK = UartTP_savePackets();
	uint8_t command_done[100] = {0};
	volatile bool com_OK = UartTP_checkComDone(command_done);
	if (com_OK){
		retval = true;
		for(uint8_t i = 0; i< 100;i++) response[i] = command_done[i];
	}
	return retval;
}

void UartTP_savePacketData(uint8_t *packet){
	switch(packet[0]){
		case ID_A:
			for(uint8_t i = 2; i < 7;i++) data_arr_a[data_idx_a++]=packet[i];
			break;
		case ID_B:
			for(uint8_t i = 2; i < 7;i++) data_arr_b[data_idx_b++]=packet[i];
			break;
		case ID_C:
			for(uint8_t i = 2; i < 7;i++) data_arr_c[data_idx_c++]=packet[i];
			break;
		case ID_D:
			for(uint8_t i = 2; i < 7;i++) data_arr_d[data_idx_d++]=packet[i];
			break;
		case ID_E:
			for(uint8_t i = 2; i < 7;i++) data_arr_e[data_idx_e++]=packet[i];
			break;
		default:
			return;
		}
}

bool UartTP_savePackets(void){
	bool retval = false;
	uint8_t packet[PACKET_LENGTH] = {0};
	volatile bool packet_OK;
	packet_OK = UartIF_getPacket(&uart,packet);
	if (packet_OK){
		switch(packet[0]){
		case ID_A:
			for(uint8_t i = 0; i < 8;i++) msg_arr_a[msg_idx_a++]=packet[i];
			UartTP_savePacketData(packet);
			retval = true;
			break;
		case ID_B:
			for(uint8_t i = 0; i < 8;i++) msg_arr_b[msg_idx_b++]=packet[i];
			UartTP_savePacketData(packet);
			retval = true;
			break;
		case ID_C:
			for(uint8_t i = 0; i < 8;i++) msg_arr_c[msg_idx_c++]=packet[i];
			UartTP_savePacketData(packet);
			retval = true;
			break;
		case ID_D:
			for(uint8_t i = 0; i < 8;i++) msg_arr_d[msg_idx_d++]=packet[i];
			UartTP_savePacketData(packet);
			retval = true;
			break;
		case ID_E:
			for(uint8_t i = 0; i < 8;i++) msg_arr_e[msg_idx_e++]=packet[i];
			UartTP_savePacketData(packet);
			retval = true;
			break;
		}
	
	}
	return retval;
	
	
}


bool UartTP_checkComDone(uint8_t *commandDone){
	bool com_OK = false;
	if (msg_idx_a == MESSAGE_LEN_A){
		com_OK = true;
		commandDone[0] = DATA_LEN_A;
		for(uint8_t i = 1;i<DATA_LEN_A;i++) commandDone[i+1] = data_arr_a[i];
		for(uint8_t i = 0;i<DATA_LEN_A;i++) data_arr_a[i] = 0;
		for(uint8_t i = 0;i<MESSAGE_LEN_A;i++) msg_arr_a[i] = 0;
		msg_idx_a = 0;
		data_idx_a = 0;
	}
	if (msg_idx_b == MESSAGE_LEN_B){
		com_OK = true;
		commandDone[0] = DATA_LEN_B;
		for(uint8_t i = 0;i<DATA_LEN_B;i++) commandDone[i+1] = data_arr_b[i];
		for(uint8_t i = 0;i<DATA_LEN_B;i++) data_arr_b[i] = 0;
		msg_idx_b = 0;
		data_idx_b = 0;
	}
	if (msg_idx_c == MESSAGE_LEN_C){
		com_OK = true;
		commandDone[0] = DATA_LEN_C;
		for(uint8_t i = 0;i<DATA_LEN_C;i++) commandDone[i+1] = data_arr_c[i];
		for(uint8_t i = 0;i<DATA_LEN_C;i++) data_arr_c[i] = 0;
		msg_idx_c = 0;
		data_idx_c = 0;
	}
	if (msg_idx_d == MESSAGE_LEN_D){
		com_OK = true;
		commandDone[0] = DATA_LEN_D;
		for(uint8_t i = 0;i<DATA_LEN_D;i++) commandDone[i+1] = data_arr_d[i];
		for(uint8_t i = 0;i<DATA_LEN_D;i++) data_arr_d[i] = 0;
		for(uint8_t i = 0;i<MESSAGE_LEN_D;i++) msg_arr_d[i] = 0;
		msg_idx_d = 0;
		data_idx_d = 0;
	}
	if (msg_idx_e == MESSAGE_LEN_E){
		com_OK = true;
		commandDone[0] = DATA_LEN_E;
		for(uint8_t i = 0;i<DATA_LEN_E;i++) commandDone[i+1] = data_arr_e[i];
		for(uint8_t i = 0;i<DATA_LEN_E;i++) data_arr_e[i] = 0;
		msg_idx_e = 0;
		data_idx_e = 0;
	}
	return com_OK;
}
