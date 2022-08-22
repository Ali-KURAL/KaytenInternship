#include "UartIF.h"

#define MESSAGE_LEN_A 64
#define MESSAGE_LEN_B 8
#define MESSAGE_LEN_C 16
#define MESSAGE_LEN_D 32
#define MESSAGE_LEN_E 128


#define DATA_LEN_A (int)((MESSAGE_LEN_A / 8) * 5)
#define DATA_LEN_B (int)((MESSAGE_LEN_B / 8) * 5)
#define DATA_LEN_C (int)((MESSAGE_LEN_C / 8) * 5)
#define DATA_LEN_D (int)((MESSAGE_LEN_D / 8) * 5)
#define DATA_LEN_E (int)((MESSAGE_LEN_E / 8) * 5)


static uint8_t msg_arr_a[MESSAGE_LEN_A];
static uint8_t msg_arr_b[MESSAGE_LEN_B];
static uint8_t msg_arr_c[MESSAGE_LEN_C];
static uint8_t msg_arr_d[MESSAGE_LEN_D];
static uint8_t msg_arr_e[MESSAGE_LEN_E];

static uint8_t data_arr_a[DATA_LEN_A];
static uint8_t data_arr_b[DATA_LEN_B];
static uint8_t data_arr_c[DATA_LEN_C];
static uint8_t data_arr_d[DATA_LEN_D];
static uint8_t data_arr_e[DATA_LEN_E];

static uint8_t msg_idx_a;
static uint8_t msg_idx_b;
static uint8_t msg_idx_c;
static uint8_t msg_idx_d;
static uint8_t msg_idx_e;

static uint8_t data_idx_a;
static uint8_t data_idx_b;
static uint8_t data_idx_c;
static uint8_t data_idx_d;
static uint8_t data_idx_e;

void UartTP_savePacketData(uint8_t *message);
bool UartTP_savePackets(void);
bool UartTP_checkComDone(uint8_t *commandDone);
bool UartTP_messageDone(uint8_t * message, uint8_t * message_len);
bool UartTP_getFullData(uint8_t *message);

void UartTP_init(void);
void UartTP_main(uint8_t* response);
