#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef union uart_com{
    int uart_buffer_int[10];
    char uart_buffer_char[10];
}uart_com;

int main() {
    uart_com* msg = malloc(sizeof (uart_com));
    uart_com* msg2 = malloc(sizeof (uart_com));
    msg->uart_buffer_char[0] = 62;
    msg->uart_buffer_char[1] = 62;
    msg->uart_buffer_char[2] = 33;
    msg->uart_buffer_char[3] = 33;
    msg->uart_buffer_char[4] = 77;
    msg->uart_buffer_char[5] = 66;
    msg->uart_buffer_char[6] = 13;
    msg->uart_buffer_char[7] = 10;
    msg->uart_buffer_char[8] = 0;
    msg->uart_buffer_char[9] = 0;

    msg2->uart_buffer_char[0] = 65;
    msg2->uart_buffer_char[1] = 65;
    msg2->uart_buffer_char[2] = 32;

    printf("%s", msg->uart_buffer_char);
    printf("%s", msg2->uart_buffer_char);
//    char test_text[10] = "ON\r\nOFF\r\n";
//    if (strstr(test_text,"\r\n")){
//        printf("I have that");
//    }
//    int count = 0;
//    int start = 0;
//    for(int i = 0;i<sizeof(test_text);i++){
//        count++;
//        if (test_text[i] == 13 && test_text[i+1] == 10){
//            strncpy(rxBuffer,test_text+start,count);
//            start = i+2;
//            count = 0;
//        }
//    }
//    printf("%s", rxBuffer);
    return 0;
}
