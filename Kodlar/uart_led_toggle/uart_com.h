
#ifndef ASCLIN_SHELL_UART_H_
#define ASCLIN_SHELL_UART_H_

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define ENDLINE     "\n\r"

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void initShellInterface(void);
void initLEDInterface(void);
void runShellInterface(void);
void runAPPCheckMsgOK(void);
void runLEDInterface(void);

#endif /* ASCLIN_SHELL_UART_H_ */
