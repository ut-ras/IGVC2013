/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <stdio.h>
#include <stdarg.h>
int fgetc (FILE * f);
int ferror(FILE * f);
uint8 UARTgetchar(void);
void UARTgetMessage(char * str);
uint16 UARTgets(char * str, int bufferSize);
int UARTprintf(const char * ch, ... );
void UARTputc(const char ch);
void InitializeUART(void);

//[] END OF FILE
