#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>

void lcd_changeColor(char, UART_HandleTypeDef);
void lcd_changeColorRGB(unsigned char r, unsigned char g, unsigned char b, UART_HandleTypeDef huart);
void lcd_showMessage(char * msg, UART_HandleTypeDef huart);
void lcd_clear(UART_HandleTypeDef huart);
void lcd_appendMessage(char * msg, UART_HandleTypeDef huart);
